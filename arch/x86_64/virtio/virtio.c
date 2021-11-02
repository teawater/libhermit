#include <hermit/virtio_config.h>
#include <hermit/errno.h>
#include "virtio.h"

/* The alignment to use between consumer and producer parts of vring.
 * x86 pagesize again. */
#define VIRTIO_PCI_VRING_ALIGN		4096

// TODO: just support PCI_LEGACY

static void
virtio_device_init(struct virtio_device *vdev)
{
	// reset interface
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, 0);
	// tell the device that we have noticed it
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	// tell the device that we will support it.
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE|VIRTIO_CONFIG_S_DRIVER);
}

static uint64_t
virtio_device_get_features(struct virtio_device *vdev)
{
	return (uint64_t)inportl(vdev->iobase + VIRTIO_PCI_HOST_FEATURES);
}

static void
virtio_device_set_features(struct virtio_device *vdev)
{
	outportl(vdev->iobase + VIRTIO_PCI_GUEST_FEATURES, (unsigned int)vdev->features);
}

static uint8_t virtio_device_get_status(struct virtio_device *vdev)
{
	return inportb(vdev->iobase + VIRTIO_PCI_STATUS);
}

static void virtio_device_set_status(struct virtio_device *vdev, uint8_t status)
{
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, status);
}

static void virtio_device_add_status(struct virtio_device *vdev, uint8_t status)
{
	virtio_device_set_status(vdev, virtio_device_get_status(vdev) | status);
}

static inline
void virtio_device_ready(struct virtio_device *vdev)
{
	virtio_device_add_status(vdev, VIRTIO_CONFIG_S_DRIVER_OK);
}

/* the notify function used when creating a virt queue */
bool vp_notify(struct virtqueue *vq)
{
	/* we write the queue's selector into the notification register to
	 * signal the other end */
	iowrite16(vq->index, vq->vdev->iobase + VIRTIO_PCI_QUEUE_NOTIFY);

	return true;
}

struct virtqueue *
virtio_setup_vq(struct virtio_device *vdev,
		int index,
		void (*callback)(struct virtqueue *vq),
		const char *name,
		bool ctx)
{
	uint16_t num;
	uint64_t q_pfn;
	struct virtqueue *vq;

	outportw(vdev->iobase + VIRTIO_PCI_QUEUE_SEL, index);

	num = inportw(vdev->iobase + VIRTIO_PCI_QUEUE_NUM);
	if (!num || ioread32(vdev->iobase + VIRTIO_PCI_QUEUE_PFN))
		return ERR_PTR(-ENOENT);

	/* create the vring */
	vq = vring_create_virtqueue_split(index, num,
					  VIRTIO_PCI_VRING_ALIGN, vdev,
					  true, false, ctx,
					  vp_notify, callback, name);
	if (!vq)
		return ERR_PTR(-ENOMEM);

	q_pfn = virtqueue_get_desc_addr(vq) >> VIRTIO_PCI_QUEUE_ADDR_SHIFT;
	if (q_pfn >> 32)
		return ERR_PTR(-E2BIG);

	/* activate the queue */
	iowrite32(q_pfn, vdev->iobase + VIRTIO_PCI_QUEUE_PFN);

	return vq;
}

int
virtio_device_setup(struct virtio_device *vdev)
{
	int ret = 0;

	virtio_device_init(vdev);

	vdev->features = virtio_device_get_features(vdev);

	vdev->features &= ~(1UL << VIRTIO_RING_F_INDIRECT_DESC);
	//XXX: VIRTIO_RING_F_EVENT_IDX is not work, don't know why.
	vdev->features &= ~(1UL << VIRTIO_RING_F_EVENT_IDX);
	virtio_device_set_features(vdev);

	virtio_device_add_status(vdev, VIRTIO_CONFIG_S_FEATURES_OK);
	if (!(virtio_device_get_status(vdev) & VIRTIO_CONFIG_S_FEATURES_OK)) {
		ret = -ENODEV;
		goto out;
	}

	virtio_device_ready(vdev);

out:
	return ret;
}
