#include "virtio.h"

static void vp_set_status(struct virtio_device *vdev, uint8_t status)
{
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, status);
}

static u8 vp_get_status(struct virtio_device *vdev)
{
	return inportb(vdev->iobase + VIRTIO_PCI_STATUS);
}

static u64 vp_get_features(struct virtio_device *vdev)
{
	return (uint64_t)inportl(vdev->iobase + VIRTIO_PCI_HOST_FEATURES);
}

static void
vp_set_features(struct virtio_device *vdev)
{
	outportl(vdev->iobase + VIRTIO_PCI_GUEST_FEATURES,
		 (u32)vdev->features);
}

/* the notify function used when creating a virt queue */
static bool vp_notify(struct virtqueue *vq)
{
	/* we write the queue's selector into the notification register to
	 * signal the other end */
	iowrite16(vq->index, vq->vdev->iobase + VIRTIO_PCI_QUEUE_NOTIFY);

	return true;
}

static void vp_get(struct virtio_device *vdev, unsigned offset,
		   u8 *buf, unsigned len)
{
	uint32_t ioaddr = vdev->iobase + VIRTIO_PCI_CONFIG_OFF(0) + offset;
	int i;

	for (i = 0; i < len; i++)
		buf[i] = inportb(ioaddr + i);
}

static struct virtqueue *
vp_setup_vq(pci_info_t* pci_info,
	    struct virtio_device *vdev,
	    int index, void (*callback)(struct virtqueue *vq),
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

void
virtio_pci_legacy_init(struct virtio_device *vdev, pci_info_t* pci_info)
{
	vdev->iobase = pci_info->base[0];

	vdev->set_status = vp_set_status;
	vdev->get_status = vp_get_status;
	vdev->get_features = vp_get_features;
	vdev->set_features = vp_set_features;
	vdev->get = vp_get;
	vdev->setup_vq = vp_setup_vq;
}
