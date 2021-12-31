#include <hermit/virtio_config.h>
#include <hermit/errno.h>
#include "virtio.h"

/* The alignment to use between consumer and producer parts of vring.
 * x86 pagesize again. */
#define VIRTIO_PCI_VRING_ALIGN		4096

static void virtio_device_set_status(struct virtio_device *vdev, uint8_t status)
{
	vdev->set_status(vdev, status);
}

static uint8_t virtio_device_get_status(struct virtio_device *vdev)
{
	return vdev->get_status(vdev);
}

static void virtio_device_add_status(struct virtio_device *vdev, uint8_t status)
{
	virtio_device_set_status(vdev, virtio_device_get_status(vdev) | status);
}

static uint64_t
virtio_device_get_features(struct virtio_device *vdev)
{
	return vdev->get_features(vdev);
}

static void
virtio_device_set_features(struct virtio_device *vdev)
{
	vdev->set_features(vdev);
}

void virtio_device_ready(struct virtio_device *vdev)
{
	virtio_device_add_status(vdev, VIRTIO_CONFIG_S_DRIVER_OK);
}

static void
virtio_device_init(struct virtio_device *vdev)
{
	// reset interface
	virtio_device_set_status(vdev, 0);
	// tell the device that we have noticed it
	virtio_device_set_status(vdev, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	// tell the device that we will support it.
	virtio_device_set_status(vdev, VIRTIO_CONFIG_S_ACKNOWLEDGE|VIRTIO_CONFIG_S_DRIVER);
}

struct virtqueue *
virtio_setup_vq(pci_info_t* pci_info,
		struct virtio_device *vdev,
		int index,
		void (*callback)(struct virtqueue *vq),
		const char *name,
		bool ctx)
{
	return vdev->setup_vq(pci_info, vdev, index, callback, name, ctx);
}

int
virtio_device_setup(struct virtio_device *vdev, pci_info_t* pci_info, bool is_legacy)
{
	int ret = 0;

	if (is_legacy)
		virtio_pci_legacy_init(vdev, pci_info);
	else {
		ret = virtio_pci_modern_init(vdev, pci_info);
		if (ret)
			goto out;
	}

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

out:
	return ret;
}

int
virtio_device_find(pci_info_t *pci_info, bool *is_legacy, uint32_t vendor_id, uint32_t device_id)
{
	int i, ret = 0;

	/* Qumranet donated their vendor ID for devices 0x1000 thru 0x10FF. */
	for(i = 0; i < 0x1040; i++) {
		if (pci_get_device_info(vendor_id, i,
					 device_id << 16 | vendor_id,
					 pci_info, 1, true) == 0) {
			*is_legacy = true;
			pci_legacy_init(pci_info);
			goto out;
		}
	}
	if (i >= 0x1040 && pci_get_device_info(vendor_id, 0x1040 + device_id, PCI_IGNORE_SUBID, pci_info, 1, true) == 0) {
		*is_legacy = false;
		pci_modern_init(pci_info);
		goto out;
	}

	ret = -ENXIO;
out:
	return ret;
}
