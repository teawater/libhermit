#include "virtio.h"

static void vp_set_status(struct virtio_device *vdev, uint8_t status)
{
	outportb(status, vdev->iobase + VIRTIO_PCI_STATUS);
}

void
virtio_pci_legacy_init(struct virtio_device *vdev, pci_info_t* pci_info)
{
	vdev->iobase = pci_info->base[0];
	vdev->set_status = vp_set_status;
}
