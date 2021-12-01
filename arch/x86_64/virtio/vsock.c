#include "virtio.h"

#define VENDOR_ID		0x1AF4
#define VIRTIO_ID_VSOCK		19 /* virtio vsock transport */

static struct virtio_device virtio_vsock;

int
virtio_vsock_init(void)
{
	int ret;
	pci_info_t pci_info;

	ret = virtio_device_find(&pci_info, VENDOR_ID, VIRTIO_ID_VSOCK);
	if (ret)
		goto out;

	virtio_vsock.iobase = pci_info.base[0];

	ret = virtio_device_setup(&virtio_vsock);
	if (ret)
		goto out;

out:
	return ret;
}
