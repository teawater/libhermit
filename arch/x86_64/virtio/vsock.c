#include "virtio.h"

#define VENDOR_ID		0x1AF4
#define VIRTIO_ID_VSOCK		19 /* virtio vsock transport */

static struct virtio_device virtio_vsock;
static uint64_t cid;
static struct virtqueue *tx_vq, *rx_vq, *event_vq;

static void get_cid(void)
{
	uint32_t ioaddr = virtio_vsock.iobase + VIRTIO_PCI_CONFIG_OFF(0);
	uint8_t *ptr = (uint8_t *)&cid;
	int i;

	for (i = 0; i < 8; i++)
		ptr[i] = inportb(ioaddr + i);

	LOG_INFO("vsock cid is %ld\n", cid);
}

static void tx_vq_callback(struct virtqueue *vq)
{
}

static void rx_vq_callback(struct virtqueue *vq)
{
}

static void event_vq_callback(struct virtqueue *vq)
{
}

static void virtio_vsock_handler(struct state *s)
{
	uint8_t isr;

	/* reading the ISR has the effect of also clearing it so it's very
	 * important to save off the value. */
	isr = inportb(virtio_vsock.iobase + VIRTIO_PCI_ISR);

	/* It's definitely not us if the ISR was not high */
	if (!isr)
		return;

	vring_interrupt(rx_vq);
}

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

	tx_vq = virtio_setup_vq(&virtio_vsock, 0, tx_vq_callback,
				"vsock_tx_vq", false);
	if (IS_ERR(tx_vq)) {
		ret = PTR_ERR(tx_vq);
		goto out;
	}

	irq_install_handler(pci_info.irq + 32, virtio_vsock_handler);

	rx_vq = virtio_setup_vq(&virtio_vsock, 1, rx_vq_callback,
				"vsock_rx_vq", false);
	if (IS_ERR(rx_vq)) {
		ret = PTR_ERR(rx_vq);
		goto out;
	}

	event_vq = virtio_setup_vq(&virtio_vsock, 2, event_vq_callback,
				"vsock_event_vq", false);
	if (IS_ERR(event_vq)) {
		ret = PTR_ERR(event_vq);
		goto out;
	}

	get_cid();

	virtio_device_ready(&virtio_vsock);

out:
	return ret;
}
