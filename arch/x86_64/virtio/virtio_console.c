#include "virtio.h"

#define VENDOR_ID		0x1AF4
#define VIRTIO_ID_CONSOLE	3 /* virtio console */

static struct virtio_device virtio_console;
static struct virtqueue *output_vq = NULL;
spinlock_irqsave_t virtio_console_output_lock = SPINLOCK_IRQSAVE_INIT;

// Don't need handle irq because doesn't support non-block mode
#if 0
static void flush_bufs(struct virtqueue *vq)
{
	unsigned int len;

	while (virtqueue_get_buf(output_vq, &len)
		&& !virtqueue_is_broken(output_vq));
}

static void output_vq_callback(struct virtqueue *vq)
{
	flush_bufs(vq);
}

static void virtio_console_handler(struct state *s)
{
	uint8_t isr;

	spinlock_irqsave_lock(&virtio_console_output_lock);

	if (output_vq)
		goto out;

	/* reading the ISR has the effect of also clearing it so it's very
	 * important to save off the value. */
	isr = inportb(virtio_console.iobase + VIRTIO_PCI_ISR);

	/* It's definitely not us if the ISR was not high */
	if (!isr)
		return;

	vring_interrupt(output_vq);

out:
	spinlock_irqsave_unlock(&virtio_console_output_lock);
}
#endif
static void output_vq_callback(struct virtqueue *vq)
{
}
static void virtio_console_handler(struct state *s)
{
}

/* Caller should hold virtio_console_output_lock */
int
virtio_console_putchars(char *buf, int size)
{
	struct scatterlist sg[1];
	unsigned int len;

	sg_init_one(sg, buf, size);

	spinlock_irqsave_lock(&virtio_console_output_lock);

	if (!output_vq)
		goto out;

	//flush_bufs(output_vq);

	virtqueue_add_outbuf(output_vq, sg, 1, &virtio_console);

	virtqueue_kick(output_vq);

	// Must wait to ensure the order of the output.
	while (!virtqueue_get_buf(output_vq, &len)
		&& !virtqueue_is_broken(output_vq));

out:
	spinlock_irqsave_unlock(&virtio_console_output_lock);
	return size;
}

int
virtio_console_init(void)
{
	int ret;
	pci_info_t pci_info;
	bool is_legacy;

	ret = virtio_device_find(&pci_info, &is_legacy, VENDOR_ID, VIRTIO_ID_CONSOLE);
	if (ret)
		goto out;

	virtio_console.iobase = pci_info.base[0];

	ret = virtio_device_setup(&virtio_console, &pci_info, is_legacy);
	if (ret)
		goto out;

	irq_install_handler(pci_info.irq + 32, virtio_console_handler);

	spinlock_irqsave_lock(&virtio_console_output_lock);
	// Don't have input_vq because cannot get data from qemu.
	output_vq = virtio_setup_vq(&virtio_console, 1,
				    output_vq_callback,
				    "virtio_console_output_vq",
				    false);
	if (IS_ERR(output_vq))
		ret = PTR_ERR(output_vq);
	spinlock_irqsave_unlock(&virtio_console_output_lock);

	virtio_device_ready(&virtio_console);

out:
	return ret;
}
