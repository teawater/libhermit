#include "virtio.h"

#define VENDOR_ID		0x1AF4
#define VIRTIO_ID_VSOCK		19 /* virtio vsock transport */

static struct virtio_device virtio_vsock;
static uint64_t cid;
static struct virtqueue *tx_vq, *rx_vq, *event_vq;

struct virtio_vsock_hdr {
	__le64	src_cid;
	__le64	dst_cid;
	__le32	src_port;
	__le32	dst_port;
	__le32	len;
	__le16	type;		/* enum virtio_vsock_type */
	__le16	op;		/* enum virtio_vsock_op */
	__le32	flags;
	__le32	buf_alloc;
	__le32	fwd_cnt;
} __attribute__((packed));

#define VIRTIO_VSOCK_DEFAULT_RX_BUF_SIZE	(1024 * 4)

enum {
	VSOCK_VQ_RX     = 0, /* for host to guest data */
	VSOCK_VQ_TX     = 1, /* for guest to host data */
	VSOCK_VQ_EVENT  = 2,
	VSOCK_VQ_MAX    = 3,
};

struct virtio_vsock_pkt {
	struct virtio_vsock_hdr	hdr;
	void *buf;
};

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

	LOG_INFO("virtio_vsock_handler\n");

	/* reading the ISR has the effect of also clearing it so it's very
	 * important to save off the value. */
	isr = inportb(virtio_vsock.iobase + VIRTIO_PCI_ISR);

	/* It's definitely not us if the ISR was not high */
	if (!isr)
		return;

	vring_interrupt(rx_vq);
}

static int virtio_vsock_rx_fill(void)
{
	int ret;
	struct virtio_vsock_pkt *pkt;
	struct scatterlist hdr, buf, *sgs[2];

	do {
		pkt = kmalloc(sizeof(*pkt));
		if (!pkt) {
			ret = -ENOMEM;
			goto out;
		}
		memset(pkt, 0, sizeof(*pkt));
		pkt->buf = page_alloc(VIRTIO_VSOCK_DEFAULT_RX_BUF_SIZE, VMA_READ|VMA_WRITE|VMA_CACHEABLE);
		if (!pkt->buf) {
			ret = -ENOMEM;
			goto out;
		}

		sg_init_one(&hdr, &pkt->hdr, sizeof(pkt->hdr));
		sgs[0] = &hdr;

		sg_init_one(&buf, pkt->buf, VIRTIO_VSOCK_DEFAULT_RX_BUF_SIZE);
		sgs[1] = &buf;

		ret = virtqueue_add_split(rx_vq, sgs, 2, 0, 2, pkt, NULL);
		if (ret)
			goto out;

	} while (rx_vq->num_free);
	virtqueue_kick(rx_vq);

out:
	return ret;
}

int
virtio_vsock_init(void)
{
	int ret;
	pci_info_t pci_info;
	bool is_legacy;

	ret = virtio_device_find(&pci_info, &is_legacy, VENDOR_ID, VIRTIO_ID_VSOCK);
	if (ret)
		goto out;

	virtio_vsock.iobase = pci_info.base[0];
	
	ret = virtio_device_setup(&virtio_vsock, &pci_info, is_legacy);
	if (ret)
		goto out;

	rx_vq = virtio_setup_vq(&virtio_vsock, VSOCK_VQ_RX, rx_vq_callback,
				"vsock_rx_vq", false);
	if (IS_ERR(rx_vq)) {
		ret = PTR_ERR(rx_vq);
		goto out;
	}

	tx_vq = virtio_setup_vq(&virtio_vsock, VSOCK_VQ_TX, tx_vq_callback,
				"vsock_tx_vq", false);
	if (IS_ERR(tx_vq)) {
		ret = PTR_ERR(tx_vq);
		goto out;
	}

	event_vq = virtio_setup_vq(&virtio_vsock, VSOCK_VQ_EVENT, event_vq_callback,
				"vsock_event_vq", false);
	if (IS_ERR(event_vq)) {
		ret = PTR_ERR(event_vq);
		goto out;
	}

	ret = virtio_vsock_rx_fill();
	if (ret)
		goto out;

	get_cid();

	irq_install_handler(pci_info.irq + 32, virtio_vsock_handler);
	LOG_INFO("vsock irq is %d\n", pci_info.irq);

	virtio_device_ready(&virtio_vsock);

out:
	return ret;
}
