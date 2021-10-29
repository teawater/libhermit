/*
 * Copyright (c) 2021 Ant Group
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the University nor the names of its contributors
 *      may be used to endorse or promote products derived from this
 *      software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <hermit/stddef.h>
#include <hermit/stdio.h>
#include <hermit/string.h>
#include <hermit/processor.h>
#include <hermit/mailbox.h>
#include <hermit/logging.h>
#include <hermit/virtio_net.h>
#include <hermit/virtio_ring.h>
#include <hermit/virtio_pci.h>
#include <hermit/virtio_net.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pci.h>

#define VENDOR_ID		0x1AF4
#define VIRTIO_ID_CONSOLE	3 /* virtio console */
#define QUEUE_LIMIT		512
#define VIRTIO_BUFFER_SIZE	2048

#define VIRTIO_CONSOLE_PORT_READY	3

typedef struct
{
	struct vring vring;
	uint64_t virt_buffer;
	uint64_t phys_buffer;
	uint16_t last_seen_used;
} virt_queue_t;

// Use PCI_LEGACY
static int
virtio_feature_setup(uint32_t iobase, uint32_t *features)
{
	uint8_t status;

	// reset interface
	outportb(iobase + VIRTIO_PCI_STATUS, 0);
	// tell the device that we have noticed it
	outportb(iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	// tell the device that we will support it.
	outportb(iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE|VIRTIO_CONFIG_S_DRIVER);

	*features = inportl(iobase + VIRTIO_PCI_HOST_FEATURES);

	// tell the device that the features are OK
	outportb(iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE|VIRTIO_CONFIG_S_DRIVER|VIRTIO_CONFIG_S_FEATURES_OK);

	status = inportb(iobase + VIRTIO_PCI_STATUS);
	if (!(status & VIRTIO_CONFIG_S_FEATURES_OK))
		return ERR_ARG;

	return 0;
}

static int
virtio_queue_setup(uint32_t iobase, int index, virt_queue_t *vq)
{
	unsigned int num;
	size_t total_size;
	int i;

	// determine queue size
	outportw(iobase + VIRTIO_PCI_QUEUE_SEL, index);
	num = inportw(iobase + VIRTIO_PCI_QUEUE_NUM);

	if (num > QUEUE_LIMIT) {
		LOG_INFO("%s: queue num %u is bigger than %u\n", __func__, num, QUEUE_LIMIT);
		return ERR_ARG;
	}

	total_size = vring_size(num, PAGE_SIZE);

	void* vring_base = page_alloc(total_size, VMA_READ | VMA_WRITE | VMA_CACHEABLE);
	if (BUILTIN_EXPECT(!vring_base, 0)) {
		LOG_INFO("%s: Not enough memory to create queue %u\n", __func__, index);
		return ERR_MEM;
	}
	memset((void*)vring_base, 0, total_size);
	vring_init(&vq->vring, num, vring_base, PAGE_SIZE);

	vq->virt_buffer = (uint64_t) page_alloc(num * VIRTIO_BUFFER_SIZE, VMA_READ|VMA_WRITE|VMA_CACHEABLE);
	if (BUILTIN_EXPECT(!vq->virt_buffer, 0)) {
		LOG_INFO("%s: Not enough memory to create buffer %u\n", __func__, index);
		return -1;
	}
	vq->phys_buffer = virt_to_phys(vq->virt_buffer);

	for(i=0; i<num; i++) {
		vq->vring.desc[i].addr = vq->phys_buffer + i * VIRTIO_BUFFER_SIZE;
	}
	outportw(iobase+VIRTIO_PCI_QUEUE_SEL, index);
	outportl(iobase+VIRTIO_PCI_QUEUE_PFN, virt_to_phys((size_t) vring_base) >> PAGE_BITS);

	return 0;
}

static int
virtio_send(virt_queue_t *vq, char *buf, int size)
{
	uint16_t buffer_index;

	for(buffer_index=0; buffer_index<vq->vring.num; buffer_index++) {
		if (!vq->vring.desc[buffer_index].len) {
			break;
		}
	}
	if (BUILTIN_EXPECT(buffer_index >= vq->vring.num, 0))
		return ERR_IF;

	vq->vring.desc[buffer_index].addr = vq->phys_buffer + buffer_index * VIRTIO_BUFFER_SIZE;
	vq->vring.desc[buffer_index].len = size;
	vq->vring.desc[buffer_index].flags = 0;
	vq->vring.desc[buffer_index].next = 0;
	memcpy((void*) (vq->virt_buffer + buffer_index * VIRTIO_BUFFER_SIZE), buf, size);

	uint16_t index = vq->vring.avail->idx % vq->vring.num;
	vq->vring.avail->ring[index] = buffer_index;
	mb();
	vq->vring.avail->idx++;
	mb();

	return 0;
}

#if 0
struct virtio_console_control {
	__virtio32 id;		/* Port number */
	__virtio16 event;	/* The kind of control event (see below) */
	__virtio16 value;	/* Extra information for the key */
};

static int
send_control_msg(virt_queue_t *vq, unsigned int event, unsigned int value)
{
	struct virtio_console_control cpkt;

	// virtio is little endian
	cpkt.id = 0;
	cpkt.event = event;
	cpkt.value = value;

	return virtio_send(vq, &cpkt, sizeof(cpkt));
}
#endif

int
virtio_console_init(void)
{
	int ret = ERR_CONN, i;
	pci_info_t pci_info;
	uint32_t iobase;
	uint32_t features;
	virt_queue_t vq;

	/* Qumranet donated their vendor ID for devices 0x1000 thru 0x10FF. */
	for(i=0; i < 0x1100; i++) {
		if ((pci_get_device_info(VENDOR_ID, i,
					 VIRTIO_ID_CONSOLE << 16 | VENDOR_ID,
					 &pci_info, 1) == 0)) {
			break;
		}
	}
	if (i >= 0x1100)
		goto out;

	ret = virtio_feature_setup(pci_info.base[0], &features);
	if (ret)
		goto out;

	ret = virtio_queue_setup(pci_info.base[0], 1, &vq);
	if (ret)
		goto out;
	
	ret = virtio_send(&vq, "tea test\n", strlen("tea test\n") + 1);
	if (ret)
		goto out;
	outportw(pci_info.base[0]+VIRTIO_PCI_QUEUE_NOTIFY, 1);

out:
	return ret;
}
