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
#include <hermit/errno.h>
#include <hermit/stdio.h>
#include <hermit/string.h>
#include <hermit/processor.h>
#include <hermit/mailbox.h>
#include <hermit/logging.h>
#include <hermit/virtio_net.h>
#include <hermit/virtio_ring.h>
//#include <hermit/virtio_pci.h>
#include <hermit/virtio_net.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/pci.h>
#include "virtio.h"

#define VENDOR_ID		0x1AF4
#define VIRTIO_ID_CONSOLE	3 /* virtio console */

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

struct virtio_device virtio_console;

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
	virtio_console.iobase = pci_info.base[0];

	ret = virtio_device_setup(&virtio_console);
	if (ret)
		goto out;
	
	ret = virtio_send(&virtio_console.vq, "tea test\n", strlen("tea test\n") + 1);
	if (ret)
		goto out;
	outportw(pci_info.base[0]+VIRTIO_PCI_QUEUE_NOTIFY, 1);

out:
	return ret;
}
