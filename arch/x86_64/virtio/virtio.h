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

#ifndef _VIRTIO_H
#define _VIRTIO_H

#include <hermit/stddef.h>

#include <hermit/stddef.h>
#include <hermit/errno.h>
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
typedef struct
{
	struct vring vring;
	uint64_t virt_buffer;
	uint64_t phys_buffer;
	uint16_t last_seen_used;
} virt_queue_t;

struct virtqueue {
	//struct list_head list;
	//void (*]callback)(struct virtqueue *vq);
	//const char *name;
	//struct virtio_device *vdev;
        //unsigned int index;
        unsigned int num_free;
	void *priv;
};

struct virtio_device {
	uint32_t iobase;
	uint64_t features;
	virt_queue_t vq
};

extern int virtio_device_setup(struct virtio_device *vdev);

#define QUEUE_LIMIT		512
#define VIRTIO_BUFFER_SIZE	2048

#endif
