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

//#include <hermit/virtio_pci.h>
#include <hermit/virtio_config.h>
#include <hermit/errno.h>
#include "virtio.h"

/* The alignment to use between consumer and producer parts of vring.
 * x86 pagesize again. */
#define VIRTIO_PCI_VRING_ALIGN		4096

// TODO: just support PCI_LEGACY

static void
virtio_device_init(struct virtio_device *vdev)
{
	// reset interface
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, 0);
	// tell the device that we have noticed it
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE);
	// tell the device that we will support it.
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, VIRTIO_CONFIG_S_ACKNOWLEDGE|VIRTIO_CONFIG_S_DRIVER);
}

static uint64_t
virtio_device_get_features(struct virtio_device *vdev)
{
	return inportl(vdev->iobase + VIRTIO_PCI_HOST_FEATURES);
}

static uint8_t virtio_device_get_status(struct virtio_device *vdev)
{
	return inportb(vdev->iobase + VIRTIO_PCI_STATUS);
}

static void virtio_device_set_status(struct virtio_device *vdev, uint8_t status)
{
	outportb(vdev->iobase + VIRTIO_PCI_STATUS, status);
}

static void virtio_device_add_status(struct virtio_device *vdev, uint8_t status)
{
	virtio_device_set_status(vdev, virtio_device_get_status(vdev) | status);
}

static inline
void virtio_device_ready(struct virtio_device *vdev)
{
	virtio_device_add_status(vdev, VIRTIO_CONFIG_S_DRIVER_OK);
}

static struct virtqueue *vring_create_virtqueue_split(unsigned int index, unsigned int num, unsigned int vring_align)
{
	void *queue = NULL;
	uint64_t dma_addr;
	size_t queue_size_in_bytes;
	struct vring vring;
	struct virtqueue *vq;

	if (num & (num - 1))
		return NULL;
	
	// TODO: the size should PAGE_ALIGN
	queue = page_alloc(vring_size(num, vring_align), VMA_READ | VMA_WRITE | VMA_CACHEABLE);
	if (!queue)
		return NULL;
	dma_addr = virt_to_phys(queue);

	queue_size_in_bytes = vring_size(num, vring_align);
	vring_init(&vring, num, queue, vring_align);

	vq = __vring_new_virtqueue(index, vring, vdev, weak_barriers, context,
				   notify, callback, name);
	if (!vq)
		return NULL;
	
}

static int
setup_vq(struct virtio_device *vdev, int index)
{
	uint16_t num;

	outportw(vdev->iobase + VIRTIO_PCI_QUEUE_SEL, index);

	num = inportw(vdev->iobase + VIRTIO_PCI_QUEUE_NUM);
	if (num == 0)
		return -ENOENT;

	virt_to_phys(vq->virt_buffer);
}

#define QUEUE_LIMIT		512
#define VIRTIO_BUFFER_SIZE	2048

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

int
virtio_device_setup(struct virtio_device *vdev)
{
	int ret = 0;

	virtio_device_init(vdev);

	vdev->features = virtio_device_get_features(vdev);

	// TODO: should check VIRTIO_F_RING_PACKED if not support

	// TODO: doesn't check features and set driver features
	//virtio_device_add_status(vdev, VIRTIO_CONFIG_S_FEATURES_OK);

	//ret = virtio_queue_setup(vdev->iobase, 1, &vdev->vq);
	
	if (ret)
		goto out;

	virtio_device_ready(vdev);

	// TODO: enable driver
	//virtio_config_enable(dev);

out:
	return ret;
}
