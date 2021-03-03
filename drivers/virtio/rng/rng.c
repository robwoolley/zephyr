/*
 * Copyright (c) 2021, Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification, are
 * permitted provided that the following conditions are met:
 *
 * 1) Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be
 * used to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <drivers/virtio/rng/rng.h>

LOG_MODULE_REGISTER(virtio_rng, CONFIG_VIRTIO_RNG_LOG_LEVEL);

static sys_slist_t rng_gen_sg;

static int virtio_rng_allocate_virtqueues(virtio_rng_dev_t *dev)
{
	struct vq_alloc_info vq_info;

	VQ_ALLOC_INFO_INIT(&vq_info, 0, NULL, dev, &dev->rng_vq, "RNG VQ");
	return dev->virtio_dev.api->alloc_virtqueues(&dev->virtio_dev, 0, 1,
							&vq_info);
}

int virtio_rng_init(virtio_rng_dev_t *dev, virtio_bus_type_t type, void *dat)
{
	int error;

	if (type == VIRTIO_PCI_DEVICE) {
#ifdef CONFIG_VIRTIO_PCI
		pcie_bdf_t bdf = *(pcie_bdf_t *)dat;

		error = virtio_pci_init(&dev->virtio_dev, bdf);
#else
		LOG_ERR("Please enable CONFIG_VIRTIO_PCI to use PCI functions");
		return -ENOTSUP;
#endif
	} else {
		LOG_ERR("MMIO devices are not currently supported");
		return -ENOTSUP;
	}

	if (error) {
		LOG_ERR("Failed to initialize virtio device");
		return error;
	}

	error = dev->virtio_dev.api->negotiate_feature(&dev->virtio_dev, 0x0);
	if (error) {
		LOG_ERR("Failed to negotiate features");
		return error;
	}

	error = virtio_rng_allocate_virtqueues(dev);
	if (error) {
		LOG_ERR("Failed to allocate virtqueues");
		return error;
	}

	sys_slist_init(&rng_gen_sg);

	dev->rng_read = virtio_rng_read_data;

	return 0;
}

int virtio_rng_read_data(virtio_rng_dev_t *dev, void *out, size_t *len)
{
	virtio_data_t data;

	data.addr = out;
	data.size = *len;
	sys_slist_append(&rng_gen_sg, &data.next);

	int error = virtqueue_enqueue(dev->rng_vq, out, NULL, 0, &rng_gen_sg, 1);

	if (error) {
		LOG_ERR("Failed to queue the rng virtqueue");
		goto fail;
	}

	virtqueue_notify(dev->rng_vq);
	void *cookie = virtqueue_poll(dev->rng_vq, (uint32_t *)len);

	if (cookie != out) {
		LOG_WRN("Virtqueue poll received invalid cookie");
	}


fail:
	sys_slist_remove(&rng_gen_sg, NULL, &data.next);

	return error;
}
