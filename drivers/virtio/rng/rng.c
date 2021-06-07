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

DEVICE_DECLARE(virtio_rng);

struct virtio_dev_data virtio_rng_data;

struct virtio_rng_cfg {
	void *config;
};

#if 0
struct virtio_rng_api {
	int (*negotiate_feature)(device_t dev, uint64_t features);
	bool (*with_feature)(device_t dev, uint64_t feature);
	int (*alloc_virtqueues)(device_t dev, int flags, int num,
					struct vq_alloc_info *info);
	int (*setup_interrupts)(device_t dev, int prio, int flags);
	void (*stop)(device_t dev);
	int (*reinit)(device_t dev, uint64_t features);
	void (*reinit_complete)(device_t dev);
	void (*notify_virtqueue)(device_t dev, uint16_t vq,
					unsigned long offset);
	int (*read_config)(device_t dev, uint32_t offset, void *dat,
					size_t len);
	int (*write_config)(device_t dev, uint32_t offset, void *dat,
							size_t len);
	void (*config_change)(const device_t dev);
	int (*rng_read)(device_t dev, void *out, size_t *len);
};

static const struct virtio_rng_api virtio_rng_api = {
	.rng_read = virtio_rng_read_data,
};
#endif /* 0 */
static int virtio_rng_allocate_virtqueues(device_t dev)
{
	struct vq_alloc_info vq_info;
	const struct __virtio_bus_interface__ *api = dev->api;
	struct virtio_dev_data *data = dev->data;

	VQ_ALLOC_INFO_INIT(&vq_info, 0, NULL, dev, &data->rng_vq, "RNG VQ");
	return api->alloc_virtqueues(dev, 0, 1, &vq_info);
}

int virtio_rng_init(const struct device *ddev)
{
	int error;
	device_t dev = (device_t) ddev;
	const struct __virtio_bus_interface__ *api;

	virtio_bus_type_t type = VIRTIO_PCI_DEVICE;

	if (type == VIRTIO_PCI_DEVICE) {
#ifdef CONFIG_VIRTIO_PCI
		const pcie_bdf_t bdf = PCIE_BDF(0, 3, 0);

		error = virtio_pci_init(dev, bdf);
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

	api = dev->api;

	error = api->negotiate_feature(dev, 0x0);
	if (error) {
		LOG_ERR("Failed to negotiate features");
		return error;
	}

	error = virtio_rng_allocate_virtqueues(dev);
	if (error) {
		LOG_ERR("Failed to allocate virtqueues");
		return error;
	}

	return 0;
}

int virtio_rng_read_data(device_t dev, void *out, size_t *len)
{
	struct sglist_seg segs[1];
	struct sglist sg;
	int error;
	void * cookie;
	struct virtio_dev_data *data = dev->data;

	sglist_init(&sg, 1, segs);

	error = sglist_append(&sg, out, *len);
	__ASSERT(error == 0,
		("%s: error %d adding buffer to sglist", __func__, error));

	error = virtqueue_enqueue(data->rng_vq, out, &sg, 0, 1);
	if (error) {
		LOG_ERR("Failed to queue the rng virtqueue");
		return (error);
	}

	virtqueue_notify(data->rng_vq);

	cookie = virtqueue_poll(data->rng_vq, (uint32_t *)len);
	if (cookie != out) {
		LOG_WRN("Virtqueue poll received invalid cookie");
	}

	return 0;
}


DEVICE_DEFINE(virtio_rng, "virtio_rng", virtio_rng_init, NULL,
              &virtio_rng_data, NULL, POST_KERNEL,
              CONFIG_APPLICATION_INIT_PRIORITY, NULL);