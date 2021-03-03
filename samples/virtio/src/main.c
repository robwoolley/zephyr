#include <zephyr.h>
#include <drivers/virtio/rng/rng.h>

#if defined(CONFIG_VIRTIO_PCI) && defined(CONFIG_VIRTIO_RNG)

void virtio_test_rng_device(pcie_bdf_t bdf)
{
	int err;
	virtio_rng_dev_t dev;

	err = virtio_rng_init(&dev, VIRTIO_PCI_DEVICE, &bdf);
	if (err) {
		printk("Failed to initialize Virtio RNG device(%d)\n", err);
		return;
	}

	uint32_t *data = k_malloc(sizeof(uint32_t) * 5);
	size_t size = sizeof(uint32_t) * 5;

	err = dev.rng_read(&dev, data, &size);

	if (err) {
		printk("Failed to read RNG device(%d)\n", err);
		goto fail;
	}

	if (size != sizeof(uint32_t) * 5) {
		printk("RNG device only returned %ld/%ld bytes\n",
				(unsigned long)size,
				(unsigned long)(sizeof(uint32_t) * 5));
		goto fail;
	}

	for (int i = 0; i < 5; ++i) {
		printk("Random Number: 0x%x\n", data[i]);
	}

fail:
	k_free(data);
}

void main(void)
{
	VIRTIO_PCI_FOREACH(device) {
		if (device->device_type == VIRTIO_ID_ENTROPY) {
			virtio_test_rng_device(device->bdf);
		}
	}
}

#else

void main(void)
{
	printk("Currently virtio only supports a PCI backend, please "
		"enable \"CONFIG_VIRTIO_PCI\" or if it is enabled, enable "
		"\"CONFIG_VIRTIO_RNG\"\n");
}

#endif
