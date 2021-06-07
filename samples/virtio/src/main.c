#include <zephyr.h>
#include <drivers/virtio/rng/rng.h>

#if defined(CONFIG_VIRTIO_PCI) && defined(CONFIG_VIRTIO_RNG)

int virtio_test_rng_device(pcie_bdf_t bdf)
{
	int err = 1;
	uint32_t *data = NULL;
	size_t size = 0;

	// virtio_rng_dev_t dev;
	device_t dev = (device_t) device_get_binding("virtio_rng");

	if (dev == NULL) {
		printk("Failed to find virtio RNG device\n");
		goto fail;
	}

	data = k_malloc(sizeof(uint32_t) * 5);
	size = sizeof(uint32_t) * 5;

	err = virtio_rng_read_data(dev, data, &size);
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

	err = 0;
fail:
	if (data != NULL)
		k_free(data);
	return err;
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
