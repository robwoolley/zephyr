/* From FreeBSD sys/bus.h Doesn't mean anything to us */
enum intr_type {
        INTR_TYPE_TTY = 1,
        INTR_TYPE_BIO = 2,
        INTR_TYPE_NET = 4,
        INTR_TYPE_CAM = 8,
        INTR_TYPE_MISC = 16,
        INTR_TYPE_CLK = 32,
        INTR_TYPE_AV = 64,
        INTR_EXCL = 256,                /* exclusive interrupt */
        INTR_MPSAFE = 512,              /* this interrupt is SMP safe */
        INTR_ENTROPY = 1024,            /* this interrupt provides entropy */
        INTR_MD1 = 4096,                /* flag reserved for MD use */
        INTR_MD2 = 8192,                /* flag reserved for MD use */
        INTR_MD3 = 16384,               /* flag reserved for MD use */
        INTR_MD4 = 32768                /* flag reserved for MD use */
};
