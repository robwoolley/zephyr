/* defines */
/* extra malloc flags that will be mostly ignored */
/* M_NOWAIT/M_WAITOK conflict with VxWorks mbuf.h */
/* #define M_NOWAIT        0x0001          do not block */
/* #define M_WAITOK        0x0002          ok to block */
#define M_ZERO          0x0100          /* bzero the allocation */
#define M_NOVM          0x0200          /* don't ask VM for pages */
#define M_USE_RESERVE   0x0400          /* can alloc out of reserve memory */
#define M_NODUMP        0x0800          /* don't dump pages in this allocation */
#define M_FIRSTFIT      0x1000          /* Only for vmem, fast fit. */
#define M_BESTFIT       0x2000          /* Only for vmem, low fragmentation. */

#ifndef M_DONTWAIT
#define M_DONTWAIT      1
#endif

#define M_FREE          0       /* should be on free list */
#define M_NOWAIT        M_DONTWAIT
#define M_MBUF          2       /* mbuf */
#define M_DEVBUF        3       /* device driver memory */
#define M_TEMP          4       /* temporary memory */

/* Replacement malloc/free */
#define malloc(A, B, C)  (((C) & M_ZERO) ? k_calloc(1,(A)) : k_malloc(A))
#define free(A, B)       k_free(A)
