typedef unsigned short u_short;
typedef unsigned int u_int;
typedef mem_addr_t vm_paddr_t;
typedef mem_addr_t vm_offset_t;

#define	    PAGE_SIZE		CONFIG_MMU_PAGE_SIZE
#define     PAGE_MASK           (CONFIG_MMU_PAGE_SIZE - 1)
#define     round_page(x)       (((unsigned long)(x) + PAGE_MASK) & ~PAGE_MASK)
#define     trunc_page(x)       ((unsigned long)(x) & ~PAGE_MASK)
