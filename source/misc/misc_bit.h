#ifndef __MISC_BIT_H__
#define __MISC_BIT_H__

#ifndef BIT_SET
#define BIT_SET(a, b) (a) |= (b)
#endif

#ifndef BIT_CLR
#define BIT_CLR(a, b) (a) &= (~(b))
#endif

#ifndef BIT
#define BIT(x) (1UL << (x))
#endif

#endif /* !__MISC_BIT_H__ */
