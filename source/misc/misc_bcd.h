#ifndef __MISC_BCD_H__
#define __MISC_BCD_H__

#define BCD2DEC(x) ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))

#define DEC2BCD(x) ((((x) / 10) << 4) | ((x) % 10))

#endif /* !__MISC_BCD_H__ */
