#ifndef __MISC_H__
#define __MISC_H__

#include "misc_bcd.h"
#include "misc_binary.h"
#include "misc_bit.h"
#include "misc_compare.h"

#define __TCMRAM __attribute__((section(".tcmram")))
#define __RAM1 __attribute__((section(".ram1")))

#endif /* !__MISC_H__ */
