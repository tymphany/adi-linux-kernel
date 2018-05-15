/*
 * Copyright (C) 2018 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#ifndef __MACH_SEC_H
#define __MACH_SEC_H

void sec_init(void __iomem *, void __iomem *, void __iomem *);
void sec_raise_irq(unsigned int irq);
void sec_enable_sci(unsigned int sid, bool fault);
void sec_enable_ssi(unsigned int sid);
void sec_set_ssi_coreid(unsigned int sid, unsigned int coreid);

#endif /* __MACH_SEC_H */
