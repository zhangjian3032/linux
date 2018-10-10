/*
 * Copyright (c) 2010-2015, Emulex Corporation.
 * Modifications made by Emulex Corporation under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/tty.h>

extern void power_putstr(const char *ptr);
void fb_printf(const char *fmt, ...)
{
    static char buf[1024];
    va_list args;
    int r;


    va_start(args, fmt);

    r = vsnprintf(buf, sizeof(buf), fmt, args);
    power_putstr(buf);

  va_end(args);

}



int fb_initialized = 0;

void fb_init(void)
{
    fb_initialized = 1;
}

