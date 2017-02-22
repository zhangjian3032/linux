#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/mach/map.h>

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/dmi.h>
#include <linux/io.h>

static char *mode_option;

struct astvga_par {
	struct platform_device 	*pdev;
	struct fb_info			*info;
	void __iomem *video_base;
	u32 pseudo_palette[17];
	
};

/*
 * Here we define the default structs fb_fix_screeninfo and fb_var_screeninfo
 * if we don't use modedb. If we do use modedb see astvgafb_init how to use it
 * to get a fb_var_screeninfo. Otherwise define a default var as well. 
 */
static struct fb_fix_screeninfo astvgafb_fix = {
	.id =		"FB's name", 
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_PSEUDOCOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1, 
	.accel =	FB_ACCEL_NONE,
};

    /*
     * 	Modern graphical hardware not only supports pipelines but some 
     *  also support multiple monitors where each display can have its  
     *  its own unique data. In this case each display could be  
     *  represented by a separate framebuffer device thus a separate 
     *  struct fb_info. Now the struct astvga_par represents the graphics
     *  hardware state thus only one exist per card. In this case the 
     *  struct astvga_par for each graphics card would be shared between 
     *  every struct fb_info that represents a framebuffer on that card. 
     *  This allows when one display changes it video resolution (info->var) 
     *  the other displays know instantly. Each display can always be
     *  aware of the entire hardware state that affects it because they share
     *  the same astvga_par struct. The other side of the coin is multiple
     *  graphics cards that pass data around until it is finally displayed
     *  on one monitor. Such examples are the voodoo 1 cards and high end
     *  NUMA graphics servers. For this case we have a bunch of pars, each
     *  one that represents a graphics state, that belong to one struct 
     *  fb_info. Their you would want to have *par point to a array of device
     *  states and have each struct fb_ops function deal with all those 
     *  states. I hope this covers every possible hardware design. If not
     *  feel free to send your ideas at jsimmons@users.sf.net 
     */

    /*
     *  If your driver supports multiple boards or it supports multiple 
     *  framebuffers, you should make these arrays, or allocate them 
     *  dynamically using framebuffer_alloc() and free them with
     *  framebuffer_release().
     */ 
static struct fb_info info;

    /* 
     * Each one represents the state of the hardware. Most hardware have
     * just one hardware state. These here represent the default state(s). 
     */
static struct astvga_par __initdata current_par;

/**
 *      astvgafb_check_var - Optional function. Validates a var passed in. 
 *      @var: frame buffer variable screen structure
 *      @info: frame buffer structure that represents a single frame buffer 
 *
 *	Checks to see if the hardware supports the state requested by
 *	var passed in. This function does not alter the hardware state!!! 
 *	This means the data stored in struct fb_info and struct astvga_par do 
 *      not change. This includes the var inside of struct fb_info. 
 *	Do NOT change these. This function can be called on its own if we
 *	intent to only test a mode and not actually set it. The stuff in 
 *	modedb.c is a example of this. If the var passed in is slightly 
 *	off by what the hardware can support then we alter the var PASSED in
 *	to what we can do.
 *
 *      For values that are off, this function must round them _up_ to the
 *      next value that is supported by the hardware.  If the value is
 *      greater than the highest value supported by the hardware, then this
 *      function must return -EINVAL.
 *
 *      Exception to the above rule:  Some drivers have a fixed mode, ie,
 *      the hardware is already set at boot up, and cannot be changed.  In
 *      this case, it is more acceptable that this function just return
 *      a copy of the currently working var (info->var). Better is to not
 *      implement this function, as the upper layer will do the copying
 *      of the current var for you.
 *
 *      Note:  This is the only function where the contents of var can be
 *      freely adjusted after the driver has been registered. If you find
 *      that you have code outside of this function that alters the content
 *      of var, then you are doing something wrong.  Note also that the
 *      contents of info->var must be left untouched at all times after
 *      driver registration.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int astvgafb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
    /* ... */
	printk("astvgafb_check_var \n");
    return 0;	   	
}

/**
 *      astvgafb_set_par - Optional function. Alters the hardware state.
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *	Using the fb_var_screeninfo in fb_info we set the resolution of the
 *	this particular framebuffer. This function alters the par AND the
 *	fb_fix_screeninfo stored in fb_info. It doesn't not alter var in 
 *	fb_info since we are using that data. This means we depend on the
 *	data in var inside fb_info to be supported by the hardware. 
 *
 *      This function is also used to recover/restore the hardware to a
 *      known working state.
 *
 *	astvgafb_check_var is always called before astvgafb_set_par to ensure that
 *      the contents of var is always valid.
 *
 *	Again if you can't change the resolution you don't need this function.
 *
 *      However, even if your hardware does not support mode changing,
 *      a set_par might be needed to at least initialize the hardware to
 *      a known working state, especially if it came back from another
 *      process that also modifies the same hardware, such as X.
 *
 *      If this is the case, a combination such as the following should work:
 *
 *      static int astvgafb_check_var(struct fb_var_screeninfo *var,
 *                                struct fb_info *info)
 *      {
 *              *var = info->var;
 *              return 0;
 *      }
 *
 *      static int astvgafb_set_par(struct fb_info *info)
 *      {
 *              init your hardware here
 *      }
 *
 *	Returns negative errno on error, or zero on success.
 */
static int astvgafb_set_par(struct fb_info *info)
{
    struct astvga_par *par = info->par;
    /* ... */
	printk("astvgafb_set_par \n");
    return 0;	
}

/**
 *  	astvgafb_setcolreg - Optional function. Sets a color register.
 *      @regno: Which register in the CLUT we are programming 
 *      @red: The red value which can be up to 16 bits wide 
 *	@green: The green value which can be up to 16 bits wide 
 *	@blue:  The blue value which can be up to 16 bits wide.
 *	@transp: If supported, the alpha value which can be up to 16 bits wide.
 *      @info: frame buffer info structure
 * 
 *  	Set a single color register. The values supplied have a 16 bit
 *  	magnitude which needs to be scaled in this function for the hardware. 
 *	Things to take into consideration are how many color registers, if
 *	any, are supported with the current color visual. With truecolor mode
 *	no color palettes are supported. Here a pseudo palette is created
 *	which we store the value in pseudo_palette in struct fb_info. For
 *	pseudocolor mode we have a limited color palette. To deal with this
 *	we can program what color is displayed for a particular pixel value.
 *	DirectColor is similar in that we can program each color field. If
 *	we have a static colormap we don't need to implement this function. 
 * 
 *	Returns negative errno on error, or zero on success.
 */
static int astvgafb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
	printk("astvgafb_setcolreg \n");
}

/**
 *      astvgafb_blank - NOT a required function. Blanks the display.
 *      @blank_mode: the blank mode we want. 
 *      @info: frame buffer structure that represents a single frame buffer
 *
 *      Blank the screen if blank_mode != FB_BLANK_UNBLANK, else unblank.
 *      Return 0 if blanking succeeded, != 0 if un-/blanking failed due to
 *      e.g. a video mode which doesn't support it.
 *
 *      Implements VESA suspend and powerdown modes on hardware that supports
 *      disabling hsync/vsync:
 *
 *      FB_BLANK_NORMAL = display is blanked, syncs are on.
 *      FB_BLANK_HSYNC_SUSPEND = hsync off
 *      FB_BLANK_VSYNC_SUSPEND = vsync off
 *      FB_BLANK_POWERDOWN =  hsync and vsync off
 *
 *      If implementing this function, at least support FB_BLANK_UNBLANK.
 *      Return !0 for any modes that are unimplemented.
 *
 */
static int astvgafb_blank(int blank_mode, struct fb_info *info)
{
    /* ... */
	printk("astvgafb_blank \n");
    return 0;
}

/* ------------ Accelerated Functions --------------------- */

/*
 * We provide our own functions if we have hardware acceleration
 * or non packed pixel format layouts. If we have no hardware 
 * acceleration, we can use a generic unaccelerated function. If using
 * a pack pixel format just use the functions in cfb_*.c. Each file 
 * has one of the three different accel functions we support.
 */

/**
 *      astvgafb_fillrect - REQUIRED function. Can use generic routines if 
 *		 	 non acclerated hardware and packed pixel based.
 *			 Draws a rectangle on the screen.		
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *	@region: The structure representing the rectangular region we 
 *		 wish to draw to.
 *
 *	This drawing operation places/removes a retangle on the screen 
 *	depending on the rastering operation with the value of color which
 *	is in the current color depth format.
 */
void astvgafb_fillrect(struct fb_info *p, const struct fb_fillrect *region)
{
	printk("astvgafb_fillrect \n");
/*	Meaning of struct fb_fillrect
 *
 *	@dx: The x and y corrdinates of the upper left hand corner of the 
 *	@dy: area we want to draw to. 
 *	@width: How wide the rectangle is we want to draw.
 *	@height: How tall the rectangle is we want to draw.
 *	@color:	The color to fill in the rectangle with. 
 *	@rop: The raster operation. We can draw the rectangle with a COPY
 *	      of XOR which provides erasing effect. 
 */
}

/**
 *      astvgafb_copyarea - REQUIRED function. Can use generic routines if
 *                       non acclerated hardware and packed pixel based.
 *                       Copies one area of the screen to another area.
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *      @area: Structure providing the data to copy the framebuffer contents
 *	       from one region to another.
 *
 *      This drawing operation copies a rectangular area from one area of the
 *	screen to another area.
 */
void astvgafb_copyarea(struct fb_info *p, const struct fb_copyarea *area) 
{
	printk("astvgafb_copyarea \n");
/*
 *      @dx: The x and y coordinates of the upper left hand corner of the
 *	@dy: destination area on the screen.
 *      @width: How wide the rectangle is we want to copy.
 *      @height: How tall the rectangle is we want to copy.
 *      @sx: The x and y coordinates of the upper left hand corner of the
 *      @sy: source area on the screen.
 */
}


/**
 *      astvgafb_imageblit - REQUIRED function. Can use generic routines if
 *                        non acclerated hardware and packed pixel based.
 *                        Copies a image from system memory to the screen. 
 *
 *      @info: frame buffer structure that represents a single frame buffer
 *	@image:	structure defining the image.
 *
 *      This drawing operation draws a image on the screen. It can be a 
 *	mono image (needed for font handling) or a color image (needed for
 *	tux). 
 */
void astvgafb_imageblit(struct fb_info *p, const struct fb_image *image) 
{
	printk("astvgafb_imageblit \n");
/*
 *      @dx: The x and y coordinates of the upper left hand corner of the
 *	@dy: destination area to place the image on the screen.
 *      @width: How wide the image is we want to copy.
 *      @height: How tall the image is we want to copy.
 *      @fg_color: For mono bitmap images this is color data for     
 *      @bg_color: the foreground and background of the image to
 *		   write directly to the frmaebuffer.
 *	@depth:	How many bits represent a single pixel for this image.
 *	@data: The actual data used to construct the image on the display.
 *	@cmap: The colormap used for color images.   
 */

/*
 * The generic function, cfb_imageblit, expects that the bitmap scanlines are
 * padded to the next byte.  Most hardware accelerators may require padding to
 * the next u16 or the next u32.  If that is the case, the driver can specify
 * this by setting info->pixmap.scan_align = 2 or 4.  See a more
 * comprehensive description of the pixmap below.
 */
}

    /*
     *  Frame buffer operations
     */

static struct fb_ops astvgafb_ops = {
	.owner		= THIS_MODULE,
	.fb_check_var	= astvgafb_check_var,
	.fb_set_par	= astvgafb_set_par,
	.fb_setcolreg	= astvgafb_setcolreg,
	.fb_blank	= astvgafb_blank,
	.fb_fillrect	= astvgafb_fillrect, 	/* Needed !!! */
	.fb_copyarea	= astvgafb_copyarea,	/* Needed !!! */
	.fb_imageblit	= astvgafb_imageblit,	/* Needed !!! */
};

/* ------------------------------------------------------------------------- */
static int astvgafb_probe(struct platform_device *pdev)
{
    struct fb_info *info;
    struct astvga_par *par;
    struct device *device = &pdev->dev; /* or &pdev->dev */
    int cmap_len, retval;	
   
    /*
     * Dynamically allocate info and par
     */
     printk("1 \n");
    info = framebuffer_alloc(sizeof(struct astvga_par), device);

    if (!info) {
	    /* goto error path */
    }
	printk("2 \n");

    	par = info->par;
	par->video_base = ioremap(0x1e700000, 0x400); 



    /* 
     * Here we set the screen_base to the virtual memory address
     * for the framebuffer. Usually we obtain the resource address
     * from the bus layer and then translate it to virtual memory
     * space via ioremap. Consult ioport.h. 
     */
    info->screen_base = ioremap(0x9f000000, 0x1000000);//framebuffer_virtual_memory; 16MB VGA
    info->fbops = &astvgafb_ops;
    info->fix = astvgafb_fix;

	info->fix.smem_start = 0x9f000000;
	info->fix.smem_len = 0x1000000;
	
    info->pseudo_palette = par->pseudo_palette;

	     printk("3 \n");
    /*
     * Set up flags to indicate what sort of acceleration your
     * driver can provide (pan/wrap/copyarea/etc.) and whether it
     * is a module -- see FBINFO_* in include/linux/fb.h
     *
     * If your hardware can support any of the hardware accelerated functions
     * fbcon performance will improve if info->flags is set properly.
     *
     * FBINFO_HWACCEL_COPYAREA - hardware moves
     * FBINFO_HWACCEL_FILLRECT - hardware fills
     * FBINFO_HWACCEL_IMAGEBLIT - hardware mono->color expansion
     * FBINFO_HWACCEL_YPAN - hardware can pan display in y-axis
     * FBINFO_HWACCEL_YWRAP - hardware can wrap display in y-axis
     * FBINFO_HWACCEL_DISABLED - supports hardware accels, but disabled
     * FBINFO_READS_FAST - if set, prefer moves over mono->color expansion
     * FBINFO_MISC_TILEBLITTING - hardware can do tile blits
     *
     * NOTE: These are for fbcon use only.
     */
    info->flags = FBINFO_DEFAULT;
	printk("4 \n");

    /*
     * This should give a reasonable default video mode. The following is
     * done when we can set a video mode. 
     */
    if (!mode_option)
	mode_option = "1024x768@60";	 	

	printk("5 \n");

    retval = fb_find_mode(&info->var, info, mode_option, NULL, 0, NULL, 8);

       printk("6 \n");
    if (!retval || retval == 4)
	return -EINVAL;			

	printk("7 \n");

    /* This has to be done! */
 //   if (fb_alloc_cmap(&info->cmap, cmap_len, 0))
//	return -ENOMEM;

	     printk("8 \n");

    /*
     * Does a call to fb_set_par() before register_framebuffer needed?  This
     * will depend on you and the hardware.  If you are sure that your driver
     * is the only device in the system, a call to fb_set_par() is safe.
     *
     * Hardware in x86 systems has a VGA core.  Calling set_par() at this
     * point will corrupt the VGA console, so it might be safer to skip a
     * call to set_par here and just allow fbcon to do it for you.
     */
    /* astvgafb_set_par(info); */
	printk("9 \n");

    if (register_framebuffer(info) < 0) {
		fb_dealloc_cmap(&info->cmap);
		return -EINVAL;
    }
	     printk("10 \n");
    fb_info(info, "%s frame buffer device\n", info->fix.id);
    platform_set_drvdata(pdev, info); /* or platform_set_drvdata(pdev, info) */
    return 0;
}

static void astvgafb_remove(struct platform_device *pdev)
{
	struct fb_info *info = platform_get_drvdata(pdev);
	/* or platform_get_drvdata(pdev); */
	printk("astvgafb_remove \n");
	if (info) {
		unregister_framebuffer(info);
		fb_dealloc_cmap(&info->cmap);
		/* ... */
		framebuffer_release(info);
	}
}

#include <linux/platform_device.h>
/* for platform devices */

#ifdef CONFIG_PM
/**
 *	astvgafb_suspend - Optional but recommended function. Suspend the device.
 *	@dev: platform device
 *	@msg: the suspend event code.
 *
 *      See Documentation/power/devices.txt for more information
 */
static int astvgafb_suspend(struct platform_device *dev, pm_message_t msg)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct astvgafb_par *par = info->par;

	/* suspend here */
	return 0;
}

/**
 *	astvgafb_resume - Optional but recommended function. Resume the device.
 *	@dev: platform device
 *
 *      See Documentation/power/devices.txt for more information
 */
static int astvgafb_resume(struct platform_dev *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);
	struct astvgafb_par *par = info->par;

	/* resume here */
	return 0;
}
#else
#define astvgafb_suspend NULL
#define astvgafb_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver astvgafb_driver = {
	.probe = astvgafb_probe,
	.remove = astvgafb_remove,
	.suspend = astvgafb_suspend, /* optional but recommended */
	.resume = astvgafb_resume,   /* optional but recommended */
	.driver = {
		.name = "astvgafb",
	},
};

static struct platform_device *astvgafb_device;

static int __init astvgafb_init(void)
{
	int ret;
	/*
	 *  For kernel boot options (in 'video=astvgafb:<options>' format)
	 */
	ret = platform_driver_register(&astvgafb_driver);

	if (!ret) {
		astvgafb_device = platform_device_register_simple("astvgafb", 0,
								NULL, 0);

		if (IS_ERR(astvgafb_device)) {
			platform_driver_unregister(&astvgafb_driver);
			ret = PTR_ERR(astvgafb_device);
		}
	}

	return ret;
}

static void __exit astvgafb_exit(void)
{
	platform_device_unregister(astvgafb_device);
	platform_driver_unregister(&astvgafb_driver);
}


module_init(astvgafb_init);
module_exit(astvgafb_exit);

MODULE_LICENSE("GPL");
