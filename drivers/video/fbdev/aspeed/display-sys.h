#ifndef _LINUX_DISPLAY_AST_H
#define _LINUX_DISPLAY_AST_H

#include <linux/device.h>
#include <linux/fb.h>
#include <linux/list.h>

struct ast_display_device;

enum ast_display_priority {
	DISPLAY_PRIORITY_TV = 0,
	DISPLAY_PRIORITY_YPbPr,
	DISPLAY_PRIORITY_VGA,
	DISPLAY_PRIORITY_HDMI,
	DISPLAY_PRIORITY_LCD,
};

enum {
	DISPLAY_SCALE_X = 0,
	DISPLAY_SCALE_Y
};

/* This structure defines all the properties of a Display. */
struct ast_display_driver {
	void (*suspend)(struct ast_display_device *, pm_message_t state);
	void (*resume)(struct ast_display_device *);
	int  (*probe)(struct ast_display_device *, void *);
	int  (*remove)(struct ast_display_device *);
	int	display_no;
	const char	*name;	
};

struct ast_display_ops {
	int (*getedid)(struct ast_display_device *, u8 *);
	int (*setenable)(struct ast_display_device *, int enable);
	int (*getenable)(struct ast_display_device *);
	int (*getstatus)(struct ast_display_device *);
	int (*getmodelist)(struct ast_display_device *, struct list_head **modelist);
	int (*setmode)(struct ast_display_device *, struct fb_videomode *mode);
	int (*getmode)(struct ast_display_device *, struct fb_videomode *mode);
	int (*setscale)(struct ast_display_device *, int, int);
	int (*getscale)(struct ast_display_device *, int);
	int (*setdebug)(struct ast_display_device *, int);
	int (*getedidaudioinfo)(struct ast_display_device *, char *audioinfo, int len);
	int (*getmonspecs)(struct ast_display_device *, struct fb_monspecs *monspecs);
	int (*config_video)(struct ast_display_device *disp_dev, struct fb_var_screeninfo *var);
};

struct ast_display_device {
	struct module *owner;			/* Owner module */
	struct ast_display_driver *driver;
	struct device *parent;			/* This is the parent */
	struct device *dev;			/* This is this display device */
	struct mutex lock;
	void *priv_data;
	char type[16];
	char *name;
	int idx;
	struct ast_display_ops *ops;
	int priority;
	struct list_head list;
};

struct ast_display_devicelist {
	struct list_head list;
	struct ast_display_device *dev;
};

extern struct ast_display_device *ast_display_device_register(struct ast_display_driver *driver,
					struct device *dev, void *devdata);
extern void ast_display_device_unregister(struct ast_display_device *dev);

extern void ast_display_device_enable(struct ast_display_device *ddev);

extern void ast_display_device_enable_other(struct ast_display_device *ddev);
extern void ast_display_device_disable_other(struct ast_display_device *ddev);

extern struct ast_display_device *ast_display_device_select(int display_no);


#define to_ast_display_device(obj) container_of(obj, struct ast_display_device, class_dev)

#endif
