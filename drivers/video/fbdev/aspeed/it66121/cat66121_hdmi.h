#ifndef __cat66121_HDMI_H__
#define __cat66121_HDMI_H__

#include <linux/workqueue.h>

//#include "typedef.h"
//#include "hdmitx_drv.h"

/////////////////////////////////////////////////////////////////////
// hdmi data structure
/////////////////////////////////////////////////////////////////////
typedef struct _HDMITXDEV_STRUCT {

//	u8 I2C_DEV ;
//	u8 I2C_ADDR ;

	/////////////////////////////////////////////////
	// Interrupt Type
	/////////////////////////////////////////////////
	u8 bIntType ; // = 0 ;
	/////////////////////////////////////////////////
	// Video Property
	/////////////////////////////////////////////////
	u8 bInputVideoSignalType ; // for Sync Embedded,CCIR656,InputDDR
	/////////////////////////////////////////////////
	// Audio Property
	/////////////////////////////////////////////////
	u8 bOutputAudioMode ; // = 0 ;
	u8 bAudioChannelSwap ; // = 0 ;
    u8 bAudioChannelEnable ;
    u8 bAudFs ;
    unsigned long TMDSClock ;
    unsigned long RCLK ;
	u8 bAuthenticated:1 ;
	u8 bHDMIMode: 1;
	u8 bIntPOL:1 ; // 0 = Low Active
	u8 bHPD:1 ;
	// 2009/11/11 added by jj_tseng@ite.com.tw
    u8 bSPDIF_OUT;
    u8 TxEMEMStatus:1 ;
    //~jau-chih.tseng@ite.com.tw 2009/11/11
} HDMITXDEV ;


struct cat66121_hdmi_pdata {
	int gpio;
	struct i2c_client *client;
	struct delayed_work delay_work;
	struct workqueue_struct *workqueue;
	int plug_status;

	/*hdmi */	
	int pwr_mode;		/* power mode */
	int hotplug;		/* hot plug status */
	int state;		/* hdmi state machine status */
	int autoconfig;		/* if true, auto config hdmi output mode
				 * according to EDID
				 */	
	HDMITXDEV hdmiTxDev;				 
};
extern int cat66121_detect_device(struct i2c_client *client);
extern int cat66121_hdmi_sys_init(struct i2c_client *client);
extern void cat66121_hdmi_interrupt(struct cat66121_hdmi_pdata *cat66121_hdmi);
extern int cat66121_hdmi_sys_detect_hpd(struct i2c_client *client);
extern int cat66121_hdmi_sys_insert(struct i2c_client *client);
extern int cat66121_hdmi_sys_remove(struct i2c_client *client);
extern int cat66121_hdmi_sys_read_edid(struct i2c_client *client, int block, unsigned char *buff);
extern int cat66121_hdmi_sys_config_video(struct i2c_client *client, struct hdmi_video_para *vpara);
extern int cat66121_hdmi_sys_config_audio(struct i2c_client *client,struct hdmi_audio *audio);
extern void cat66121_hdmi_sys_enalbe_output(struct i2c_client *client, int enable);
extern int cat66121_hdmi_register_hdcp_callbacks(void (*hdcp_cb)(void),
					 void (*hdcp_irq_cb)(int status),
					 int (*hdcp_power_on_cb)(void),
					 void (*hdcp_power_off_cb)(void));
#endif
