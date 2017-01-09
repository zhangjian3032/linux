#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#if defined(CONFIG_DEBUG_FS)
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#endif
#include <linux/of_gpio.h>
#include <linux/of_device.h>

#include "cat66121_hdmi.h"
#include "cat66121_hdmi_hw.h"

#include "../display-sys.h"
#include "../../edid.h"

#include <mach/irqs.h>


#define HDMI_POLL_MDELAY 	50//100

static void cat66121_irq_work_func(struct cat66121_hdmi_pdata *cat66121_hdmi)	
{
 	cat66121_hdmi_interrupt(cat66121_hdmi);
	if(!cat66121_hdmi->client->irq){
		queue_delayed_work(cat66121_hdmi->workqueue, &cat66121_hdmi->delay_work, HDMI_POLL_MDELAY);
	}

}

static irqreturn_t cat66121_thread_interrupt(int irq, void *dev_id)
{
	struct cat66121_hdmi_pdata *cat66121_hdmi = (struct cat66121_hdmi_pdata *)dev_id;

	cat66121_irq_work_func(cat66121_hdmi);
	msleep(HDMI_POLL_MDELAY);
	return IRQ_HANDLED;
}

int cat66121_hdmi_config_video(struct ast_display_device *disp_dev, struct fb_var_screeninfo *var)
//truct hdmi *hdmi_drv, struct hdmi_video_para *vpara)
{
	struct fb_videomode *mode;
	HDMI_Aspec aspec ;
	HDMI_Colorimetry Colorimetry ;
	VIDEOPCLKLEVEL level ;
	unsigned long TMDSClock;	
	u8 pixelrep = 0;
	struct cat66121_hdmi_pdata *cat66121_hdmi = (struct cat66121_hdmi_pdata *)disp_dev->priv_data;	

	if(var == NULL) {
		printk("[%s] input parameter error\n", __FUNCTION__);
		return 1;
	}

	HDMITX_PowerOn(cat66121_hdmi->client);

	HDMITX_DisableAudioOutput(cat66121_hdmi->client);
	HDMITX_EnableHDCP(cat66121_hdmi->client, FALSE);

	// output Color mode

	printk("F_MODE_RGB444 , pixclock = %d ~~~~~~~~~~~~~~~~~\n", var->pixclock);

	var->pixclock = 65000000;

	TMDSClock = var->pixclock * (pixelrep + 1);
	if( TMDSClock>80000000L )
	{
		level = PCLK_HIGH ;
	}
	else if(TMDSClock>20000000L)
	{
		level = PCLK_MEDIUM ;
	}
	else
	{
		level = PCLK_LOW ;
	}

	//add
	setHDMITX_ColorDepthPhase(cat66121_hdmi->client, 24, 0);
	setHDMITX_VideoSignalType(cat66121_hdmi->client, T_MODE_INDDR);

	HDMITX_EnableVideoOutput(cat66121_hdmi->client, level,F_MODE_RGB444,F_MODE_RGB444 ,FALSE);

	HDMITX_EnableAVIInfoFrame(cat66121_hdmi->client, FALSE ,NULL);
	HDMITX_EnableVSInfoFrame(cat66121_hdmi->client, FALSE,NULL);

	setHDMITX_AVMute(cat66121_hdmi->client, FALSE);

	return 0;

}

int cat66121_hdmi_read_edid(struct ast_display_device *disp_dev, u8 *buf)
{

	struct cat66121_hdmi_pdata *cat66121_hdmi = (struct cat66121_hdmi_pdata *)disp_dev->priv_data;
	// collect the EDID ucdata of segment 0
	u8 CheckSum ;
	u8 BlockCount ;
	int i ;

	getHDMITX_EDIDBlock(cat66121_hdmi->client, 0, buf);

	for( i = 0, CheckSum = 0 ; i < 128 ; i++ )
	{
		CheckSum += buf[i] ; CheckSum &= 0xFF ;
	}
	if( CheckSum != 0 ) {
		printk("check sum fail \n");		
		return 0 ;
	}
	if( buf[0] != 0x00 ||
		buf[1] != 0xFF ||
		buf[2] != 0xFF ||
		buf[3] != 0xFF ||
		buf[4] != 0xFF ||
		buf[5] != 0xFF ||
		buf[6] != 0xFF ||
		buf[7] != 0x00) {
		printk("not edid format \n");		
		return 0;
	}

	BlockCount = buf[0x7E] ;
	printk("BlockCount = %d \n", BlockCount);

	if( BlockCount == 0 )
	{
		return 1 ; // do nothing.
	}
	else if ( BlockCount > 4 )
	{
		BlockCount = 4 ;
	}
			
	 // read all segment for test
	for( i = 1 ; i <= BlockCount ; i++ )
	{

		getHDMITX_EDIDBlock(cat66121_hdmi->client, i, buf);
	
	}

}

static struct ast_display_ops cat66121_display_ops = {
	.getedid = cat66121_hdmi_read_edid,
	.config_video = cat66121_hdmi_config_video,
#if 0	
	.setenable = cvbs_set_enable,
	.getenable = cvbs_get_enable,
	.getstatus = cvbs_get_status,
	.getmodelist = cvbs_get_modelist,
	.setmode = cvbs_set_mode,
	.getmode = cvbs_get_mode,
#endif	

#if 0
hdmi_drv->insert = cat66121_hdmi_sys_insert;
hdmi_drv->remove = cat66121_hdmi_sys_remove;
hdmi_drv->control_output = cat66121_hdmi_sys_enalbe_output;
hdmi_drv->config_video = cat66121_hdmi_sys_config_video;
hdmi_drv->config_audio = cat66121_hdmi_sys_config_audio;
hdmi_drv->detect_hotplug = cat66121_hdmi_sys_detect_hpd;
hdmi_drv->read_edid = cat66121_hdmi_sys_read_edid;

#endif
};

static int display_ite66121_probe(struct ast_display_device *device, void *devdata)
{
	device->owner = THIS_MODULE;
	strcpy(device->type, "HDMI");
	device->name = "cat66121_hdmi";
	device->priv_data = devdata;
	device->ops = &cat66121_display_ops;
	return 1;
}


static struct ast_display_driver display_ite66121 = {
	.probe = display_ite66121_probe,
	.name = "ite66121"
};

struct cat66121_hdmi_info {
	struct i2c_client *client;
	struct fb_info *fb_info;
//	struct monitor_info *mon_info;
	u8 state;//0:unplug 1:plug
	int count;
};



#if defined(CONFIG_OF)
static const struct of_device_id cat66121_dt_ids[] = {
	{.compatible = "ite,cat66121",},
	{}
};
MODULE_DEVICE_TABLE(of, cat66121_dt_ids);
#endif

static int cat66121_hdmi_i2c_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int rc = 0;
	struct cat66121_hdmi_pdata *cat66121_hdmi;

	if (client->dev.of_node) {
		if (!of_match_device(cat66121_dt_ids, &client->dev)) {
			dev_err(&client->dev,"Failed to find matching dt id\n");
			return -EINVAL;
		}
	}
	
	cat66121_hdmi = kzalloc(sizeof(struct cat66121_hdmi_pdata), GFP_KERNEL);
	if(!cat66121_hdmi)
	{
		dev_err(&client->dev, "no memory for state\n");
		return -ENOMEM;
	}
	
	cat66121_hdmi->client = client;
	i2c_set_clientdata(client, cat66121_hdmi);
	
	if(cat66121_detect_device(client)!=1){
		dev_err(&client->dev, "can't find it66121 device \n");
		rc = -ENXIO;
	}

	cat66121_hdmi->plug_status = -1;

	cat66121_hdmi_sys_init(client);
	//init InstanceData
	
	/////////////////////////////////////////////////
	// Interrupt Type
	/////////////////////////////////////////////////
	cat66121_hdmi->hdmiTxDev.bIntType = 0x40;  // BYTE bIntType ; // = 0 ;
	// Video Property
	/////////////////////////////////////////////////
	cat66121_hdmi->hdmiTxDev.bInputVideoSignalType = INPUT_SIGNAL_TYPE; // BYTE bInputVideoSignalType ; // for Sync Embedded,CCIR656,InputDDR

	// Audio Property
	/////////////////////////////////////////////////
	cat66121_hdmi->hdmiTxDev.bInputVideoSignalType = I2S_FORMAT; // BYTE bOutputAudioMode ; // = 0 ;

	cat66121_hdmi->hdmiTxDev.bAudioChannelSwap = FALSE; // BYTE bAudioChannelSwap ; // = 0 ;

	cat66121_hdmi->hdmiTxDev.bAudFs = INPUT_SAMPLE_FREQ;// BYTE bAudFs ;

	cat66121_hdmi->hdmiTxDev.bAudioChannelEnable = 0x01, // BYTE bAudioChannelEnable ;

	cat66121_hdmi->hdmiTxDev.TMDSClock = 0; // unsigned long TMDSClock ;

	cat66121_hdmi->hdmiTxDev.bAuthenticated = FALSE; // BYTE bAuthenticated:1 ;
	cat66121_hdmi->hdmiTxDev.bHDMIMode = FALSE; // BYTE bHDMIMode: 1;
	cat66121_hdmi->hdmiTxDev.bIntPOL = FALSE; // BYTE bIntPOL:1 ; // 0 = Low Active
	cat66121_hdmi->hdmiTxDev.bHPD = FALSE; // BYTE bHPD:1 ;

//	printk("irq %d\n", cat66121_hdmi->client->irq);
	
	if(cat66121_hdmi->client->irq) {
		if((rc = request_threaded_irq(cat66121_hdmi->client->irq, NULL ,cat66121_thread_interrupt, IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(&client->dev), cat66121_hdmi)) < 0) 
		{
			dev_err(&client->dev, "fail to request hdmi irq\n");
			goto err_kzalloc_hdmi;
		}
	}else{
		cat66121_hdmi->workqueue = create_singlethread_workqueue("cat66121 irq");
		INIT_DELAYED_WORK(&(cat66121_hdmi->delay_work), cat66121_irq_work_func);
		cat66121_irq_work_func(NULL);
	}

	dev_info(&client->dev, "cat66121 hdmi i2c probe ok\n");

	switch(cat66121_hdmi->client->irq) {
		case IRQ_GPIOB0:
			display_ite66121.display_no = 0;
			break;
		case IRQ_GPIOB1:
			display_ite66121.display_no = 1;
			break;
		case IRQ_GPIOB2:
			display_ite66121.display_no = 2;
			break;
		case IRQ_GPIOB3:
			display_ite66121.display_no = 3;
			break;			
	}

	
#ifdef CONFIG_DISPLAY_SUPPORT
	ast_display_device_register(&display_ite66121, client->dev.parent, cat66121_hdmi); 
#endif

    return 0;

	
err_kzalloc_hdmi:
	kfree(cat66121_hdmi);
	dev_err(&client->dev, "cat66121 hdmi probe error\n");
	return rc;

}

static int cat66121_hdmi_i2c_remove(struct i2c_client *client)
{	
	//TODO ~~
#if 0
	hdmi_dbg(hdmi->dev, "%s\n", __func__);
	if(hdmi) {
		mutex_lock(&hdmi->enable_mutex);
		if(!hdmi->suspend && hdmi->enable && hdmi->irq)
			disable_irq(hdmi->irq);
		mutex_unlock(&hdmi->enable_mutex);
		if(hdmi->irq)
			free_irq(hdmi->irq, NULL);
		flush_workqueue(hdmi->workqueue);
		destroy_workqueue(hdmi->workqueue);
		#ifdef CONFIG_SWITCH
		switch_dev_unregister(&(hdmi->switch_hdmi));
		#endif
		hdmi_unregister_display_sysfs(hdmi);
		#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&hdmi->early_suspend);
		#endif
		fb_destroy_modelist(&hdmi->edid.modelist);
		if(hdmi->edid.audio)
			kfree(hdmi->edid.audio);
		if(hdmi->edid.specs)
		{
			if(hdmi->edid.specs->modedb)
				kfree(hdmi->edid.specs->modedb);
			kfree(hdmi->edid.specs);
		}
		kfree(hdmi);
		hdmi = NULL;
	}
#endif	
    return 0;
}

static const struct i2c_device_id cat66121_hdmi_id[] = {
	{ "cat66121_hdmi", 0 },
	{ }
};

static struct i2c_driver cat66121_hdmi_i2c_driver = {
	.class = I2C_CLASS_DDC,		
	.driver = {
		.name  = "cat66121_hdmi",
		.owner = THIS_MODULE,
//		.of_match_table = of_match_ptr(cat66121_dt_ids),
	},
	.probe      	= cat66121_hdmi_i2c_probe,
	.remove     	= cat66121_hdmi_i2c_remove,
	.id_table		= cat66121_hdmi_id,
};

static int __init cat66121_hdmi_init(void)
{
	return i2c_add_driver(&cat66121_hdmi_i2c_driver);
}

static void __exit cat66121_hdmi_exit(void)
{
	i2c_del_driver(&cat66121_hdmi_i2c_driver);
}

//module_init(cat66121_hdmi_init);
subsys_initcall(cat66121_hdmi_init);
module_exit(cat66121_hdmi_exit);
