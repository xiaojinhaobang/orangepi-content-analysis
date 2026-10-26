#include "ANO_LX.h"
#include "Drv_RcIn.h"
#include "ANO_DT_LX.h"
#include "ANO_Math.h"
#include "Drv_PwmOut.h"
#include "LX_FC_State.h"
#include "LX_FC_EXT_Sensor.h"
#include "Drv_AnoOf.h"
#include "Drv_adc.h"
#include "Drv_led.h"
#include "Drv_UbloxGPS.h"
#include "LX_FC_Fun.h"
#include "Drv_Uart.h"
#include "User_Task.h"
#include "AnoDTRasp.h"
#include "TJC_lcd.h"

/*==========================================================================
 * 锟斤拷锟斤拷    锟斤拷锟斤拷锟斤拷锟缴匡拷锟斤拷锟诫、锟斤拷锟斤拷锟斤拷锟斤拷锟?
 * 锟斤拷锟斤拷时锟戒：2020-01-22 
 * 锟斤拷锟斤拷		 锟斤拷锟斤拷锟斤拷锟狡达拷-Jyoun
 * 锟斤拷锟斤拷    锟斤拷www.anotc.com
 * 锟皆憋拷    锟斤拷anotc.taobao.com
 * 锟斤拷锟斤拷Q群 锟斤拷190169595
 * 锟斤拷目锟斤拷锟斤拷锟斤拷18084888982锟斤拷18061373080
============================================================================
 * 锟斤拷锟斤拷锟狡达拷锟脚队革拷谢锟斤拷业锟街э拷郑锟斤拷锟接拷锟揭斤拷群锟斤拷锟洁交锟斤拷锟斤拷锟斤拷锟桔★拷学习锟斤拷
 * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟叫诧拷锟矫的地凤拷锟斤拷锟斤拷迎锟斤拷锟斤拷砖锟斤拷锟斤拷锟斤拷锟?
 * 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷萍锟斤拷锟街э拷锟斤拷锟斤拷恰锟?
 * 锟斤拷锟斤拷锟斤拷源锟斤拷锟斤拷锟斤拷牖队拷锟斤拷锟斤拷锟斤拷谩锟斤拷锟斤拷锟斤拷锟斤拷展锟斤拷锟斤拷锟斤拷锟斤拷希锟斤拷锟斤拷锟斤拷使锟斤拷时锟斤拷注锟斤拷锟斤拷锟斤拷锟斤拷
 * 锟斤拷锟斤拷坦锟斤拷锟斤拷锟斤拷小锟剿筹拷锟斤拷锟捷ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟剿拷锟斤拷锟斤拷锟斤拷锟斤拷樱锟揭诧拷锟轿达拷泄锟侥拷锟酵拷械锟斤拷锟轿拷锟? 
 * 锟斤拷源锟斤拷锟阶ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷祝锟较ｏ拷锟斤拷锟揭伙拷锟斤拷锟斤拷锟截★拷锟斤拷锟斤互锟斤拷锟斤拷锟斤拷同锟斤拷锟斤拷锟斤拷
 * 只锟斤拷锟斤拷锟斤拷支锟街ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟矫革拷锟矫★拷  
===========================================================================*/
extern int takeoff_if;
extern int flown_flag;


_rt_tar_un rt_tar;
_pwm_st pwm_to_esc;
_fc_bat_un fc_bat;
_fc_att_un fc_att;
_fc_att_qua_un fc_att_qua;
_fc_vel_un fc_vel;

_fc_alt_un fc_alt;
_fc_acc_st fc_acc;
_fc_mag_st fc_mag;
_fc_kz_st fc;

double sensi=0.00002;

s16 m_speed_yaw;
s16 m_speed_x;
s16 m_speed_y;
s16 m_speed_z;

void Set_m_speed(s16 x,s16 y,s16 z,s16 yaw)
{
    Send_speed_cun(x,y,z,yaw,1);
    if(x>60)
        x=60;
    else if(x<-60)
        x=-60;
    if(y>60)
        y=60;
    else if(y<-60)
        y=-60;
    if(z>60)
        z=60;
    else if(z<-60)
        z=-60;
    if(yaw>30)
        yaw=30;
    else if(yaw<-30)
        yaw=-30;
    
    m_speed_x=x;
    m_speed_y=y;
    m_speed_z=z;
    m_speed_yaw=yaw;
}




//遥锟斤拷CH5(AUX1)通锟斤拷值(1000-1500-2000)锟斤拷锟斤拷模式1-2-3锟斤拷模式0锟斤拷要通锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷指锟斤拷锟斤拷锟斤拷
//模式0锟斤拷锟斤拷态锟斤拷锟斤拷    ->遥锟斤拷CH1-CH4直锟接匡拷锟斤拷锟斤拷态锟斤拷锟斤拷锟脚★拷
//模式1锟斤拷锟斤拷锟斤拷+锟斤拷锟斤拷   ->遥锟斤拷CH1/CH2/CH3锟斤拷锟斤拷锟斤拷态锟斤拷锟斤拷锟斤拷遥锟斤拷CH3(锟斤拷锟斤拷摇锟斤拷)锟斤拷锟狡达拷直锟斤拷锟斤拷锟劫度★拷
//模式2锟斤拷锟斤拷锟斤拷        ->遥锟斤拷CH1/CH2锟斤拷锟斤拷水平锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟斤拷遥锟斤拷CH3(锟斤拷锟斤拷摇锟斤拷)锟斤拷锟狡达拷直锟斤拷锟斤拷锟劫讹拷,遥锟斤拷CH4锟斤拷锟斤拷YAW锟斤拷态锟斤拷
//模式3锟斤拷锟教匡拷        ->遥锟斤拷摇锟剿诧拷锟斤拷锟斤拷锟斤拷锟?

//
#define MAX_ANGLE 3500	//锟斤拷锟斤拷锟绞憋拷嵌龋锟?锟斤拷位 0.01锟斤拷
#define MAX_YAW_DPS 200 //锟斤拷锟斤拷锟绞盰AW锟斤拷锟劫度ｏ拷锟斤拷位锟斤拷每锟斤拷
//
#define MAX_VELOCITY 500  //锟斤拷锟斤拷锟绞彼斤拷俣龋锟斤拷锟轿伙拷锟斤拷锟矫匡拷锟?
#define MAX_VER_VEL_P 300 //锟斤拷锟斤拷锟绞憋拷锟街憋拷锟斤拷俣龋锟斤拷锟轿伙拷锟斤拷锟矫匡拷锟?
#define MAX_VER_VEL_N 200 //锟斤拷锟斤拷锟绞憋拷锟街憋拷锟斤拷俣龋锟斤拷锟轿伙拷锟斤拷锟矫匡拷锟?

//////////////////////////////////////////////////////////////////////
//锟斤拷锟斤拷为锟缴控伙拷锟斤拷锟斤拷锟杰筹拷锟津，诧拷锟斤拷锟斤拷锟矫伙拷锟侥讹拷锟酵碉拷锟矫★拷
//////////////////////////////////////////////////////////////////////

//遥锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷
static inline void RC_Data_Task(float dT_s)
{
	static u8 fail_safe_change_mod, fail_safe_return_home;
	static u8 mod_f[3];
	static u16 mod_f_time_cnt;

	//遥锟斤拷没锟斤拷失锟截憋拷遣锟街达拷锟?
	if ( rc_in.fail_safe == 0)
	{
		//摇锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷模式锟斤拷锟斤拷态+锟斤拷压锟斤拷锟竭ｏ拷锟斤拷锟竭讹拷锟姐，锟教控ｏ拷
		//注锟解，锟教匡拷模式锟铰ｏ拷锟缴匡拷只锟斤拷应锟斤拷锟斤拷指锟斤拷锟结供锟斤拷锟脚号ｏ拷锟斤拷锟斤拷锟斤拷应摇锟剿★拷
		if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1200)
		{
			LX_Change_Mode(1);
			mod_f[0] = 1;
		}
		else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1700)
		{
			LX_Change_Mode(2);
			mod_f[0] = 2;
		}
		else
		{
			LX_Change_Mode(3);
			mod_f[0] = 3;
		}
		//锟斤拷锟叫伙拷模式时执锟斤拷
		if (mod_f[1] != mod_f[0])
		{
			mod_f[1] = mod_f[0];
			//锟斤拷锟斤拷锟侥Ｊ?锟斤拷锟斤拷锟斤拷一锟轿★拷
			if (mod_f[0] == 3)
			{
				mod_f[2]++;
			}
		}
		//锟剿段筹拷锟斤拷锟斤拷时2000ms锟节硷拷锟斤拷谢锟斤拷炭锟侥Ｊ斤拷拇锟斤拷锟?锟斤到3锟斤拷锟斤拷执锟叫凤拷锟斤拷
		if (mod_f[2] != 0)
		{
			if (mod_f_time_cnt < 2000)
			{
				mod_f_time_cnt += 1e3f * dT_s;
			}
			else
			{
				u8 tmp;
				if (mod_f[2] >= 3)
				{
					//执锟叫凤拷锟斤拷
					tmp = OneKey_Return_Home();
				}
				else
				{
					//null
				}
				//reset
				if (tmp)
				{
					mod_f_time_cnt = 0;
					mod_f[2] = 0;
				}
			}
		}

		//摇锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
		//摇锟斤拷锟斤拷锟斤拷转锟斤拷锟斤拷+-500锟斤拷锟斤拷锟斤拷锟斤拷
		float tmp_ch_dz[4];
		tmp_ch_dz[ch_1_rol] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_1_rol] - 1500), 0, 40);
		tmp_ch_dz[ch_2_pit] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_2_pit] - 1500), 0, 40);
		tmp_ch_dz[ch_3_thr] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_3_thr] - 1500), 0, 80);
		tmp_ch_dz[ch_4_yaw] = my_deadzone((rc_in.rc_ch.st_data.ch_[ch_4_yaw] - 1500), 0, 80);
		//准锟斤拷锟斤拷锟斤拷时锟斤拷ROL,PIT,YAW锟斤拷效
		if (sti_fun.pre_locking)
		{
			tmp_ch_dz[ch_1_rol] = 0;
			tmp_ch_dz[ch_2_pit] = 0;
			tmp_ch_dz[ch_4_yaw] = 0;
		}
		//摇锟斤拷转锟斤拷锟斤拷态锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
		//注锟斤拷0.00217f锟斤拷0.00238f锟斤拷为锟剿讹拷应锟侥诧拷锟斤拷锟斤拷锟斤拷锟斤拷小锟侥诧拷锟街ｏ拷原锟斤拷为0.002f锟斤拷锟斤拷500转锟斤拷锟斤拷锟斤拷1锟斤拷
		//注锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷要锟斤拷锟斤拷ANO锟斤拷锟斤拷系锟斤拷锟藉，一锟斤拷锟斤拷锟斤拷锟教拷潜锟绞撅拷锟斤拷颍ò锟斤拷锟斤拷锟轿伙拷锟斤拷锟斤拷锟揭ｏ拷锟揭★拷朔锟斤拷锟斤拷锟缴匡拷使锟矫碉拷ANO锟斤拷锟斤拷系锟斤拷同锟斤拷
//		if(mod_f[0]<2)
//		{		
			rt_tar.st_data.rol = tmp_ch_dz[ch_1_rol] * 0.00217f * MAX_ANGLE*sensi;                                                                    //锟揭ｏ拷锟斤拷锟斤拷
			rt_tar.st_data.pit = -tmp_ch_dz[ch_2_pit] * 0.00217f * MAX_ANGLE*sensi;		//锟斤拷为摇锟剿革拷锟斤拷锟斤拷锟斤拷投锟斤拷锟侥革拷锟斤拷锟斤拷锟斤拷锟洁反锟斤拷锟斤拷锟斤拷取锟斤拷            //锟揭ｏ拷锟斤拷锟斤拷
			rt_tar.st_data.thr = (rc_in.rc_ch.st_data.ch_[ch_3_thr] - 1000);		//0.1%                                                      //锟斤拷锟斤拷锟斤拷
			rt_tar.st_data.yaw_dps = -tmp_ch_dz[ch_4_yaw] * 0.00238f * MAX_YAW_DPS; //锟斤拷为摇锟剿猴拷锟斤拷锟斤拷投锟斤拷锟侥猴拷锟斤拷锟斤拷锟洁反锟斤拷锟斤拷锟斤拷取锟斤拷		    //锟斤拷锟斤拷锟斤拷
//		}
		//############(实时锟斤拷锟斤拷帧锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟秸伙拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟斤赋值锟斤拷锟斤拷)##############
		//实时XYZ-YAW锟斤拷锟斤拷锟劫讹拷(实时锟斤拷锟斤拷帧)
        
       
		
        if(takeoff_if)
        {
            // Horizontal velocity is released only after the takeoff command
            // has been accepted, so a stray 0x31 cannot drag the drone
            // sideways while it is still on the ground.
            rt_tar.st_data.vel_x = flown_flag ? m_speed_x : 0;    //头锟斤拷锟劫度ｏ拷锟斤拷锟斤拷每锟斤拷
            rt_tar.st_data.vel_y = flown_flag ? m_speed_y : 0;    //锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟斤拷每锟斤拷
            rt_tar.st_data.vel_z = m_speed_z;    //锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟斤拷每锟斤拷
            rt_tar.st_data.yaw_dps = m_speed_yaw;//锟斤拷锟斤拷转锟斤拷锟斤拷锟劫度ｏ拷锟斤拷每锟诫，锟斤拷时锟斤拷为锟斤拷
        }else
        {
            rt_tar.st_data.vel_x = 0;    //头锟斤拷锟劫度ｏ拷锟斤拷锟斤拷每锟斤拷
            rt_tar.st_data.vel_y = 0;    //锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟斤拷每锟斤拷
            rt_tar.st_data.vel_z = 0;    //锟斤拷锟斤拷锟劫度ｏ拷锟斤拷锟斤拷每锟斤拷
            rt_tar.st_data.yaw_dps = 0;  //锟斤拷锟斤拷转锟斤拷锟斤拷锟劫度ｏ拷锟斤拷每锟诫，锟斤拷时锟斤拷为锟斤拷
        }
		//########################################################################
		//=====
		dt.fun[0x41].WTS = 1; //锟斤拷要锟斤拷锟斤拷rt_tar锟斤拷锟捷★拷
		//失锟截憋拷锟斤拷锟斤拷歉锟轿?
		fail_safe_change_mod = 0;
		fail_safe_return_home = 0;
	}
	else //锟斤拷遥锟斤拷锟脚猴拷
	{
		//锟斤拷锟斤拷锟襟，讹拷失锟脚猴拷失锟斤拷锟斤拷要锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟缴凤拷锟斤拷时锟斤拷锟皆讹拷锟斤拷锟斤拷锟斤拷锟戒）
		if (fc_sta.unlock_sta != 0)
		{
			//锟斤拷应锟侥改憋拷模式
			if (fail_safe_change_mod == 0)
			{
				//失锟截憋拷锟斤拷锟斤拷锟叫伙拷锟斤拷锟教匡拷模式
				fail_safe_change_mod = LX_Change_Mode(3);
			}
			else if (fail_safe_return_home == 0)
			{
				//锟叫伙拷锟斤拷锟教匡拷模式锟襟，凤拷锟酵凤拷锟斤拷指锟斤拷
				fail_safe_return_home = OneKey_Return_Home();
			}
		}

		//失锟截憋拷锟斤拷时目锟斤拷值
		rt_tar.st_data.rol = 0;
		rt_tar.st_data.pit = 0;
		rt_tar.st_data.thr = 350; //锟斤拷锟斤拷模式0锟斤拷锟斤拷锟斤拷模式0时失锟截ｏ拷锟斤拷锟脚癸拷锟斤拷锟斤拷埽锟斤拷锟揭伙拷锟斤拷缘锟斤拷锟斤拷锟轿伙拷锟斤拷锟斤拷锟?
		//锟斤拷锟斤拷锟斤拷实时XYZ-YAW锟斤拷锟斤拷锟劫讹拷锟斤拷锟斤拷		
		rt_tar.st_data.yaw_dps = 0;
		rt_tar.st_data.vel_x =
			rt_tar.st_data.vel_y =
				rt_tar.st_data.vel_z = 0;
	}
}

//锟斤拷锟斤拷锟斤拷锟斤拷
static inline void ESC_Output(u8 unlocked)
{
	static u8 esc_calibrated;
	static s16 pwm[8];
	//
	pwm[0] = pwm_to_esc.pwm_m1 * 0.1f;
	pwm[1] = pwm_to_esc.pwm_m2 * 0.1f;
	pwm[2] = pwm_to_esc.pwm_m3 * 0.1f;
	pwm[3] = pwm_to_esc.pwm_m4 * 0.1f;
	pwm[4] = pwm_to_esc.pwm_m5 * 0.1f;
	pwm[5] = pwm_to_esc.pwm_m6 * 0.1f;
	pwm[6] = pwm_to_esc.pwm_m7 * 0.1f;
	pwm[7] = pwm_to_esc.pwm_m8 * 0.1f;
	//

	if (esc_calibrated == 0)
	{
//注锟解，锟斤拷锟斤拷ESC校准锟斤拷锟杰ｏ拷锟斤拷锟斤拷锟杰凤拷锟斤拷锟斤拷锟斤拷预锟较碉拷锟金坏伙拷锟斤拷锟斤拷锟斤拷锟剿猴拷锟斤拷锟斤拷锟斤拷愿锟斤拷锟?
//一锟斤拷锟斤拷要校准时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟解发锟斤拷锟斤拷锟解。
//校准ESC锟缴癸拷锟襟，记得关闭此癸拷锟杰ｏ拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷狻?
#if (ESC_CALI == 1)
		//
		for (u8 i = 0; i < 8; i++)
		{
			pwm[i] = 1000; //校准时锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟脚ｏ拷注锟斤拷锟斤拷危锟秸★拷
		}
		//
		//锟斤拷遥锟斤拷锟脚猴拷 锟斤拷 锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷位时锟斤拷锟斤拷锟?锟斤拷锟斤拷锟脚猴拷
		if (rc_in.fail_safe == 0 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1150)
		{
			//
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = 0;
			}
			//锟斤拷锟叫Ｗ硷拷锟缴★拷
			esc_calibrated = 1;
		}
#else
		//没锟叫匡拷校准锟斤拷锟杰ｏ拷直锟接憋拷锟叫Ｗ硷拷锟缴★拷
		esc_calibrated = 1;
#endif
	}
	else
	{
		//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷0锟斤拷锟斤拷
		if (unlocked)
		{
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = LIMIT(pwm[i], 0, 1000);
			}
		}
		else
		{
			for (u8 i = 0; i < 8; i++)
			{
				pwm[i] = 0;
			}
		}
	}
	//锟斤拷锟阶诧拷PWM锟斤拷锟斤拷锟斤拷锟斤拷藕锟?
	DrvMotorPWMSet(pwm);
}

//锟斤拷锟斤拷ADC锟斤拷锟斤拷锟截碉拷压
static void Bat_Voltage_Data_Handle()
{
	fc_bat.st_data.voltage_100 = Drv_AdcGetBatVot() * 100; //锟斤拷位锟斤拷10mv
}

//锟斤拷时1ms锟斤拷锟斤拷
void ANO_LX_Task()
{
	static u16 tmp_cnt[2];
	//锟斤拷10ms
	tmp_cnt[0]++;
	tmp_cnt[0] %= 10;
	if (tmp_cnt[0] == 0)
	{
		//遥锟斤拷锟斤拷锟斤拷
		DrvRcInputTask(0.01f);
		//遥锟斤拷锟斤拷锟捷达拷锟斤拷
		RC_Data_Task(0.01f);
		//锟缴匡拷状态锟斤拷锟斤拷
		LX_FC_State_Task(0.01f); //
		//锟斤拷锟斤拷锟斤拷锟斤拷状态锟斤拷锟?
		AnoOF_Check_State(0.01f);
		//==
		//锟斤拷100ms
		tmp_cnt[1]++;
		tmp_cnt[1] %= 10;
		if (tmp_cnt[1] == 0)
		{
			//锟斤拷取锟斤拷氐锟窖癸拷锟较?
			Bat_Voltage_Data_Handle();
		}
	}
	//锟斤拷锟斤拷锟斤拷锟节斤拷锟秸碉拷锟斤拷锟斤拷锟斤拷
	DrvUartDataCheck();
	//GPS锟斤拷锟捷达拷锟斤拷
	GPS_Data_Prepare_Task(1);
	//锟解部锟斤拷锟斤拷锟斤拷锟斤拷锟捷达拷锟斤拷
	LX_FC_EXT_Sensor_Task(0.001f);
	//通锟脚斤拷锟斤拷
	ANO_LX_Data_Exchange_Task(0.001f);
    AnoDTRaspRunTask1Ms();
	//锟斤拷锟斤拷锟斤拷
	ESC_Output(1); //unlocked
	//锟狡癸拷锟斤拷锟斤拷
	LED_1ms_DRV();
}
