#include "AnoDTRasp.h"
#include "Drv_Uart.h"
#include "ANO_DT_LX.h"
#include "Drv_led.h"
#include "AnoRos_DT.h"
#include "LX_FC_EXT_Sensor.h"
#include "User_Task.h"
#include "ANO_LX.h"
#include "LX_FC_Fun.h"
#include "TJC_lcd.h"
#include "LX_FC_State.h"
#include "Drv_BSP.h"

static void anoDTRaspDataFrameAnl(void);
static void anoDTRaspSendCheck(void);
int Con_flag=0;
int my_fly_flag=0;
int Ano_Duoji=0;
int qr_num=0;
int qr_num_cun=0;
//锟斤拷锟斤拷锟斤拷锟捷伙拷锟斤拷
static myTAnoPTv7Frame rxFrame;
//锟斤拷锟斤拷锟斤拷锟捷伙拷锟斤拷
#define ANOPTV8TXBUFNUM	10
static myTPTv7TxFrame txFrameBuf[ANOPTV8TXBUFNUM];
static u8 txBufWAddr = 0;
static u8 txBufRAddr = 0;

ROS_DATA_ST rosData;//锟斤拷锟斤拷锟斤拷锟捷结构锟斤拷


//===== Mission control (RDK -> FC) =====
u8  mission_end_sub  = 0;   // last received sub-command, echoed back in 0xF1
u8  mission_end_req  = 0;   // pending end action, 0 = none
int mission_end_flag = 0;   // 1 = RDK declared mission over; cleared when CH6 goes low
u8  mission_tk_req   = 0;   // pending takeoff request
u16 mission_tk_h     = 0;   // takeoff target height in cm (0 -> default 150)
u8  mission_tk_sent    = 0;   // 2026-10-16: frame queued, waiting for the FC echo
u8  mission_tk_retry   = 0;   // 2026-10-16: attempts of the current takeoff transaction
u8  mission_tk_fail    = 0;   // 2026-10-16: 1 = takeoff never accepted (budget exhausted)
u16 mission_tk_wait_ms = 0;   // 2026-10-16: ms spent waiting for the FC echo
#define TAKEOFF_VERIFY_MS 300   // 2026-10-16: how long to wait for cmd1==5
#define TAKEOFF_RETRY_MAX 4     // bounded attempts; never auto-unlock on failure
int flown_flag       = 0;   // 1 = airborne, horizontal velocity released

//===== Velocity-link watchdogs, ticked from the 1ms task =====
#define CMD_TIMEOUT_MS  200
#define FBK_TIMEOUT_MS  200
static u16 cmd_check_ms = 0, fbk_check_ms = 0;
static u8  cmd_lost = 0, fbk_lost = 0;
static u8  mission_last_unlock_sta = 0;

extern int takeoff_if;          // defined in User_Task.c
extern int mission_step_cun;    // defined in TJC_lcd.c
extern volatile u16 onekey_land_calls;  // incremented at every OneKey_Land() call

/*
 * Clear every field belonging to a pending takeoff transaction.
 * A failed transaction must also close the velocity gate so a stale request
 * can never release 0x31 after the operator has landed or locked the aircraft.
 */
static void missionTakeoffAbort(u8 failed)
{
    mission_tk_req     = 0;
    mission_tk_h       = 0;
    mission_tk_sent    = 0;
    mission_tk_retry   = 0;
    mission_tk_wait_ms = 0;
    mission_tk_fail    = failed ? 1 : 0;
    if(failed)
    {
        takeoff_if = 0;
        flown_flag = 0;
    }
}

//
void AnoDTRaspRunTask1Ms(void)
{
    static u16 tmp_cnt[2];
    static u16 diag_cnt = 0;

    //----- watchdog: command leg (0x31) -----
    // If 0x31 stops arriving, zero the velocity command. Deliberately does NOT
    // touch takeoff_if - a 200ms hiccup must not end the mission.
    if(cmd_check_ms < CMD_TIMEOUT_MS) cmd_check_ms++;
    else if(!cmd_lost) { cmd_lost = 1; Set_m_speed(0,0,0,0); }

    //----- watchdog: feedback leg (0x32) -----
    // If the SLAM velocity stops arriving, push 0x8000 so the IMU board knows
    // the observation is invalid instead of integrating stale data.
    if(fbk_check_ms < FBK_TIMEOUT_MS) fbk_check_ms++;
    else if(!fbk_lost) { fbk_lost = 1; Set_m_speed_now_Invalid(); }

    if(mission_tk_req && mission_last_unlock_sta == 1
            && fc_sta.unlock_sta == 0)
    {
        /* Operator disarmed: a pending takeoff must not survive into re-arm. */
        missionTakeoffAbort(1);
    }
    mission_last_unlock_sta = fc_sta.unlock_sta;

    /*=== 2026-10-16 takeoff transaction (v2) ===
     * "Queued" is NOT success any more: the CK ack only means the frame
     * arrived, and the FC may still refuse to act on it. The only real
     * confirmation is the FC own echo of the command: cmd1 == 0x05
     * (fc_sta.cmd_fun.CMD_1, decoded in ANO_DT_LX.c).
     */
    if(mission_tk_req && mission_end_req)
    {
        /* An end/kill command always wins over a queued takeoff. */
        missionTakeoffAbort(0);
    }
    if(mission_tk_req)
    {
        u16 h = mission_tk_h ? mission_tk_h : 150;
        if(!mission_tk_sent)
        {
            if(OneKey_Takeoff(h))          /* slot free -> queued */
            {
                mission_tk_sent    = 1;
                mission_tk_wait_ms = 0;
            }
        }
        else if(fc_sta.cmd_fun.CID == 0x10 && fc_sta.cmd_fun.CMD_1 == 0x05)
        {
            /* the FC really took the takeoff */
            missionTakeoffAbort(0);
            takeoff_if       = 1;    // release the velocity channel
            flown_flag       = 1;    // release horizontal velocity
        }
        else if(++mission_tk_wait_ms >= TAKEOFF_VERIFY_MS)
        {
            /* not accepted: count an attempt and re-queue */
            mission_tk_wait_ms = 0;
            mission_tk_sent    = 0;
            mission_tk_retry++;
            if(mission_tk_retry >= TAKEOFF_RETRY_MAX)
            {
                /* Never auto-unlock. Only the operator may re-arm. */
                missionTakeoffAbort(1);
            }
        }
    }

    //----- end request: retry until accepted -----
    if(mission_end_req)
    {
        u8 ok = 0;
        switch(mission_end_req)
        {
            case 0x02: ok = OneKey_Land();        break;
            case 0x03: ok = OneKey_Return_Home(); break;
            case 0x04: ok = FC_Lock();            break;
            default:   ok = 1;                    break;   // 0x01: hover in place
        }
        if(ok) mission_end_req = 0;
    }
    //锟斤拷5ms
    tmp_cnt[0]++;
    tmp_cnt[0] %= 5;
    if (tmp_cnt[0] == 0)
    {
        AnoPTv7SendALT(fc_alt.st_data.add);
        AnoPTv7SendATT(fc_att.st_data.rol_x100,fc_att.st_data.pit_x100,fc_att.st_data.yaw_x100);
        AnoPTv7SendCTRL(rt_tar.st_data.rol,rt_tar.st_data.pit,rt_tar.st_data.thr,rt_tar.st_data.yaw_dps,rt_tar.st_data.vel_x,rt_tar.st_data.vel_y,rt_tar.st_data.vel_z);
        AnoPTv7SendIMU(
                    fc_acc.acc_x,fc_acc.acc_y,fc_acc.acc_z,
                    fc_acc.gyr_x,fc_acc.gyr_y,fc_acc.gyr_z,
                    fc_att_qua.st_data.w_x10000,
                    fc_att_qua.st_data.x_x10000,
                    fc_att_qua.st_data.y_x10000,
                    fc_att_qua.st_data.z_x10000
                );
		AnoPTv7SendVEL(fc_vel.st_data.vel_x,fc_vel.st_data.vel_y,fc_vel.st_data.vel_z);
		AnoPTv7SendPWM();   // 走 5ms 已验证通道发 0x20（4 路）
//		AnoPTv7SendMAG(fc_mag.mag[0],fc_mag.mag[1],fc_mag.mag[2],fc_mag.tmp);
//		AnoPTv7SendRC(
//						rc_in.rc_ch.st_data.ch_[ch_1_rol],
//						rc_in.rc_ch.st_data.ch_[ch_2_pit],
//						rc_in.rc_ch.st_data.ch_[ch_3_thr],
//						rc_in.rc_ch.st_data.ch_[ch_4_yaw],
//						rc_in.rc_ch.st_data.ch_[ch_5_aux1],
//						rc_in.rc_ch.st_data.ch_[ch_6_aux2],
//						rc_in.rc_ch.st_data.ch_[ch_7_aux3],
//						rc_in.rc_ch.st_data.ch_[ch_8_aux4],
//						rc_in.rc_ch.st_data.ch_[ch_9_aux5],
//						rc_in.rc_ch.st_data.ch_[ch_10_aux6]
//					);
        //锟斤拷10ms
        tmp_cnt[1]++;
        tmp_cnt[1] %= 2;
        if (tmp_cnt[1] == 0)
        {
//			AnoPTv7SendGPS(100000000,-100000000,100000000);
			AnoPTv7SendUserData(mission_end_sub,(u8)mission_step_cun,qr_num_cun,(s16)rxFrame.frame.dataLen,rosData.tacc[0],rosData.tacc[1]);
            
        }
        // Read-only diagnostics at 20 Hz (5ms task, every 10 ticks).
        // Payload (20 bytes):
        //   0-1 voltage_100 u16 LE; 2 no_signal; 3 fail_safe;
        //   4 imu_state; 5 bar_sta; 6 mag_sta; 7 mission_end_req;
        //   8-9 onekey_land_calls; 10 cmd_lost; 11 fbk_lost;
        //   12 state bits; 13 reserved;
        //   14-15 CH3; 16-17 CH5; 18-19 CH6 (all s16 LE).
        // No field changes control behavior.
        diag_cnt++;
        if (diag_cnt >= 10)
        {
            u8 diag[20];
            u16 voltage_100 = fc_bat.st_data.voltage_100;
            diag_cnt = 0;
            diag[0] = (u8)(voltage_100 & 0xFF);
            diag[1] = (u8)(voltage_100 >> 8);
            diag[2] = (u8)(rc_in.no_signal ? 1 : 0);
            diag[3] = (u8)(rc_in.fail_safe ? 1 : 0);
            diag[4] = (u8)fc_acc.state;
            diag[5] = (u8)fc_mag.bar_sta;
            diag[6] = (u8)fc_mag.mag_sta;
            diag[7] = (u8)mission_end_req;
            diag[8] = (u8)(onekey_land_calls & 0xFF);
            diag[9] = (u8)(onekey_land_calls >> 8);
            diag[10] = (u8)cmd_lost;
            diag[11] = (u8)fbk_lost;
            diag[12] = (u8)((takeoff_if ? 0x01 : 0)
                          | (flown_flag ? 0x02 : 0)
                          | (mission_tk_req ? 0x04 : 0)
                          | (mission_tk_fail ? 0x08 : 0));
            diag[13] = 0;
            diag[14] = (u8)((u16)rc_in.rc_ch.st_data.ch_[ch_3_thr] & 0xFF);
            diag[15] = (u8)((u16)rc_in.rc_ch.st_data.ch_[ch_3_thr] >> 8);
            diag[16] = (u8)((u16)rc_in.rc_ch.st_data.ch_[ch_5_aux1] & 0xFF);
            diag[17] = (u8)((u16)rc_in.rc_ch.st_data.ch_[ch_5_aux1] >> 8);
            diag[18] = (u8)((u16)rc_in.rc_ch.st_data.ch_[ch_6_aux2] & 0xFF);
            diag[19] = (u8)((u16)rc_in.rc_ch.st_data.ch_[ch_6_aux2] >> 8);
            AnoPTv7ForwardFrame(0x0E, diag, 20);
        }
    }
    
    //锟斤拷锟斤拷欠锟斤拷锟斤拷锟斤拷锟斤拷锟揭拷锟斤拷锟?
    anoDTRaspSendCheck();
}
//
void AnoDTRaspRecvOneByte(u8 data)
{

    static u16 _recv_cnt = 0;
    static u16 _data_cnt = 0;
    static u8 rxstate = 0;

    //锟叫讹拷帧头锟角凤拷锟斤拷锟斤拷锟斤拷锟斤拷协锟斤拷锟?xAB
    if (rxstate == 0 && data == 0xAA)
    {
        _recv_cnt = 0;
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxstate ++;
    }
    //锟斤拷锟捷碉拷址锟街斤拷
    else if (rxstate == 1)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxstate ++;
    }
    //帧ID
    else if (rxstate == 2)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxstate ++;
    }
    //锟斤拷锟斤拷锟斤拷锟捷筹拷锟斤拷
    else if (rxstate == 3)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        rxFrame.frame.dataLen = data;
        _data_cnt = 0;
        rxstate ++;
    }
    //锟斤拷锟斤拷锟斤拷锟斤拷
    else if (rxstate == 4)
    {
        rxFrame.rawBytes[_recv_cnt++] = data;
        _data_cnt++;
        if(_data_cnt >= rxFrame.frame.dataLen)
        {
            rxstate ++;
        }
    }
    //锟斤拷锟斤拷SC1
    else  if (rxstate == 5)
    {
        rxFrame.frame.sc1 = data;
        rxstate ++;
    }
    //锟斤拷锟斤拷SC2
    else  if (rxstate == 6)
    {
        rxFrame.frame.sc2 = data;
        rxstate = 0;
        //锟斤拷锟斤拷锟斤拷锟捷斤拷锟斤拷锟斤拷希锟斤拷锟斤拷薪锟斤拷锟?
        anoDTRaspDataFrameAnl();
    }
    else
    {
        rxstate = 0;
    }
}


static void anoDTRaspDataFrameAnl(void)
{
    u8 check_sum1 = 0, check_sum2 = 0;
    int zancun=0;
    //锟斤拷锟斤拷锟秸碉拷锟斤拷锟斤拷锟捷硷拷锟斤拷校锟斤拷锟街斤拷1锟斤拷2
    for (u16 i = 0; i < (rxFrame.frame.dataLen + 4); i++)
    {
        check_sum1 += rxFrame.rawBytes[i];
        check_sum2 += check_sum1;
    }
    //锟斤拷锟斤拷锟斤拷锟叫ｏ拷锟斤拷纸诤锟斤拷盏锟斤拷锟叫ｏ拷锟斤拷纸锟斤拷锟斤拷员龋锟斤拷锟饺伙拷麓锟斤拷锟斤拷锟街★拷锟斤拷莺戏锟斤拷锟斤拷锟揭伙拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟?
    if ((check_sum1 != rxFrame.frame.sc1) || (check_sum2 != rxFrame.frame.sc2)) //锟叫讹拷sum校锟斤拷
        return;
    /*******************************************/
    switch(rxFrame.frame.ID)
    {
        case 0xA2:
        {
            Con_flag=(s8)*((s8*)(rxFrame.frame.dataBuf + 0));
            Ano_Duoji=(s8)*((s8*)(rxFrame.frame.dataBuf + 2));
            zancun=(s16)*((s16*)(rxFrame.frame.dataBuf + 7));
            if(zancun!=0)
                qr_num=zancun;
            Send_qr_cun(qr_num);
        }
        break;
        case 0x31://锟劫度匡拷锟斤拷帧
        {			
            cmd_check_ms = 0; cmd_lost = 0;
            rosData.acc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
            rosData.acc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
            rosData.acc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
            rosData.yaw    = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
            Set_m_speed(rosData.acc[0],rosData.acc[1],rosData.acc[2],rosData.yaw);

        }
        break;
        case 0x32://位锟斤拷锟斤拷锟劫度ｏ拷t(原锟斤拷指t265)
        {			
            
            fbk_check_ms = 0; fbk_lost = 0;
            if(rxFrame.frame.dataLen >= 6)
            {
                rosData.tacc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
                rosData.tacc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
                rosData.tacc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
                // Velocity must come from tacc (the "speed" array), NOT tloc.
                // The old code fed tloc (position) into Set_m_speed_now, so the
                // +/-60 gate rejected it and 0x33 always reported 0x8000.
                Set_m_speed_now(rosData.tacc[0],rosData.tacc[1],rosData.tacc[2]);
                Send_speed_cun(rosData.tacc[0],rosData.tacc[1],rosData.tacc[2],0,0);
            }
            if(rxFrame.frame.dataLen >= 12)
            {
                rosData.tloc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 6));
                rosData.tloc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 8));
                rosData.tloc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 10));
            }
        }
        break;
        case 0x33://锟斤拷锟斤拷锟阶达拷位锟斤拷锟斤拷锟斤拷
        {
            rosData.loc[0] = (s16)*((s16*)(rxFrame.frame.dataBuf + 0));
            rosData.loc[1] = (s16)*((s16*)(rxFrame.frame.dataBuf + 2));
            rosData.loc[2] = (s16)*((s16*)(rxFrame.frame.dataBuf + 4));
        }break;
        case 0X63:
        {
            Send_route_now((s8)*((s8*)(rxFrame.frame.dataBuf+0)),(s16)*((s16*)(rxFrame.frame.dataBuf + 1)),(s16)*((s16*)(rxFrame.frame.dataBuf + 3)),(s16)*((s16*)(rxFrame.frame.dataBuf + 5)),(s16)*((s16*)(rxFrame.frame.dataBuf + 7)));
        }
        break;
        case 0x66://mission control (RDK -> FC)
        {
            if(rxFrame.frame.dataLen >= 3)
            {
                u8 sub = rxFrame.frame.dataBuf[0];
                mission_end_sub = sub;                       // echoed back in 0xF1
                if(sub == 0x01)                              // takeoff to h cm
                {
                    if(fc_sta.unlock_sta == 1 && mission_end_req == 0)
                    {
                        missionTakeoffAbort(0);   /* discard the old transaction */
                        mission_tk_h   = (u16)rxFrame.frame.dataBuf[1]
                                       | ((u16)rxFrame.frame.dataBuf[2] << 8);
                        mission_tk_req     = 1;
                        mission_tk_sent    = 0;   /* fresh transaction */
                        mission_tk_retry   = 0;
                        mission_tk_fail    = 0;
                        mission_tk_wait_ms = 0;
                    }
                    else
                    {
                        /* Reject late takeoff requests received while locked. */
                        missionTakeoffAbort(1);
                    }
                }
                else                                         // 0x02 land / 0x03 rth / 0x04 kill
                {
                    /* A stale takeoff request must never survive an end command. */
                    missionTakeoffAbort(0);
                    mission_end_req  = sub;
                    mission_end_flag = 1;
                    takeoff_if       = 0;                    // stop velocity commands at once
                }
            }
        }
        break;
        default:
            break;
    }
}


//
void AnoPTv7DrvSend(u8 linktype, u8 *buf, u16 len)
{
    DrvUart3SendBuf(buf,len);
}
//
//锟斤拷锟斤拷要锟斤拷锟酵碉拷帧写锟诫缓锟斤拷锟斤拷
static void anoPTv7AddTxFrame(u8 linkType, myTAnoPTv7Frame *pFrame)
{
    if(txFrameBuf[txBufWAddr].cmd._isUsed)
        return;
    txFrameBuf[txBufWAddr].cmd._isUsed = 1;
    txFrameBuf[txBufWAddr].cmd._linkType = linkType;
    for(u16 i = 0; i < (pFrame->frame.dataLen + 6); i++)
        txFrameBuf[txBufWAddr].data.rawBytes[i] = pFrame->rawBytes[i];
    
    txFrameBuf[txBufWAddr].cmd._isUsed = 2;
    
    txBufWAddr++;
    if(txBufWAddr >= ANOPTV8TXBUFNUM)
        txBufWAddr = 0;
}
//
static void anoDTRaspSendCheck(void)
{
    //锟斤拷锟斤拷锟斤拷突锟斤拷锟斤拷锟斤拷锟斤拷锟揭拷锟斤拷偷锟斤拷锟斤拷荩锟街达拷蟹锟斤拷锟?
    for(u8 i=0; i<ANOPTV8TXBUFNUM; i++)
    {
        if(txFrameBuf[txBufRAddr].cmd._isUsed == 2)
        {
            //锟斤拷锟斤拷写锟斤拷锟斤拷希锟斤拷锟斤拷梅锟斤拷锟?
            AnoPTv7DrvSend(txFrameBuf[txBufRAddr].cmd._linkType, txFrameBuf[txBufRAddr].data.rawBytes, txFrameBuf[txBufRAddr].data.frame.dataLen+6);
            txFrameBuf[txBufRAddr].cmd._isUsed = 0;
        }
        txBufRAddr++;
        if(txBufRAddr >= ANOPTV8TXBUFNUM)
        txBufRAddr = 0;
    }
}

//锟斤拷锟斤拷锟劫讹拷锟斤拷锟斤拷
void AnoPTv7SendVEL(s16 vel_x,s16 vel_y,s16 vel_z)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x07;
    txFrame.frame.dataLen = 6;
    txFrame.frame.dataBuf[0] = BYTE0(vel_x);
    txFrame.frame.dataBuf[1] = BYTE1(vel_x);
    
    txFrame.frame.dataBuf[2] = BYTE0(vel_y);
    txFrame.frame.dataBuf[3] = BYTE1(vel_y);
    
    txFrame.frame.dataBuf[4] = BYTE0(vel_z);
    txFrame.frame.dataBuf[5] = BYTE1(vel_z);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

// 2026-10-09 新增：8 路电机输出（单位 0.1%），帧 ID 0x09。
// 这 8 路是控制链最末端：IMU 板算完的混控输出，407 拿去驱动电调。
// 用途：判定航向慢摆是硬件还是软件 —— 输出在 8~12 秒周期上摆 = 飞控自己在驱动；
//       输出平 = 外部扰动，飞控只是被动在纠。
void AnoPTv7SendPWM(void)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x09;
    txFrame.frame.dataLen = 8;
    txFrame.frame.dataBuf[0] = BYTE0(pwm_to_esc.pwm_m1);
    txFrame.frame.dataBuf[1] = BYTE1(pwm_to_esc.pwm_m1);
    txFrame.frame.dataBuf[2] = BYTE0(pwm_to_esc.pwm_m2);
    txFrame.frame.dataBuf[3] = BYTE1(pwm_to_esc.pwm_m2);
    txFrame.frame.dataBuf[4] = BYTE0(pwm_to_esc.pwm_m3);
    txFrame.frame.dataBuf[5] = BYTE1(pwm_to_esc.pwm_m3);
    txFrame.frame.dataBuf[6] = BYTE0(pwm_to_esc.pwm_m4);
    txFrame.frame.dataBuf[7] = BYTE1(pwm_to_esc.pwm_m4);
    txFrame.frame.dataBuf[8] = BYTE0(pwm_to_esc.pwm_m5);
    txFrame.frame.dataBuf[9] = BYTE1(pwm_to_esc.pwm_m5);
    txFrame.frame.dataBuf[10] = BYTE0(pwm_to_esc.pwm_m6);
    txFrame.frame.dataBuf[11] = BYTE1(pwm_to_esc.pwm_m6);
    txFrame.frame.dataBuf[12] = BYTE0(pwm_to_esc.pwm_m7);
    txFrame.frame.dataBuf[13] = BYTE1(pwm_to_esc.pwm_m7);
    txFrame.frame.dataBuf[14] = BYTE0(pwm_to_esc.pwm_m8);
    txFrame.frame.dataBuf[15] = BYTE1(pwm_to_esc.pwm_m8);

    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

// 2026-10-10 新增：通用转发 —— 把 IMU 发来某一帧的"载荷"原样打包发给香橙派。
// 参数：id = 转发的帧 ID（保持与协议一致，便于上位机/日志对照）
//       payload = 载荷首地址，len = 载荷长度（不含帧头/长度/校验）
// 用途：目标姿态(0x0A)、目标速度(0x0B)、PWM控制量(0x20)、姿态控制量(0x21)
void AnoPTv7ForwardFrame(u8 id, const u8 *payload, u16 len)
{
    if (len > 200) return;                       // 防御：dataBuf 256 字节
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = id;
    txFrame.frame.dataLen = (u8)len;
    for (u16 i = 0; i < len; i++) txFrame.frame.dataBuf[i] = payload[i];

    u8 sc1 = 0, sc2 = 0;
    for (u16 i = 0; i < (txFrame.frame.dataLen + 4); i++) { sc1 += txFrame.rawBytes[i]; sc2 += sc1; }
    txFrame.rawBytes[txFrame.frame.dataLen + 4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen + 5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//锟斤拷锟斤拷GPS锟斤拷锟斤拷
//send attitude frame (Euler x100), ID 0x03
void AnoPTv7SendATT(s16 rol_x100,s16 pit_x100,s16 yaw_x100)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x03;
    txFrame.frame.dataLen = 6;
    txFrame.frame.dataBuf[0] = BYTE0(rol_x100);
    txFrame.frame.dataBuf[1] = BYTE1(rol_x100);
    txFrame.frame.dataBuf[2] = BYTE0(pit_x100);
    txFrame.frame.dataBuf[3] = BYTE1(pit_x100);
    txFrame.frame.dataBuf[4] = BYTE0(yaw_x100);
    txFrame.frame.dataBuf[5] = BYTE1(yaw_x100);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

void AnoPTv7SendGPS(s32 GPS_LNG,s32 GPS_LAT,s32 GPS_ALT)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x30;
    txFrame.frame.dataLen = 12;
    txFrame.frame.dataBuf[0] = BYTE0(GPS_LNG);
    txFrame.frame.dataBuf[1] = BYTE1(GPS_LNG);
    txFrame.frame.dataBuf[2] = BYTE2(GPS_LNG);
    txFrame.frame.dataBuf[3] = BYTE3(GPS_LNG);
 
    txFrame.frame.dataBuf[4] = BYTE0(GPS_LAT);
    txFrame.frame.dataBuf[5] = BYTE1(GPS_LAT);
    txFrame.frame.dataBuf[6] = BYTE2(GPS_LAT);
    txFrame.frame.dataBuf[7] = BYTE3(GPS_LAT);
    
    txFrame.frame.dataBuf[8] = BYTE0(GPS_ALT);
    txFrame.frame.dataBuf[9] = BYTE1(GPS_ALT);
    txFrame.frame.dataBuf[10] = BYTE2(GPS_ALT);
    txFrame.frame.dataBuf[11] = BYTE3(GPS_ALT);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//锟斤拷锟斤拷imu锟斤拷锟斤拷锟斤拷锟斤拷元锟斤拷
void AnoPTv7SendIMU(s16 accx,s16 accy,s16 accz,s16 gyr_x,s16 gyr_y,s16 gyr_z,s16 v0,s16 v1,s16 v2,s16 v3)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x01;
    txFrame.frame.dataLen = 20;
    txFrame.frame.dataBuf[0] = BYTE0(accx);
    txFrame.frame.dataBuf[1] = BYTE1(accx);

    txFrame.frame.dataBuf[2] = BYTE0(accy);
    txFrame.frame.dataBuf[3] = BYTE1(accy);

    txFrame.frame.dataBuf[4] = BYTE0(accz);
    txFrame.frame.dataBuf[5] = BYTE1(accz);

    txFrame.frame.dataBuf[6] = BYTE0(gyr_x);
    txFrame.frame.dataBuf[7] = BYTE1(gyr_x);

    txFrame.frame.dataBuf[8] = BYTE0(gyr_y);
    txFrame.frame.dataBuf[9] = BYTE1(gyr_y);

    txFrame.frame.dataBuf[10] = BYTE0(gyr_z);
    txFrame.frame.dataBuf[11] = BYTE1(gyr_z);
    
    txFrame.frame.dataBuf[12] = BYTE0(v0);
    txFrame.frame.dataBuf[13] = BYTE1(v0);
    
    txFrame.frame.dataBuf[14] = BYTE0(v1);
    txFrame.frame.dataBuf[15] = BYTE1(v1);
    
    txFrame.frame.dataBuf[16] = BYTE0(v2);
    txFrame.frame.dataBuf[17] = BYTE1(v2);
    
    txFrame.frame.dataBuf[18] = BYTE0(v3);
    txFrame.frame.dataBuf[19] = BYTE1(v3);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//锟斤拷锟斤拷锟矫伙拷锟斤拷锟斤拷
void AnoPTv7SendUserData(u8 d1,u8 d2,u8 d3,s16 d4,s16 d5,s16 d6)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0xF1;
    txFrame.frame.dataLen = 16;
    txFrame.frame.dataBuf[0] = d1;
    txFrame.frame.dataBuf[1] = d2;
    txFrame.frame.dataBuf[2] = d3;

    txFrame.frame.dataBuf[3] = BYTE0(d4);
    txFrame.frame.dataBuf[4] = BYTE1(d4);

    txFrame.frame.dataBuf[5] = BYTE0(d5);
    txFrame.frame.dataBuf[6] = BYTE1(d5);

    txFrame.frame.dataBuf[7] = BYTE0(d6);
    txFrame.frame.dataBuf[8] = BYTE1(d6);

    //----- core-board echo, appended 2026-09-13 -----
    // The F407 has always parsed these out of the IMU board 0x06 frame
    // (ANO_DT_LX.c:189) and then thrown them away -- fc_mode_sta in
    // particular has ZERO readers. That is why every "motors do not spin"
    // investigation has been guesswork: the board says which mode it is in
    // and whether it is armed, and nobody listens.
    // cmd_fun is the IMU board echoing the last command it actually executed,
    // which is the only thing that closes the send-vs-accept gap.
    txFrame.frame.dataBuf[9]  = fc_sta.fc_mode_sta;
    txFrame.frame.dataBuf[10] = fc_sta.unlock_sta;
    txFrame.frame.dataBuf[11] = fc_sta.fc_mode_cmd;
    txFrame.frame.dataBuf[12] = fc_sta.unlock_cmd;
    txFrame.frame.dataBuf[13] = fc_sta.cmd_fun.CID;
    txFrame.frame.dataBuf[14] = fc_sta.cmd_fun.CMD_0;
    txFrame.frame.dataBuf[15] = fc_sta.cmd_fun.CMD_1;
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
//锟斤拷锟酵猴拷锟斤拷锟斤拷锟斤拷
void AnoPTv7SendRouteData(u8 d1,u8 d2)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x62;
    txFrame.frame.dataLen = 2;
    txFrame.frame.dataBuf[0] = d1;
    txFrame.frame.dataBuf[1] = d2;
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
//锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷
void AnoPTv7SendMAG(s16 mag_x,s16 mag_y,s16 mag_z,s16 tmp)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x02;
    txFrame.frame.dataLen = 8;
    txFrame.frame.dataBuf[0] = BYTE0(mag_x);
    txFrame.frame.dataBuf[1] = BYTE1(mag_x);
    
    txFrame.frame.dataBuf[2] = BYTE0(mag_y);
    txFrame.frame.dataBuf[3] = BYTE1(mag_y);
    
    txFrame.frame.dataBuf[4] = BYTE0(mag_z);
    txFrame.frame.dataBuf[5] = BYTE1(mag_z);
    
    txFrame.frame.dataBuf[6] = BYTE0(tmp);
    txFrame.frame.dataBuf[7] = BYTE1(tmp);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//锟斤拷锟酵高讹拷锟斤拷锟斤拷
//send FC control target frame (ID 0x41): rol/pit x100deg, thr 0-1000, yaw_dps deg/s, vel cm/s
void AnoPTv7SendCTRL(s16 rol,s16 pit,s16 thr,s16 yaw_dps,s16 vel_x,s16 vel_y,s16 vel_z)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x41;
    txFrame.frame.dataLen = 14;
    txFrame.frame.dataBuf[0] = BYTE0(rol);
    txFrame.frame.dataBuf[1] = BYTE1(rol);
    txFrame.frame.dataBuf[2] = BYTE0(pit);
    txFrame.frame.dataBuf[3] = BYTE1(pit);
    txFrame.frame.dataBuf[4] = BYTE0(thr);
    txFrame.frame.dataBuf[5] = BYTE1(thr);
    txFrame.frame.dataBuf[6] = BYTE0(yaw_dps);
    txFrame.frame.dataBuf[7] = BYTE1(yaw_dps);
    txFrame.frame.dataBuf[8] = BYTE0(vel_x);
    txFrame.frame.dataBuf[9] = BYTE1(vel_x);
    txFrame.frame.dataBuf[10] = BYTE0(vel_y);
    txFrame.frame.dataBuf[11] = BYTE1(vel_y);
    txFrame.frame.dataBuf[12] = BYTE0(vel_z);
    txFrame.frame.dataBuf[13] = BYTE1(vel_z);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

void AnoPTv7SendALT(s16 alt)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x05;
    txFrame.frame.dataLen = 2;
    txFrame.frame.dataBuf[0] = BYTE0(alt);
    txFrame.frame.dataBuf[1] = BYTE1(alt);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}

//锟斤拷锟斤拷遥锟斤拷锟斤拷锟斤拷锟斤拷
void AnoPTv7SendRC(u16 ch_1,u16 ch_2,u16 ch_3,u16 ch_4,u16 ch_5,u16 ch_6,u16 ch_7,u16 ch_8,u16 ch_9,u16 ch_10)
{
    myTAnoPTv7Frame txFrame;
    txFrame.frame.head = 0xAA;
    txFrame.frame.addr = 0xFF;
    txFrame.frame.ID = 0x40;
    txFrame.frame.dataLen = 20;
    txFrame.frame.dataBuf[0] = BYTE0(ch_1);
    txFrame.frame.dataBuf[1] = BYTE1(ch_1);

    txFrame.frame.dataBuf[2] = BYTE0(ch_2);
    txFrame.frame.dataBuf[3] = BYTE1(ch_2);

    txFrame.frame.dataBuf[4] = BYTE0(ch_3);
    txFrame.frame.dataBuf[5] = BYTE1(ch_3);

    txFrame.frame.dataBuf[6] = BYTE0(ch_4);
    txFrame.frame.dataBuf[7] = BYTE1(ch_4);

    txFrame.frame.dataBuf[8] = BYTE0(ch_5);
    txFrame.frame.dataBuf[9] = BYTE1(ch_5);

    txFrame.frame.dataBuf[10] = BYTE0(ch_6);
    txFrame.frame.dataBuf[11] = BYTE1(ch_6);

    txFrame.frame.dataBuf[12] = BYTE0(ch_7);
    txFrame.frame.dataBuf[13] = BYTE1(ch_7);
    
    txFrame.frame.dataBuf[14] = BYTE0(ch_8);
    txFrame.frame.dataBuf[15] = BYTE1(ch_8);
    
    txFrame.frame.dataBuf[16] = BYTE0(ch_9);
    txFrame.frame.dataBuf[17] = BYTE1(ch_9);
    
    txFrame.frame.dataBuf[18] = BYTE0(ch_10);
    txFrame.frame.dataBuf[19] = BYTE1(ch_10);
    
    u8 sc1 = 0;
    u8 sc2 = 0;
    for(u16 i=0; i<(txFrame.frame.dataLen+4); i++)
    {
        sc1 += txFrame.rawBytes[i];
        sc2 += sc1;
    }
    txFrame.rawBytes[txFrame.frame.dataLen+4] = sc1;
    txFrame.rawBytes[txFrame.frame.dataLen+5] = sc2;
    anoPTv7AddTxFrame(LinkType_ToRasp, &txFrame);
}
