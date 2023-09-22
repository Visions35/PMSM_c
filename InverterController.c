

#include "ControlFunction.h"
#include "InverterController.h"
#include "Observer.h"
#include <math.h>

INVERTER_CONTROLLER InverterControl;
extern PMSMSim PMSM;
int i=0;
float theta=0.0, omega =0.0;

void initInverterContr(pINVERTER_CONTROLLER pInvCon)
{
	// pInvCon->VolPll.pidPll.Kp = 0.2;
	// pInvCon->VolPll.pidPll.Ki = 2.0;
	// pInvCon->VolPll.pidPll.Ts = CON_TS;
	// pInvCon->VolPll.Ts= CON_TS;
	// pInvCon->VolPll.ForwardFrequency = 50;
	// pInvCon->VolPll.pidPll.LimitHigh = 200;
	// pInvCon->VolPll.pidPll.LimitLow = -200;
	pInvCon->timebase = 0.0;
	pInvCon->Rmpn.Step = 2;
	pInvCon->Rmpn.MinErr =1 ;
//初始化龙伯格观测器
	init_luenberger(&pInvCon->Luenberger_obser);
	
	pInvCon->Targetn  = 60 ;
	pInvCon->Targetid = 0;

	
//转速pi初始化
	// pInvCon->Pidn.Kp  = 0.02548532;
	pInvCon->Pidn.Kp  = 0.0648532;
	pInvCon->Pidn.Ki  = 1.2158;
	// pInvCon->Pidn.Kd  = 0.001220697;
	pInvCon->Pidn.Ts  = CON_TS;
	pInvCon->Pidn.LimitHigh = 20;
	pInvCon->Pidn.LimitLow  = -20;

	
//dq轴电流初始化初始化
	pInvCon->Pid_iq.Kp = 22.33548;
	pInvCon->Pid_iq.Ki = 8272.4;
	pInvCon->Pid_iq.Kd = 0;
	pInvCon->Pid_iq.Ts = CON_TS;
	pInvCon->Pid_iq.LimitHigh = 350;
	pInvCon->Pid_iq.LimitLow = -350;
	
	pInvCon->Pid_id.Kp = 22.33548;
	pInvCon->Pid_id.Ki = 8272.4;
	pInvCon->Pid_id.Kd = 0;
	pInvCon->Pid_id.Ts = CON_TS;
	pInvCon->Pid_id.LimitHigh = 350;
	pInvCon->Pid_id.LimitLow = -350;
	printf("speedloop pid parameter:%f,%f\n",pInvCon->Pidn.Kp,pInvCon->Pidn.Ki);
	printf("dcurrentloop pid parameter:%f,%f\n",pInvCon->Pid_id.Kp,pInvCon->Pid_id.Ki);
	printf("qcurrentloop pid parameter:%f,%f\n",pInvCon->Pid_iq.Kp,pInvCon->Pid_iq.Ki);
	
}


void calcInverterControl(pINVERTER_CONTROLLER pInvCon)
{
	
	// #if SENSORLESS_CONTROL

	pInvCon->omega_fb = pInvCon->Luenberger_obser.omg_elec;
	pInvCon->theta_fb = pInvCon->Luenberger_obser.theta_d;

	pInvCon->Rmpn.Target = pInvCon->Targetn;
	calcRmp(&pInvCon->Rmpn);

	pInvCon->Pidn.Ref = pInvCon->Rmpn.outputValue;
	pInvCon->Pidn.Fdb = pInvCon->omega_fb * rad2rpm / PMSM.np;
	calcPidReg(&pInvCon->Pidn);

	pInvCon->Pid_id.Ref = pInvCon->Targetid ;
	pInvCon->Pid_id.Fdb = PMSM.Id;
	calcPidReg(&pInvCon->Pid_id);

	pInvCon->Pid_iq.Ref = pInvCon->Pidn.PidOut;
	pInvCon->Pid_iq.Fdb = PMSM.Iq;
	calcPidReg(&pInvCon->Pid_iq); 
	
	pInvCon->Tem_ref = PMSM.np*1.5*(pInvCon->Pidn.PidOut*flux_f +pInvCon->Targetid*pInvCon->Pidn.PidOut*(PMSM.Ld-PMSM.Lq));
	pInvCon->Tem=PMSM.np*1.5*(PMSM.Iq*flux_f +PMSM.Id*PMSM.Iq*(PMSM.Ld-PMSM.Lq));
	
	PMSM.Ud = pInvCon->Pid_id.PidOut;
	PMSM.Uq = pInvCon->Pid_iq.PidOut;
	
	#ifdef HFSI_ON
        // Extra excitation for observation
        {
            static int square_wave_internal_register = 1;
            static int dfe_counter = 0; 
            #define HFSI_VOLTAGE 5 // V
            #define HFSI_CEILING 1
            if(dfe_counter++==HFSI_CEILING){
                dfe_counter = 0;
                square_wave_internal_register *= -1;
            }
            PMSM.Ud += HFSI_VOLTAGE*square_wave_internal_register;
        }
    #endif

	pInvCon->VolIPark.Ud = PMSM.Ud;
	pInvCon->VolIPark.Uq = PMSM.Uq;
	pInvCon->VolIPark.theta = pInvCon->theta_fb;
	calcI_ParkReg(&pInvCon->VolIPark);
	PMSM.Ualpha = pInvCon->VolIPark.Alpha;
	PMSM.Ubeta  = pInvCon->VolIPark.Beta; 
	// printf("calcluenberger is doing\n");

}

void initDspInterrupt(void)
{
	initInverterContr(&InverterControl);
}

// void calcDspInterrupt(double *Uabc, double *Iabc, double *ref_id_n,double *theta_n ,double *PWMRef, double *Display_id, double *Display_n, double *Display_iq,double *Display)
// {
// 	InverterControl.Ua = Uabc[0];
// 	InverterControl.Ub = Uabc[1];
// 	InverterControl.Uc = Uabc[2];

// 	InverterControl.Ia = Iabc[0];
// 	InverterControl.Ib = Iabc[1];
// 	InverterControl.Ic = Iabc[2];

// 	InverterControl.Targetn     = ref_id_n[1];
// 	InverterControl.Targetid    = ref_id_n[0];

// 	InverterControl.theta       = theta_n[1];
// 	InverterControl.n           = theta_n[0];
	

// 	calcInverterControl(&InverterControl);
	
// 	PWMRef[0] = InverterControl.OutPwmRef[0];
// 	PWMRef[1] = InverterControl.OutPwmRef[1];
// 	PWMRef[2] = InverterControl.OutPwmRef[2];
	
// 	Display_id[0] = InverterControl.Pid_id.Ref;
// 	Display_id[1] = InverterControl.Pid_id.Fdb;
// 	Display_id[2] = InverterControl.Pid_id.PidOut;
// 	Display_n[0]  = InverterControl.Pidn.Ref;
// 	Display_n[1]  = InverterControl.Pidn.Fdb;
// 	Display_n[2]  = InverterControl.Pidn.PidOut;
// 	Display_iq[0] = InverterControl.Pid_iq.Ref;
// 	Display_iq[1] = InverterControl.Pid_iq.Fdb;
// 	Display_iq[2] = InverterControl.Pid_iq.PidOut;
// 	Display[0]    = InverterControl.SMO_theta.Ealpha_ob;
// 	Display[1]    = InverterControl.SMO_theta.Ebeta_ob;
// 	Display[2]    = InverterControl.theta;
// 	Display[3]    = InverterControl.SMO_theta.omega*rad2rpm/Pn;
// 	Display[4]    = InverterControl.n;
// 	Display[5]    = InverterControl.SMO_theta.theta;
// 	Display[6]    = InverterControl.SMO_theta.Ealpha;
// 	Display[7]    = InverterControl.SMO_theta.Ebeta;
// 	Display[8]    = InverterControl.CurClark.Alpha;
// 	Display[9]	  = InverterControl.CurClark.Beta;
// 	Display[10]   = InverterControl.SMO_theta.sum1;
// 	Display[11]   = InverterControl.SMO_theta.sum2;
// }