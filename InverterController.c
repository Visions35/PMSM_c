

#include "ControlFunction.h"
#include "InverterController.h"
#include "Observer.h"
#include <math.h>
INVERTER_CONTROLLER InverterControl;
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
	
	pInvCon->Rmpn.Step = 10;
	pInvCon->Rmpn.MinErr =1 ;

	// pInvCon->SMO_theta.Ts = CON_TS ;
	// pInvCon->SMO_theta.ksw = 200;
	// pInvCon->SMO_theta.LPF_Ealpha.omega_c = omegac;
	// pInvCon->SMO_theta.LPF_Ealpha.Ts = CON_TS;
	// pInvCon->SMO_theta.LPF_Ebeta .omega_c = omegac;
	// pInvCon->SMO_theta.LPF_Ebeta .Ts = CON_TS;
	// pInvCon->SMO_theta.last_sum1 = 0;
	// pInvCon->SMO_theta.last_sum2 = 0;
	// pInvCon->SMO_theta.sum1 = 0;
	// pInvCon->SMO_theta.sum2 = 0;
	// pInvCon->SMO_theta.LPF_Ealpha.last_output = 0;
	// pInvCon->SMO_theta.LPF_Ebeta .last_output = 0;
	// pInvCon->SMO_theta.Ealpha = 0;
	// pInvCon->SMO_theta.Ebeta  = 0;
	initSMO(&pInvCon->SMO_theta);
	initTWOorderLPF(&pInvCon->SMO_theta.LPF_Ealpha2);
    initTWOorderLPF(&pInvCon->SMO_theta.LPF_Ebeta2);
	pInvCon->SMO_theta.LPF_n.omega = cutoff_fre; 
	initTWOorderLPF(&pInvCon->SMO_theta.LPF_n);
	
//转速pi初始化
	pInvCon->Pidn.Kp  = 0.02548532;
	pInvCon->Pidn.Ki  = 0.5158;
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
	
}


void calcInverterControl(pINVERTER_CONTROLLER pInvCon)
{
	
	// pInvCon->VolPll.Ua = InverterControl.Ua;
	// pInvCon->VolPll.Ub = InverterControl.Ub;
	// pInvCon->VolPll.Uc = InverterControl.Uc;
	// calcPLLReg(&pInvCon->VolPll);
	float max_v=0;
	float min_v=0;
	
//电压Clark变换
	pInvCon->VolClark.Ua = InverterControl.Ua;
	pInvCon->VolClark.Ub = InverterControl.Ub;
	pInvCon->VolClark.Uc = InverterControl.Uc;
	calcClarkReg(&pInvCon->VolClark);

//电流Clark变换
	pInvCon->CurClark.Ua = InverterControl.Ia;
	pInvCon->CurClark.Ub = InverterControl.Ib;
	pInvCon->CurClark.Uc = InverterControl.Ic;
	calcClarkReg(&pInvCon->CurClark);
	
//SMO
	pInvCon->SMO_theta.Ualpha     = InverterControl.VolClark.Alpha;
	pInvCon->SMO_theta.Ubeta      = InverterControl.VolClark.Beta;
	pInvCon->SMO_theta.Ialpha_fdb = InverterControl.CurClark.Alpha;
	pInvCon->SMO_theta.Ibeta_fdb  = InverterControl.CurClark.Beta;
	calcSMO(&pInvCon->SMO_theta);
	
	
	if (i<delaytime)
	{
		omega = pInvCon->n;
		theta = pInvCon->theta;
	}else
	{
		omega = pInvCon->SMO_theta.omega*rad2rpm/Pn;
		theta = pInvCon->SMO_theta.theta;
	}
	i++;

//电流park变换 
	pInvCon->CurPark.Alpha  = pInvCon->CurClark.Alpha;
	pInvCon->CurPark.Beta   = pInvCon->CurClark.Beta;
	pInvCon->CurPark.theta  = theta ;
	// pInvCon->VolIPark.theta = pInvCon->theta;
	calcParkReg(&pInvCon->CurPark);

//转速给定柔化
	pInvCon->Rmpn.Target = pInvCon->Targetn;
	calcRmp(&pInvCon->Rmpn);

//转速控制器
	pInvCon->Pidn.Ref = pInvCon->Rmpn.outputValue;
	pInvCon->Pidn.Fdb = omega;
	// pInvCon->Pidn.Fdb = pInvCon->n;
	calcPidReg(&pInvCon->Pidn);
	

//d轴电流控制器
	pInvCon->Pid_id.Ref = pInvCon->Targetid;
	pInvCon->Pid_id.Fdb = pInvCon->CurPark.Ud;
	calcPidReg(&pInvCon->Pid_id);

//q轴电流控制器
	pInvCon->Pid_iq.Ref = pInvCon->Pidn.PidOut;
	pInvCon->Pid_iq.Fdb = pInvCon->CurPark.Uq;
	calcPidReg(&pInvCon->Pid_iq);

//电压反变换
	pInvCon->VolIPark.Ud    =  pInvCon->Pid_id.PidOut - pInvCon->SMO_theta.omega * pInvCon->CurPark.Uq * L;
	pInvCon->VolIPark.Uq    =  pInvCon->Pid_iq.PidOut + pInvCon->SMO_theta.omega * (pInvCon->CurPark.Ud * L + flux )  ;
	pInvCon->VolIPark.theta =  theta;
	// pInvCon->VolIPark.theta =  pInvCon->theta;
	calcI_ParkReg(&pInvCon->VolIPark);

	pInvCon->VolIClark.Alpha = pInvCon->VolIPark.Alpha;
	pInvCon->VolIClark.Beta  = pInvCon->VolIPark.Beta;
	calcI_ClarkReg(&pInvCon->VolIClark);
	
	if( pInvCon->VolIClark.Ua>max_v )
		max_v = pInvCon->VolIClark.Ua;
	if(pInvCon->VolIClark.Ub>max_v)
		max_v = pInvCon->VolIClark.Ub;
	if(pInvCon->VolIClark.Uc>max_v)
		max_v = pInvCon->VolIClark.Uc;
	if (pInvCon->VolIClark.Ua<min_v)
		min_v = pInvCon->VolIClark.Ua;
	if (pInvCon->VolIClark.Ub<min_v)
		min_v = pInvCon->VolIClark.Ub;
	if (pInvCon->VolIClark.Uc<min_v)
		min_v = pInvCon->VolIClark.Uc;
		
	
	pInvCon->OutPwmRef[0] = (pInvCon->VolIClark.Ua -(max_v+min_v)*0.5)* 0.003215;
	pInvCon->OutPwmRef[1] = (pInvCon->VolIClark.Ub -(max_v+min_v)*0.5)* 0.003215;
	pInvCon->OutPwmRef[2] = (pInvCon->VolIClark.Uc -(max_v+min_v)*0.5)* 0.003215;
}

void initDspInterrupt(void)
{
	initInverterContr(&InverterControl);
}

void calcDspInterrupt(double *Uabc, double *Iabc, double *ref_id_n,double *theta_n ,double *PWMRef, double *Display_id, double *Display_n, double *Display_iq,double *Display)
{
	InverterControl.Ua = Uabc[0];
	InverterControl.Ub = Uabc[1];
	InverterControl.Uc = Uabc[2];

	InverterControl.Ia = Iabc[0];
	InverterControl.Ib = Iabc[1];
	InverterControl.Ic = Iabc[2];

	InverterControl.Targetn     = ref_id_n[1];
	InverterControl.Targetid    = ref_id_n[0];

	InverterControl.theta       = theta_n[1];
	InverterControl.n           = theta_n[0];
	

	calcInverterControl(&InverterControl);
	
	PWMRef[0] = InverterControl.OutPwmRef[0];
	PWMRef[1] = InverterControl.OutPwmRef[1];
	PWMRef[2] = InverterControl.OutPwmRef[2];
	
	Display_id[0] = InverterControl.Pid_id.Ref;
	Display_id[1] = InverterControl.Pid_id.Fdb;
	Display_id[2] = InverterControl.Pid_id.PidOut;
	Display_n[0]  = InverterControl.Pidn.Ref;
	Display_n[1]  = InverterControl.Pidn.Fdb;
	Display_n[2]  = InverterControl.Pidn.PidOut;
	Display_iq[0] = InverterControl.Pid_iq.Ref;
	Display_iq[1] = InverterControl.Pid_iq.Fdb;
	Display_iq[2] = InverterControl.Pid_iq.PidOut;
	Display[0]    = InverterControl.SMO_theta.Ealpha_ob;
	Display[1]    = InverterControl.SMO_theta.Ebeta_ob;
	Display[2]    = InverterControl.theta;
	Display[3]    = InverterControl.SMO_theta.omega*rad2rpm/Pn;
	Display[4]    = InverterControl.n;
	Display[5]    = InverterControl.SMO_theta.theta;
	Display[6]    = InverterControl.SMO_theta.Ealpha;
	Display[7]    = InverterControl.SMO_theta.Ebeta;
	Display[8]    = InverterControl.CurClark.Alpha;
	Display[9]	  = InverterControl.CurClark.Beta;
	Display[10]   = InverterControl.SMO_theta.sum1;
	Display[11]   = InverterControl.SMO_theta.sum2;
}