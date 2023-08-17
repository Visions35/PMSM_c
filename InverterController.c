

#include "ControlFunction.h"
#include "InverterController.h"

INVERTER_CONTROLLER InverterControl;

// void calcPLLReg(pPLL_REG v)
// {
// 	v->clarkUabc.Ua = v->Ua;
// 	v->clarkUabc.Ub = v->Ub;
// 	v->clarkUabc.Uc = v->Uc;
// 	calcClarkReg(&v->clarkUabc);
	
// 	v->parkUabc.Alpha = v->clarkUabc.Alpha;
// 	v->parkUabc.Beta  = v->clarkUabc.Beta;
// 	v->parkUabc.theta = v->theta;
// 	calcParkReg(&v->parkUabc);
	
// 	v->pidPll.Ref = v->parkUabc.Ud;
// 	v->pidPll.Fdb = 0;
// 	calcPidReg(&v->pidPll);
	
// 	v->Frequency = v->pidPll.PidOut + v->ForwardFrequency;
// 	v->theta += PI_2*v->Frequency*v->Ts;
	
// 	if (v->theta>PI_2)
// 	{
// 		v->theta -= PI_2;
// 	}
// 	else if (v->theta<0)
// 	{
// 		v->theta += PI_2;
// 	}
	
// 	v->Ud = v->parkUabc.Ud;
// 	v->Uq = v->parkUabc.Uq;
// }

// void restartPLLReg(pPLL_REG v)
// {
// 	restartPidReg(&v->pidPll);
// 	v->Ud = 0;
// 	v->Uq = 0;
// 	v->Frequency = 0;
// 	v->theta = 0;
// }

void initInverterContr(pINVERTER_CONTROLLER pInvCon)
{
	// pInvCon->VolPll.pidPll.Kp = 0.2;
	// pInvCon->VolPll.pidPll.Ki = 2.0;
	// pInvCon->VolPll.pidPll.Ts = CON_TS;
	// pInvCon->VolPll.Ts= CON_TS;
	// pInvCon->VolPll.ForwardFrequency = 50;
	// pInvCon->VolPll.pidPll.LimitHigh = 200;
	// pInvCon->VolPll.pidPll.LimitLow = -200;
	pInvCon->Rmpn.Step = 2;
	pInvCon->Rmpn.MinErr =1 ;

	pInvCon->Rmpflux.Step = 0.1;
	pInvCon->Rmpflux.MinErr = 0.1;
	
//滑膜转矩初始化
	pInvCon->SMCTe.ki  = 15000;
	pInvCon->SMCTe.kp  = 100;
	pInvCon->SMCTe.Ts =      CON_TS;
	pInvCon->SMCTe.last_sign=0;

	
//滑膜磁链初始化
	pInvCon->SMCflux.ki  = 15000;
	pInvCon->SMCflux.kp  = 100;
	pInvCon->SMCflux.Ts =      CON_TS;
	pInvCon->SMCflux.last_sign=0;
	
	pInvCon->Pidn.Kp = 0.05;
	pInvCon->Pidn.Ki = 20;
	pInvCon->Pidn.Ts = CON_TS;
	pInvCon->Pidn.LimitHigh = 50;
	pInvCon->Pidn.LimitLow = -50;

	pInvCon->LPF_Te.a=0.4966;
	pInvCon->LPF_Te.b=0.5034;
	pInvCon->LPF_Te.last_output=0;
	
	pInvCon->last_flux=0;
}


void calcInverterControl(pINVERTER_CONTROLLER pInvCon)
{
	// pInvCon->VolPll.Ua = InverterControl.Ua;
	// pInvCon->VolPll.Ub = InverterControl.Ub;
	// pInvCon->VolPll.Uc = InverterControl.Uc;
	// calcPLLReg(&pInvCon->VolPll);
	float max_v=0;
	float min_v=0;
	
//电压Clark park变换
	pInvCon->VolClark.Ua = InverterControl.Ua;
	pInvCon->VolClark.Ub = InverterControl.Ub;
	pInvCon->VolClark.Uc = InverterControl.Uc;
	calcClarkReg(&pInvCon->VolClark);

	pInvCon->VolPark.Alpha = pInvCon->VolClark.Alpha;
	pInvCon->VolPark.Beta  = pInvCon->VolClark.Beta;
	pInvCon->VolPark.theta = pInvCon->theta;
	calcParkReg(&pInvCon->VolPark);

//电流Clark park变换
	pInvCon->CurClark.Ua = InverterControl.Ia;
	pInvCon->CurClark.Ub = InverterControl.Ib;
	pInvCon->CurClark.Uc = InverterControl.Ic;
	calcClarkReg(&pInvCon->CurClark);
	
	pInvCon->CurPark.Alpha = pInvCon->CurClark.Alpha;
	pInvCon->CurPark.Beta  = pInvCon->CurClark.Beta;
	pInvCon->CurPark.theta = pInvCon->theta;
	calcParkReg(&pInvCon->CurPark);
	

//转速给定柔化
	pInvCon->Rmpn.Target = pInvCon->Targetn;
	calcRmp(&pInvCon->Rmpn);

//转速控制器
	pInvCon->Pidn.Ref = pInvCon->Rmpn.outputValue;
	pInvCon->Pidn.Fdb = pInvCon->n;
	calcPidReg(&pInvCon->Pidn);

//磁链估算
	pInvCon->current_flux = CON_TS*(pInvCon->VolPark.Ud-R*pInvCon->CurPark.Ud)+pInvCon->last_flux;
	pInvCon->last_flux = pInvCon->current_flux;

//SMC磁链控制器
	pInvCon->SMCflux.Ref = pInvCon->Targetflux;
	pInvCon->SMCflux.Fdb = pInvCon->current_flux;
	calcSMCReg(&pInvCon->SMCflux);   //滑膜磁链控制器
 
//转矩滤波
    pInvCon->LPF_Te.filter_input=pInvCon->Te;
    calcLPFFILTER(&pInvCon->LPF_Te);

//转矩SMC控制器
    pInvCon->SMCTe.Ref = pInvCon->Pidn.PidOut;
	pInvCon->SMCTe.Fdb = pInvCon->LPF_Te.filter_output;
	calcSMCReg(&pInvCon->SMCTe);  

//电压反变换
	pInvCon->VolIPark.Ud =  pInvCon->SMCflux.SMC_output ;
	pInvCon->VolIPark.Uq =  pInvCon->SMCTe.SMC_output ;
	pInvCon->VolIPark.theta = pInvCon->theta;
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

void calcDspInterrupt(double *Uabc, double *Iabc, double *ref_flux_n, double *fed_n_theta_Te,double *PWMRef, double *Display_Te, double *Display_n, double *Display_flux,double *Display)
{
	InverterControl.Ua = Uabc[0];
	InverterControl.Ub = Uabc[1];
	InverterControl.Uc = Uabc[2];

	InverterControl.Ia = Iabc[0];
	InverterControl.Ib = Iabc[1];
	InverterControl.Ic = Iabc[2];

	InverterControl.Targetn     = ref_flux_n[1];
	InverterControl.Targetflux  = ref_flux_n[0];
	
	InverterControl.n     = fed_n_theta_Te[0]*rad2rpm;
	InverterControl.theta = fed_n_theta_Te[1]*Pn-0.5*3.1415926;
	InverterControl.Te    = fed_n_theta_Te[2];
	

	calcInverterControl(&InverterControl);
	
	PWMRef[0] = InverterControl.OutPwmRef[0];
	PWMRef[1] = InverterControl.OutPwmRef[1];
	PWMRef[2] = InverterControl.OutPwmRef[2];
	
	Display_Te[0] = InverterControl.SMCTe.Ref;
	Display_Te[1] = InverterControl.SMCTe.Fdb;
	Display_Te[2] = InverterControl.SMCTe.SMC_output;
	Display_n[0] = InverterControl.Pidn.Ref;
	Display_n[1] = InverterControl.Pidn.Fdb;
	Display_n[2] = InverterControl.Pidn.PidOut;
	Display_flux[0] = InverterControl.SMCflux.Ref;
	Display_flux[1] = InverterControl.SMCflux.Fdb;
	Display_flux[2] = InverterControl.SMCflux.SMC_output;
	Display[0] = InverterControl.VolIClark.Ua;
	Display[1] = InverterControl.VolIClark.Ub;
	Display[2] = InverterControl.VolIClark.Uc;
	Display[3] = InverterControl.CurPark.Ud;
	Display[4] = InverterControl.CurPark.Uq;
	Display[5] = InverterControl.VolPark.Ud;
	Display[6] = InverterControl.VolPark.Uq;
	Display[7] = InverterControl.Te;
}