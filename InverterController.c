

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

	pInvCon->RmpId.Step = 0.2;
	pInvCon->RmpId.MinErr = 0.1;
	
	pInvCon->RmpIq.Step = 0.2;
	pInvCon->RmpIq.MinErr = 0.1;

	pInvCon->SMCn.c  = 60;
	pInvCon->SMCn.mu = 100;
	pInvCon->SMCn.q  = 300;
	pInvCon->SMCn.limit_high = 30;
	pInvCon->SMCn.limit_low  = -30;
	pInvCon->SMCn.Ts =      CON_TS;
	pInvCon->SMCn.last_output=0;

	

	pInvCon->PidId.Kp = 16.4;
	pInvCon->PidId.Ki = R*100;
	pInvCon->PidId.Ts = CON_TS;
	pInvCon->PidId.LimitHigh = 161;
	pInvCon->PidId.LimitLow = -161;
	
	pInvCon->PidIq.Kp = 16.4;
	pInvCon->PidIq.Ki = R*500;
	pInvCon->PidIq.Ts = CON_TS;
	pInvCon->PidIq.LimitHigh = 161;
	pInvCon->PidIq.LimitLow = -161;
}


void calcInverterControl(pINVERTER_CONTROLLER pInvCon)
{
	// pInvCon->VolPll.Ua = InverterControl.Ua;
	// pInvCon->VolPll.Ub = InverterControl.Ub;
	// pInvCon->VolPll.Uc = InverterControl.Uc;
	// calcPLLReg(&pInvCon->VolPll);
	float max_v=0;
	float min_v=0;
	pInvCon->CurClark.Ua = InverterControl.Ia;
	pInvCon->CurClark.Ub = InverterControl.Ib;
	pInvCon->CurClark.Uc = InverterControl.Ic;
	calcClarkReg(&pInvCon->CurClark);
	
	pInvCon->CurPark.Alpha = pInvCon->CurClark.Alpha;
	pInvCon->CurPark.Beta  = pInvCon->CurClark.Beta;
	pInvCon->CurPark.theta = pInvCon->theta;
	calcParkReg(&pInvCon->CurPark);
	
	pInvCon->RmpId.Target = pInvCon->TargetId;
	calcRmp(&pInvCon->RmpId);
	
	pInvCon->Rmpn.Target = pInvCon->Targetn;
	calcRmp(&pInvCon->Rmpn);
	
	pInvCon->PidId.Ref = pInvCon->RmpId.outputValue;
	pInvCon->PidId.Fdb = pInvCon->CurPark.Ud;
	calcPidReg(&pInvCon->PidId);
	
	pInvCon->SMCn.Ref = pInvCon->Rmpn.outputValue;
	pInvCon->SMCn.Fdb = pInvCon->n * rad2rpm;
	pInvCon->SMCn.last_Err = 0;
	calcSMCReg(&pInvCon->SMCn);   //滑膜转速控制器
    
    pInvCon->PidIq.Ref =pInvCon->SMCn.SMC_output;
    pInvCon->PidIq.Fdb = pInvCon->CurPark.Uq;
    calcPidReg(&pInvCon->PidIq);

	
	pInvCon->VolIPark.Ud =  pInvCon->PidId.PidOut +(-Pn*pInvCon->n*Lq*pInvCon->PidIq.Fdb);
	pInvCon->VolIPark.Uq =  pInvCon->PidIq.PidOut +(Pn*pInvCon->n*(flux+Ld*pInvCon->PidId.Fdb));
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

void calcDspInterrupt(double *nthetafdb, double *Iabc, double *targetIdn, double *PwmRef, double *Display)
{


	InverterControl.Ia = Iabc[0];
	InverterControl.Ib = Iabc[1];
	InverterControl.Ic = Iabc[2];

	InverterControl.Targetn   = targetIdn[1];
	InverterControl.TargetId  = targetIdn[0];
	InverterControl.theta = nthetafdb[1];
	InverterControl.n     = nthetafdb[0];

	calcInverterControl(&InverterControl);
	
	PwmRef[0] = InverterControl.OutPwmRef[0];
	PwmRef[1] = InverterControl.OutPwmRef[1];
	PwmRef[2] = InverterControl.OutPwmRef[2];
	
	Display[0] = InverterControl.PidId.Ref;
	Display[1] = InverterControl.PidId.Fdb;
	Display[2] = InverterControl.PidId.PidOut;
	Display[3] = InverterControl.PidIq.Ref;
	Display[4] = InverterControl.PidIq.Fdb;
	Display[5] = InverterControl.PidIq.PidOut;
	Display[6] = InverterControl.SMCn.Ref;
	Display[7] = InverterControl.SMCn.Fdb;
	Display[8] = InverterControl.SMCn.SMC_output;
	Display[9] = InverterControl.VolIClark.Ua;
	Display[10] = InverterControl.VolIClark.Ub;
	Display[11] = InverterControl.VolIClark.Uc;
	Display[12] = InverterControl.theta;
}