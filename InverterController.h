
#ifndef _INVERTER_CONTROLLER_H_
#define _INVERTER_CONTROLLER_H_
#include "ControlFunction.h"

typedef struct PLL_REG_X
{
	float Ua, Ub, Uc;
	float Ts;
	float ForwardFrequency;
	CLARK_REG clarkUabc;
	PARK_REG  parkUabc;
	PID_REG pidPll;

	float theta;
	float Frequency;
	float Ud, Uq;
}PLL_REG, *pPLL_REG;

void calcPLLReg(pPLL_REG v);
void restartPLLReg(pPLL_REG v);

//---------------------------------------------Inverter Controller
typedef struct INVERTER_CONTROLLER_X
{
	
	
	CLARK_REG CurClark;
	PARK_REG CurPark;

	I_CLARK_REG VolIClark;
	I_PARK_REG VolIPark;
	
	RMP_CTR RmpId, RmpIq,Rmpn;
	PID_REG PidId, PidIq ;
	SMC_REG SMCn;

	float TargetId;
	float TargetIq;
	float Targetn;
	
	float n;
	float theta; 
	float Ia, Ib, Ic;
	float OutPwmRef[3];
	
}INVERTER_CONTROLLER, *pINVERTER_CONTROLLER;

void initInverterContr(pINVERTER_CONTROLLER pInvCon);
void calcInverterControl(pINVERTER_CONTROLLER pInvCon);

void initDspInterrupt(void);
void calcDspInterrupt(double *nthetafdb, double *Iabc, double *targetIdn, double *PwmRef, double *Display);

extern INVERTER_CONTROLLER InverterControl;

#endif