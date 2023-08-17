
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

	CLARK_REG VolClark;
	PARK_REG VolPark;

	I_CLARK_REG VolIClark;
	I_PARK_REG VolIPark;
	
	RMP_CTR Rmpflux,Rmpn;
	PID_REG Pidn;
	SMC_REG SMCTe,SMCflux;
	LPF_FILTER LPF_Te;

	float TargetTe;
	float Targetflux;
	float Targetn;
	
	float n;
	float theta; 
	float Te,flux;
	float Ua, Ub, Uc;
	float Ia, Ib, Ic;
	float OutPwmRef[3];

	float last_flux;
	float current_flux;
	
}INVERTER_CONTROLLER, *pINVERTER_CONTROLLER;

void initInverterContr(pINVERTER_CONTROLLER pInvCon);
void calcInverterControl(pINVERTER_CONTROLLER pInvCon);

void initDspInterrupt(void);
void calcDspInterrupt(double *Uabc, double *Iabc, double *ref_flux_n, double *fed_n_theta_Te,double *PWMRef, double *Display_Te, double *Display_n, double *Display_flux,double *Display);

extern INVERTER_CONTROLLER InverterControl;

#endif