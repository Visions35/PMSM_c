
#ifndef _INVERTER_CONTROLLER_H_
#define _INVERTER_CONTROLLER_H_
#include "ControlFunction.h"
#include "Observer.h"
#include "PMSMSim.h"

//---------------------------------------------Inverter Controller
typedef struct INVERTER_CONTROLLER_X
{
	
	double timebase;
	CLARK_REG CurClark;
	PARK_REG CurPark;

	CLARK_REG VolClark;
	PARK_REG VolPark;

	I_CLARK_REG VolIClark;
	I_PARK_REG VolIPark;
	
	RMP_CTR Rmpn;
	PID_REG Pidn,Pid_id,Pid_iq;
	Luenberger  Luenberger_obser;

	double Targetid;
	double Targetn;


	double Ua, Ub, Uc;
	double Ia, Ib, Ic;

	double Id,Iq;
	double Ud,Uq;
	double theta_fb,omega_fb;
	
	double Ialpha,Ibeta;
	double Tem,Tem_ref;
	
	
}INVERTER_CONTROLLER, *pINVERTER_CONTROLLER;

void initInverterContr(pINVERTER_CONTROLLER pInvCon);
void calcInverterControl(pINVERTER_CONTROLLER pInvCon);

void initDspInterrupt(void);
// void calcDspInterrupt(double *Uabc, double *Iabc, double *ref_id_n, double *theta_n,double *PWMRef, double *Display_id, double *Display_n, double *Display_iq,double *Display);


extern INVERTER_CONTROLLER InverterControl;

#endif