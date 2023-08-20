#ifndef _OBSERVER_H_
#define _OBSERVER_H_
#include "Observer.h"
#include "ControlFunction.h"


typedef struct PLL_REG_X
{
	// float Ua, Ub, Uc;
	float alpha,beta;
    float alpha_ob,beta_ob;
	float Ts;
	float ForwardFrequency;
	CLARK_REG clarkUabc;
	PARK_REG  parkUabc;
	PID_REG pidPll;

	float omega;
	float theta;
	// float Frequency;
	float last_theta;
	// float Ud, Uq;
}PLL_REG, *pPLL_REG;

void calcPLLReg(pPLL_REG v);
void restartPLLReg(pPLL_REG v);

typedef struct SlideMode_Obser_X
{
    float Ualpha, Ubeta;
    float Ialpha_fdb, Ibeta_fdb;
    float Ts;
    float A,B;

    float sum1,sum2;
    float ksw;
    float last_sum1,last_sum2;
    float Ealpha,Ebeta;
    float Ealpha_ob,Ebeta_ob;
    float err1,err2;
    
    float Ialpha,Ibeta;
    float omega;
    float delta_theta;
    float theta_ob;
    float theta;
    float sum_theta;

    PLL_REG PLL_theta;
    TWO_orderLPF LPF_Ealpha2,LPF_Ebeta2;
    // LPF_FILTER LPF_Ealpha,LPF_Ebeta;
    LPF_FILTER LPF_omega;

}SlideMode_Obser, *pSlideMode_Obser;



typedef struct MRAS_obser_x
{
    float Ts;
    float Ud,Uq;
    float hat_id,hat_iq;
    float last_hatid,last_hatiq;
    float sum1,sum2;

    float error;
    float id,iq;
    float omegae;
    float last_theta;
    float theta;
    PID_REG Pid_i;
}MRAS_obser , *pMRAS_obser ; 

void initSMO(pSlideMode_Obser v);
void calcSMO(pSlideMode_Obser v);
void initMRAS(pMRAS_obser v);
void calcMRAS(pMRAS_obser v);

#endif

