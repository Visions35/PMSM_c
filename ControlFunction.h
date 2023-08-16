
#ifndef _CONTROL_FUNCTION_H_
#define _CONTROL_FUNCTION_H_

#define CON_TS 1e-4
#define PI_2 3.141592654*2
#define flux 0.175
#define Pn 4
#define R 2.875
#define Ld 8.2e-3
#define Lq 8.2e-3

#define rpm2rad 3.1415926154/30
#define rad2rpm 30/3.141592654

typedef struct PID_REG_X
{
	float Kp;
	float Ki;
	float Ts;

	float Ref;
	float Fdb;

	float LimitHigh;
	float LimitLow;

	float Err;

	float PidKpOut;
	float PidKiOut;

	float PidOut;
}PID_REG, *pPID_REG;

typedef struct SMC_REG_X
{
	float c;
	float mu;
	float q;
	float Ts;

	float Ref;
	float Fdb;

	float Err;
	float last_Err;
	float last_output;
	float current_Err;
	float sum1;
	float sum2;

	float limit_high;
	float limit_low;

	float deri_out;

	float SMC_output;
}SMC_REG, *pSMC_REG;


int sign(float v);
void calcSMCReg(pSMC_REG v);
void restartSMCReg(pSMC_REG v);
void calcPidReg(pPID_REG v);
void restartPidReg(pPID_REG v);

typedef struct RMP_CTR_X
{
	float Err;
	float MinErr;
	float Step;

	float Target;
	float outputValue;
}RMP_CTR, *pRMP_CTR;

void calcRmp(pRMP_CTR v);
void restartRmp(pRMP_CTR v);

typedef struct CLARK_REG_X
{
	float Ua, Ub, Uc;
	float Alpha, Beta, Zero;
}CLARK_REG, *pCLARK_REG;

void calcClarkReg(pCLARK_REG v);

typedef struct I_CLARK_REG_X//clark反变换
{
	float Ua, Ub, Uc;
	float Alpha, Beta, Zero;
}I_CLARK_REG, *pI_CLARK_REG;

void calcI_ClarkReg(pI_CLARK_REG v);

typedef struct PARK_REG_X
{
	float Alpha, Beta;
	float Ud, Uq;
	float theta;
	float SinTemp, CosTemp;
}PARK_REG, *pPARK_REG;

void calcParkReg(pPARK_REG v);

typedef struct I_PARK_REG_X
{
	float Alpha, Beta;
	float Ud, Uq;
	float theta;
	float SinTemp, CosTemp;
}I_PARK_REG, *pI_PARK_REG;

void calcI_ParkReg(pI_PARK_REG v);


#endif