
#ifndef _CONTROL_FUNCTION_H_
#define _CONTROL_FUNCTION_H_

#define CON_TS 1e-5
#define PI_2 3.141592654*2

#define Pn 4
#define R 2.8750
#define L 8.5e-3
#define J 0.0008
#define rpm2rad 3.141592654/30
#define rad2rpm 30/3.141592654
#define PI 3.141592654

#define flux 0.175
#define invL  117.64706
#define RinvL 338.23529
#define invflux 5.714286
#define omegac 100
#define piinv2 1.570796
#define cutoff_fre 5

//PID结构体
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

//滑膜控制器结构体
typedef struct SMC_REG_X
{
	float ki;
	float kp;
	float Ts;

	float Ref;
	float Fdb;

	float Err;
	float sign_Err;
	float last_sign;
	float fabs_Err;
	float sum1;
	float sum2;

	float SMC_output;
}SMC_REG, *pSMC_REG;


//低通滤波器结构体
typedef struct LPF_FILTER_X
{
	float omega_c;
	float a,b;
	float Ts;

	float last_output;
	
	float filter_output;
	
	float filter_input;

}LPF_FILTER,*pLPF_FILTER;

typedef struct TWO_orderLPF_X
{
	float b0 , b1 , b2;
	float a1 , a2 ;
	
	float x1 , x2 ; 
	float out1 , out2;

	float cutoff_freq;
	float Ts;

	float LPF_input;
	float LPF_output;

}TWO_orderLPF,*pTWO_orderLPF;

//参考信号柔化结构体
typedef struct RMP_CTR_X
{
	float Err;
	float MinErr;
	float Step;

	float Target;
	float outputValue;
}RMP_CTR, *pRMP_CTR;

//低通滤波
void calcLPFFILTER(pLPF_FILTER v);

int sign(float v);

//滑膜控制器
void calcSMCReg(pSMC_REG v);
void restartSMCReg(pSMC_REG v);
void calcPidReg(pPID_REG v);
void restartPidReg(pPID_REG v);
void calcRmp(pRMP_CTR v);
void restartRmp(pRMP_CTR v);

//二阶低通滤波
void initTWOorderLPF(pTWO_orderLPF v);
void calcTWOorderLPF(pTWO_orderLPF v);

//clark变换结构体
typedef struct CLARK_REG_X
{
	float Ua, Ub, Uc;
	float Alpha, Beta, Zero;
}CLARK_REG, *pCLARK_REG;

void calcClarkReg(pCLARK_REG v);


//clark反变换结构体
typedef struct I_CLARK_REG_X
{
	float Ua, Ub, Uc;
	float Alpha, Beta, Zero;
}I_CLARK_REG, *pI_CLARK_REG;

void calcI_ClarkReg(pI_CLARK_REG v);


//park变换结构体
typedef struct PARK_REG_X
{
	float Alpha, Beta;
	float Ud, Uq;
	float theta;
	float SinTemp, CosTemp;
}PARK_REG, *pPARK_REG;

void calcParkReg(pPARK_REG v);


//反park变换结构体
typedef struct I_PARK_REG_X
{
	float Alpha, Beta;
	float Ud, Uq;
	float theta;
	float SinTemp, CosTemp;
}I_PARK_REG, *pI_PARK_REG;

void calcI_ParkReg(pI_PARK_REG v);


#endif