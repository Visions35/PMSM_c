#ifndef _OBSERVER_H_
#define _OBSERVER_H_
#include "Observer.h"
#include "ControlFunction.h"

#define LUENBERGER_GAIN_1 30      // 30       // 30     // 30    // 20  // Large gain to position will cause steady state position error, but increase it close to limit
#define LUENBERGER_GAIN_2 (900)    // (750)    // (300)  // (300) // 100 // If speed estimate has too much dynamics during reversal, you need to increase this gain actually...
#define LUENBERGER_GAIN_3 (1500) // (0*6000) // (1500) // (790) // 500 // Tune reversal response to slight over-shoot

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
    TWO_orderLPF LPF_Ealpha2,LPF_Ebeta2,LPF_n;
    // LPF_FILTER LPF_Ealpha,LPF_Ebeta;
    LPF_FILTER LPF_omega;

}SlideMode_Obser, *pSlideMode_Obser;

void initSMO(pSlideMode_Obser v);
void calcSMO(pSlideMode_Obser v);

typedef struct Luenberger_x
{
    double test_signal_al;
    double test_signal_be;
    double test_signal_d;
    double test_signal_q;
    double d_lpf;
    double q_lpf;
    double d_hpf;
    double q_hpf;
    double theta_filter;
    double theta_d_raw;//未处理过的位置信号
    double theta_d;//处理过的位置信号
    double omg_elec;
    
    double pseudo_load_torque;//含参数的负载转矩
    double mismatch;      //角度偏差
    
    PARK_REG CurPark_LPF,CurPark_HPF;
    I_PARK_REG InvCurPark_LPF,InvCurPark_HPF;

}Luenberger ,*pLuenberger;

void dynamics_lpf(double input, double *state, double *derivative);

void RK4_111_general(void (*pointer_dynamics)(), double input, double *state, double hs);

void RK4_333_general(void (*pointer_dynamics)(), double input, double *state, double hs);
void init_luenberger(pLuenberger v);

double difference_between_two_angles(double first, double second);
void dynamics_position_observer(double input, double *state, double *derivative);
// void luenberger_filter(double theta_d_raw);
void hfsi_do(pLuenberger v);
#endif

