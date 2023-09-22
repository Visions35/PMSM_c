#ifndef _PMSMSim_H_
#define _PMSMSim_H_


#include <stdio.h> // printf #include <stdbool.h> // bool for _Bool and true for 1
#include <process.h>//reqd. for system function prototype
#include <conio.h> // for clrscr, and getch()
#include "stdlib.h" // for rand()
#include "math.h"
#include "time.h"

#define HFSI_ON 1
#define CON_TS 0.00025 //4000hz
#define PI 3.14159265358979323846
#define flux_f 0.646
#define NUMBER_OF_STATES 4 // valid for PMSM
#define TS_UPSAMPLING_FREQ_EXE 0.5
#define TS_UPSAMPLING_FREQ_EXE_INVERSE 2
#define SENSORLESS_CONTROL true
#define SENSORLESS_CONTROL_HFSI true
#define rpm2rad 3.141592654/30
#define rad2rpm 30/3.141592654
#define true (1)
#define false (0)
#define NUMBER_OF_STEPS 150000
#define DATA_FILE_NAME "pmsm_hfsi.dat"
#define DOWN_SAMPLE 1
#define MACHINE_TS         (CON_TS*TS_UPSAMPLING_FREQ_EXE) //1.25e-4 
#define LPF_TIME_CONST_INVERSE (5*2*PI)
#define rk 1 //1代表4阶
typedef struct PMSMSim_x
{
    double x[NUMBER_OF_STATES];
    double dotx[NUMBER_OF_STATES];
    double rpm;//转速单位为转每分钟
    double rpm_cmd;
    double rpm_deriv_cmd;
    double omegae;
    double Tload;
    double Tem;

    double Ua,Ub,Uc;
    double Ia,Ib,Ic;
    
    double Ualpha,Ubeta;
    double Ialpha,Ibeta;
    
    double Ud,Uq;
    double Id,Iq;

    double Ld,Lq;
    double R;
    double Ts;

    double J;//转动惯量
    double theta_d;
    double F;//阻尼系数
    double np;
    double TS;

    double RinvLd,RinvLq;

    double eemf_q;
    double eemf_al,eemf_be;
    double theta_d__eemf;


}PMSMSim, *pPMSMSim;

void PMSM_init();
void RK_dynamics(double t, double *x, double *fx);
void RK_Linear(double t, double *x, double hs);
void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
int isNumber(double x);

#endif