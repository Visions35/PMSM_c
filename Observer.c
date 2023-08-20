#include "ControlFunction.h"
#include "InverterController.h"
#include "Observer.h"
#include "math.h"

void calcPLLReg(pPLL_REG v)
{
	// v->clarkUabc.Ua = v->Ua;
	// v->clarkUabc.Ub = v->Ub;
	// v->clarkUabc.Uc = v->Uc;
	// calcClarkReg(&v->clarkUabc);
	
	// v->parkUabc.Alpha = v->clarkUabc.Alpha;
	// v->parkUabc.Beta  = v->clarkUabc.Beta;
	// v->parkUabc.theta = v->theta;
	// calcParkReg(&v->parkUabc);
	v->alpha_ob = -1 * v->alpha * cos(v->theta);
	v->beta_ob  = v->beta *sin(v->theta);

	v->pidPll.Ref = v->alpha_ob;
	v->pidPll.Fdb = v->beta_ob;
	calcPidReg(&v->pidPll);
	
	// v->Frequency = v->pidPll.PidOut + v->ForwardFrequency;
	// v->theta += PI_2*v->Frequency*v->Ts;
	v->omega = v->pidPll.PidOut;
	v->theta = v->omega * v->Ts + v->last_theta;
	v->last_theta = v->theta;
	v->theta = fmodf(v->theta - piinv2 ,2*PI);
	
	// v->Ud = v->parkUabc.Ud;
	// v->Uq = v->parkUabc.Uq;
}

// void restartPLLReg(pPLL_REG v)
// {
// 	restartPidReg(&v->pidPll);
// 	v->Ud = 0;
// 	v->Uq = 0;
// 	v->Frequency = 0;
// 	v->theta = 0;
// }

void initSMO(pSlideMode_Obser v)
{
    v->Ts = CON_TS ;
    v->A = exp(-RinvL * v->Ts);
    v->B = 1/R * (1 - v->A);
    v->ksw = 200;
    v->last_sum1 = 0 ;
    v->last_sum2 = 0 ;
    v->sum1 = 0;
    v->sum2 = 0;
    v->Ealpha = 0;
    v->Ebeta  =  0;
    v->sum_theta = 0;
    v->PLL_theta.last_theta=0;
    v->PLL_theta.theta =0 ;
    v->PLL_theta.Ts =CON_TS;
    v->PLL_theta.pidPll.Kp = 100;
    v->PLL_theta.pidPll.Ki = 20000.0;
    v->PLL_theta.pidPll.Ts = CON_TS;
    v->PLL_theta.pidPll.LimitHigh = 1000;
    v->PLL_theta.pidPll.LimitLow  = 0;
    v->LPF_omega.omega_c = cutoff_fre;
    v->LPF_omega.Ts =CON_TS ;
    v->LPF_omega.last_output=0;
    

}

void calcSMO(pSlideMode_Obser v)
{

    float x=0;
    
    // v->err1 = v-> Ualpha * invL - RinvL * v->last_sum1 - invL * v->Ealpha;
    // v->sum1 = v->last_sum1 + v->err1 * v->Ts;

    // v->err2 = v-> Ubeta  * invL - RinvL * v->last_sum2 - invL * v->Ebeta;
    // v->sum2 = v->last_sum2 + v->err2 * v->Ts;
    v->sum1 = v->A * v->last_sum1 + v->B * (v->Ualpha - v->Ealpha) ;
    v->sum2 = v->A * v->last_sum2 + v->B * (v->Ubeta  - v->Ebeta ) ;

    v->last_sum1 = v->sum1;
    v->last_sum2 = v->sum2;
    
    v->Ialpha = v->sum1 - v->Ialpha_fdb ;
    v->Ibeta  = v->sum2 - v->Ibeta_fdb  ;

    v->Ealpha = sign(v->Ialpha) * v->ksw;
    v->Ebeta  = sign(v->Ibeta)  * v->ksw;
    
   

    v->LPF_Ealpha2.LPF_input = v->Ealpha;
    calcTWOorderLPF(&v->LPF_Ealpha2);
    v->Ealpha_ob = v->LPF_Ealpha2.LPF_output;

    v->LPF_Ebeta2.LPF_input = v->Ebeta;
    calcTWOorderLPF(&v->LPF_Ebeta2);
    v->Ebeta_ob = v->LPF_Ebeta2.LPF_output;

    v->PLL_theta.alpha = v->Ealpha_ob;
    v->PLL_theta.beta  = v->Ebeta_ob;
    calcPLLReg(&v->PLL_theta) ;

    v->theta = v->PLL_theta.theta;
    v->omega = v->PLL_theta.omega;

//转速滤波
    v->LPF_omega.filter_input = v->omega;
    calcLPFFILTER(&v->LPF_omega);
    v->omega = v->LPF_omega.filter_output;


//dont need
    // v->LPF_Ealpha.filter_input = v->Ealpha;
    // calcLPFFILTER(&v->LPF_Ealpha);
    // v->Ealpha_ob = v->LPF_Ealpha.filter_output;
    
    // v->LPF_Ebeta.filter_input  = v->Ebeta;
    // calcLPFFILTER(&v->LPF_Ebeta);
    // v->Ebeta_ob  = v->LPF_Ebeta.filter_output;

//need
    // v->omega = sqrt(pow(v->Ealpha_ob,2) + pow(v->Ebeta_ob,2)) * invflux;
    //  v->delta_theta = atan2f( v->omega , omegac  );

//dont need   
    // v->theta_ob    = atan2f(-1*v->Ealpha_ob , v->Ebeta_ob) +(-1 * v->Ebeta_ob + abs(v->Ebeta_ob))*piinv2/ abs(v->Ebeta_ob);

//need 
    // v->theta_ob    = atan2f(-1*v->Ealpha_ob , v->Ebeta_ob);

//dont need
    // v->theta_ob = fmodf(v->theta_ob , 2 * PI)  ;

//need    
    // if (v->theta_ob > 0)
    // {
    //     if (v->Ebeta_ob < 0 )
    //     {
    //         x = PI + v->theta_ob;
    //     }
    //     else if (v->Ebeta_ob > 0)
    //     {
    //         x = v->theta_ob ;
    //     }else if (v->Ebeta_ob == 0)
    //     {
    //         x = 0.5 * PI ;
    //     }
    // }
    // else if (v->theta_ob < 0)
    // {
    //     if (v->Ebeta_ob < 0)
    //     {
    //         x = PI + v->theta_ob;
    //     }
    //     else if (v->Ebeta_ob > 0)
    //     {
    //         x = 2 * PI + v->theta_ob;
    //     }
    //     else if (v->Ebeta_ob == 0)
    //     {
    //         x = 1.5 * PI;
    //     }
    // }
    // else if (v->theta_ob == 0)
    // {
    //     if (v->Ebeta_ob < 0)
    //     {
    //         x = PI ;
    //     }
    //     else if (v->Ebeta_ob > 0)
    //     {
    //         x = 0;
    //     }
    //     else if (v->Ebeta_ob == 0)
    //     {
    //         x = 0;
    //     }
          
    // }

//dont need
    //  v->theta = PI + v->theta;

//need  
    // v->theta = v->delta_theta + x ;

//dont need
    // v->sum_theta += v->theta ;

//need  
    //  v->theta = fmodf(v->theta , 2 * PI);
    
}

void initMRAS(pMRAS_obser v)
{
    v->Ts = CON_TS;
    v->last_hatid = 0;
    v->last_hatiq = 0;
    v->last_theta = 0;
    v->Pid_i.Kp = 0.5;
    v->Pid_i.Ki = 0.3;
    v->Pid_i.Ts =CON_TS ;
    v->Pid_i.LimitHigh = 1000;
    v->Pid_i.LimitLow  = -100;
}

void calcMRAS(pMRAS_obser v)
{
    v->sum1 = v->Ud * invL - RinvL * v->hat_id + v->omegae * v->hat_iq;
    v->hat_id   = v->Ts * v->sum1 + v->last_hatid;
    
    v->sum2 = v->Uq * invL - RinvL * v->hat_iq - v->omegae * flux * invL - v->omegae * v->hat_id ;
    v->hat_iq   = v->Ts * v->sum2 + v->last_hatiq;

    v->last_hatid = v->hat_id;
    v->last_hatiq = v->hat_iq;

    // v->error = v->id * v->hat_iq - v->iq * v->hat_id - flux * invL* (v->iq - v->hat_iq);
    v->Pid_i.Ref = v->id * v->hat_iq;
    v->Pid_i.Fdb = v->iq * v->hat_id + flux * invL* (v->iq - v->hat_iq);
    calcPidReg(&v->Pid_i);

    v->omegae = v->Pid_i.PidOut ; 
    v->theta = v->omegae * v->Ts + v->last_theta;
    v->last_theta = v->theta;
    v->theta = fmodf(v->theta, 2*PI);
}