#include "ControlFunction.h"
#include "InverterController.h"
#include "Observer.h"
#include "math.h"
extern float theta;
extern int i;

void calcPLLReg(pPLL_REG v)
{
	
    
	v->alpha_ob = -1 * v->alpha * cos(v->theta);
	v->beta_ob  = v->beta *sin(v->theta);
    
   
	v->pidPll.Ref = v->alpha_ob;
	v->pidPll.Fdb = v->beta_ob ;
	calcPidReg(&v->pidPll);

	v->omega = v->pidPll.PidOut;
	v->theta = v->omega * v->Ts + v->last_theta;
	v->last_theta = v->theta;
	v->theta = fmodf(v->theta - piinv2 , 2*PI);
	
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
    v->PLL_theta.pidPll.Ki = 20000;
    v->PLL_theta.pidPll.Kd = 0;
    v->PLL_theta.pidPll.Ts = CON_TS;
    v->PLL_theta.pidPll.LimitHigh = 1000;
    v->PLL_theta.pidPll.LimitLow  = 0;
   
    // v->LPF_omega.omega_c = cutoff_fre;
    // v->LPF_omega.Ts =CON_TS ;
    // v->LPF_omega.last_output=0;
    

}

void calcSMO(pSlideMode_Obser v)
{

    float x=0;
    
    v->err1 = v-> Ualpha * invL - RinvL * v->last_sum1 - invL * v->Ealpha;
    // v->err1 = v-> Ualpha * invL - RinvL * v->Ialpha_fdb - invL * v->Ealpha;
    v->sum1 = v->last_sum1 + v->err1 * v->Ts;

    v->err2 = v-> Ubeta  * invL - RinvL * v->last_sum2 - invL * v->Ebeta;
    // v->err2 = v-> Ubeta  * invL - RinvL * v->Ibeta_fdb - invL * v->Ebeta;
    v->sum2 = v->last_sum2 + v->err2 * v->Ts;
    // v->sum1 = v->A * v->last_sum1 + v->B * (v->Ualpha - v->Ealpha) ;
    // v->sum2 = v->A * v->last_sum2 + v->B * (v->Ubeta  - v->Ebeta ) ;

    v->last_sum1 = v->sum1;
    v->last_sum2 = v->sum2;
    
    v->Ialpha = v->sum1 - v->Ialpha_fdb ;
    v->Ibeta  = v->sum2 - v->Ibeta_fdb  ;

    v->Ealpha = sign(v->Ialpha) * v->ksw;
    v->Ebeta  = sign(v->Ibeta)  * v->ksw;
    //v->Ealpha,v->Ebeta包含有有用的位置信息
   

    v->LPF_Ealpha2.LPF_input = v->Ealpha;
    calcTWOorderLPF(&v->LPF_Ealpha2);
    v->Ealpha_ob = v->LPF_Ealpha2.LPF_output;

    v->LPF_Ebeta2.LPF_input = v->Ebeta;
    calcTWOorderLPF(&v->LPF_Ebeta2);
    v->Ebeta_ob = v->LPF_Ebeta2.LPF_output;
    
    
    
    v->PLL_theta.alpha = v->Ealpha_ob;
    v->PLL_theta.beta  = v->Ebeta_ob;
    calcPLLReg(&v->PLL_theta) ;
   
    v->omega = v->PLL_theta.omega;
    // 转速滤波
    // v->LPF_omega.filter_input = v->omega;
    // calcLPFFILTER(&v->LPF_omega);
    // v->omega = v->LPF_omega.filter_output;
    
    v->LPF_n.LPF_input = v->omega;
    calcTWOorderLPF(&v->LPF_n);
    v->omega = v->LPF_n.LPF_output;


    v->delta_theta = atan2f( v->omega , omegac  );


    if (i< delaytime)
    {
        // v->theta = v->PLL_theta.theta;
        v->theta = theta;
    }
    else
    {
        v->theta = v->PLL_theta.theta + 0.8*v->delta_theta ;
    }
    
    v->theta = fmodf(v->theta , 2*PI);



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
