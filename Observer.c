#include "ControlFunction.h"
#include "InverterController.h"
#include "Observer.h"
#include "PMSMSim.h"
#include "math.h"
extern float theta;
extern int i;
extern PMSMSim PMSM;
extern INVERTER_CONTROLLER InverterControl;
#if  HFSI_ON


//龙格库塔法，低通滤波器， 1阶ode
 void dynamics_lpf(double input, double *state, double *derivative){
        derivative[0] = LPF_TIME_CONST_INVERSE * ( input - *state );
    }
void RK4_111_general(void (*pointer_dynamics)(), double input, double *state, double hs){

        #define NS 1

        double k1[NS], k2[NS], k3[NS], k4[NS], intemediate_state[NS];
        double derivative[NS];
        int i;

        pointer_dynamics(input, state, derivative); // timer.t,
        for(i=0;i<NS;++i){        
            k1[i] = derivative[i] * hs;
            intemediate_state[i] = state[i] + k1[i]*0.5;
        }

        pointer_dynamics(input, intemediate_state, derivative); // timer.t+hs/2., 
        for(i=0;i<NS;++i){        
            k2[i] = derivative[i] * hs;
            intemediate_state[i] = state[i] + k2[i]*0.5;
        }
        
        pointer_dynamics(input, intemediate_state, derivative); // timer.t+hs/2., 
        for(i=0;i<NS;++i){        
            k3[i] = derivative[i] * hs;
            intemediate_state[i] = state[i] + k3[i];
        }
        
        pointer_dynamics(input, intemediate_state, derivative); // timer.t+hs, 
        for(i=0;i<NS;++i){        
            k4[i] = derivative[i] * hs;
            state[i] = state[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0;
        }
        #undef NS
}


void dynamics_position_observer(double input, double *state, double *derivative){
    #define TEM_USED PMSM.Tem
    // #define TEM_USED ACM.Tem
    InverterControl.Luenberger_obser.mismatch = difference_between_two_angles(input, state[0]); // difference in angle
    derivative[0] = LUENBERGER_GAIN_1*InverterControl.Luenberger_obser.mismatch + state[1];
    derivative[1] = LUENBERGER_GAIN_2*InverterControl.Luenberger_obser.mismatch + TEM_USED*PMSM.np/PMSM.J - state[2];
    derivative[2] = -LUENBERGER_GAIN_3*InverterControl.Luenberger_obser.mismatch;
    //  printf("%g, %g, %g\n", InverterControl.timebase, state[2], derivative[2]);
}
    
//三阶ode
void RK4_333_general(void (*pointer_dynamics)(), double input, double *state, double hs){
        #define NS 3
        double k1[NS], k2[NS], k3[NS], k4[NS], intemediate_state[NS];
        double derivative[NS];
        int i;

        pointer_dynamics(input, state, derivative); // timer.t,
        for(i=0;i<NS;++i){        
            k1[i] = derivative[i] * hs;
            intemediate_state[i] = state[i] + k1[i]*0.5;
        }

        pointer_dynamics(input, intemediate_state, derivative); // timer.t+hs/2., 
        for(i=0;i<NS;++i){        
            k2[i] = derivative[i] * hs;
            intemediate_state[i] = state[i] + k2[i]*0.5;
        }
        
        pointer_dynamics(input, intemediate_state, derivative); // timer.t+hs/2., 
        for(i=0;i<NS;++i){        
            k3[i] = derivative[i] * hs;
            intemediate_state[i] = state[i] + k3[i];
        }
        
        pointer_dynamics(input, intemediate_state, derivative); // timer.t+hs, 
        for(i=0;i<NS;++i){        
            k4[i] = derivative[i] * hs;
            state[i] = state[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0;
        }
        #undef NS
       
}



void init_luenberger(pLuenberger v)
{
    v->test_signal_al= 0.0;
    v->test_signal_be= 0.0;
    v->test_signal_d = 0.0;
    v->test_signal_q = 0.0;
    v->d_lpf = 0.0;
    v->q_lpf = 0.0;
    v->d_hpf = 0.0;
    v->q_lpf = 0.0;
    v->theta_filter = 0.0;
    v->theta_d_raw  = 0.0;
    v->theta_d      = 0.0;
    v->omg_elec     = 0.0;
    v->pseudo_load_torque =0.0;
    v->mismatch     = 0.0;
    printf("luenberger initial\n");

}


//角度作差
double difference_between_two_angles(double first, double second){
        while(first>2*PI){
            first-=2*PI;
        }
        while(second>2*PI){
            second-=2*PI;
        }

        while(first<0.0){
            first+=2*PI;
        }
        while(second<0.0){
            second+=2*PI;
        }

        if(fabs(first-second)<PI){
            return first-second;
        }else{
            if(first>second){
                return first-2*PI-second;
            }else{                
                return first+2*PI-second;
            }
        }
}



void luenberger_filter(double theta_d_raw){
    static double state[3];
    RK4_333_general(dynamics_position_observer, theta_d_raw, state, CON_TS);
    // if(state[0]>PI){
    //         state[0] -= 2*PI;
    // }else if(state[0]<-PI){
    //         state[0] += 2*PI;
    // }
    
    InverterControl.Luenberger_obser.theta_d            = state[0] - 0.495*PI; // DEBUG HERE
    // InverterControl.Luenberger_obser.theta_d            = state[0]+atan2(LPF_TIME_CONST_INVERSE,state[1]); 
    InverterControl.Luenberger_obser.omg_elec           = state[1];
    InverterControl.Luenberger_obser.pseudo_load_torque = state[2];
    // printf("parameter convey\n");`

    while (InverterControl.Luenberger_obser.theta_d>PI ||InverterControl.Luenberger_obser.theta_d<-PI)
    {
        if( InverterControl.Luenberger_obser.theta_d>PI){
         InverterControl.Luenberger_obser.theta_d -= 2*PI;
        }else if( InverterControl.Luenberger_obser.theta_d<-PI){
            InverterControl.Luenberger_obser.theta_d += 2*PI;
        }
    }
   
}

void hfsi_do(pLuenberger v){
        // luenberger_filter() needs CTRL.Tem so hfsi_do() should executed after contorl().
        // If filtered currents are used instead of IS_C, move everything before luenberger_filter() to measurement(). <- Not suggested: lpf ruins current loop control.
        double IAB_LPF[2],IAB_HPF[2];
        double LAST_IAB_HPF[2];
        double DELTA_IAB_HPF[2];
        v->test_signal_al = PMSM.Ialpha;
        v->test_signal_be = PMSM.Ibeta;
        // printf("hfsi doing\n");
            // hfsi.theta_filter = sm.theta_d;
            v->theta_filter      = v->theta_d;
            v->CurPark_LPF.Alpha = v->test_signal_al;
            v->CurPark_LPF.Beta  = v->test_signal_be;
            v->CurPark_LPF.theta = v->theta_filter;
            calcParkReg(&v->CurPark_LPF);
            v->test_signal_d = v->CurPark_LPF.Ud;
            v->test_signal_q = v->CurPark_LPF.Uq;
            

            // LPF
            RK4_111_general(dynamics_lpf, v->test_signal_d, &v->d_lpf, CON_TS);
            RK4_111_general(dynamics_lpf, v->test_signal_q, &v->q_lpf, CON_TS);

            //反park变换
            v->InvCurPark_LPF.Ud = v->d_lpf;
            v->InvCurPark_LPF.Uq = v->q_lpf;
            v->InvCurPark_LPF.theta = v->theta_filter;
            calcI_ParkReg(&v->InvCurPark_LPF);
            IAB_LPF[0] = v->InvCurPark_LPF.Alpha;
            IAB_LPF[1] = v->InvCurPark_LPF.Beta ;
            

            // HPF
            
            LAST_IAB_HPF[0] = IAB_HPF[0];
            LAST_IAB_HPF[1] = IAB_HPF[1];
            v->d_hpf = v->test_signal_d - v->d_lpf;
            v->q_hpf = v->test_signal_q - v->q_lpf;

            //反park变换
            v->InvCurPark_HPF.Ud=v->d_hpf;
            v->InvCurPark_HPF.Uq=v->q_hpf;
            v->InvCurPark_HPF.theta = v->theta_filter;
            calcI_ParkReg(&v->InvCurPark_HPF);
            IAB_HPF[0] = v->InvCurPark_HPF.Alpha;
            IAB_HPF[1] = v->InvCurPark_HPF.Beta;

            //作差
            DELTA_IAB_HPF[0] = IAB_HPF[0] - LAST_IAB_HPF[0];
            DELTA_IAB_HPF[1] = IAB_HPF[1] - LAST_IAB_HPF[1];
         
            v->theta_d_raw = atan2(DELTA_IAB_HPF[1], DELTA_IAB_HPF[0]);
            if(v->theta_d_raw>PI){
                printf("%g", v->theta_d_raw/PI*180);
                v->theta_d_raw -= 2*PI;
            }else if(v->theta_d_raw<-PI){
                printf("%g", v->theta_d_raw/PI*180);
                v->theta_d_raw += 2*PI;
            }
        // IS_C(0) = IS_LPF(0); // The waveform of filtered currents looks good but it is still delayed and that is detrimental to control system stability.
        // IS_C(1) = IS_LPF(1); 

        luenberger_filter(InverterControl.Luenberger_obser.theta_d_raw);
    }
#else

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

    v->Ealpha = sat(v->Ialpha) * v->ksw;
    v->Ebeta  = sat(v->Ibeta)  * v->ksw;
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
}



#endif