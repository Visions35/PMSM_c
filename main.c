#include "PMSMSim.h"
#include "InverterController.h"
#include "Observer.h"
#include "ControlFunction.h"

PMSMSim PMSM;
extern INVERTER_CONTROLLER InverterControl;
void PMSM_init()
{
    PMSM.Ts = MACHINE_TS;
    int i;
    for ( i = 0; i < NUMBER_OF_STATES; i++)
    {
        PMSM.x[i] = 0.0;
        PMSM.dotx[i] = 0.0;
    }
    PMSM.rpm=0.0;
    PMSM.Id=0.0;
    PMSM.Iq=0.0;
    PMSM.Ud=0.0;
    PMSM.Uq=0.0;

    PMSM.Ualpha=0.0;
    PMSM.Ubeta =0.0;
    PMSM.Ialpha =0.0;
    PMSM.Ibeta =0.0;

    PMSM.Tload=0.0;
    PMSM.omegae = 0.0;
    PMSM.rpm_cmd = 0.0;
    PMSM.rpm_deriv_cmd =0.0;
    PMSM.omegae = 0.0;

    PMSM.F=0;
    PMSM.Ld=0.0052;
    PMSM.Lq=0.0174;
    PMSM.R =0.33;
    PMSM.np =2;
    PMSM.J  =0.008;
    PMSM.theta_d=0;

    PMSM.RinvLd=PMSM.R / PMSM.Ld;
    PMSM.RinvLq=PMSM.R / PMSM.Lq;


}

void RK_dynamics(double t, double *x, double *fx){//电机动态方程
    // electromagnetic model     fx代表微分量
    //x[0]=id,x[1]=iq,x[2]=omegae,x[3]=theta,fx[0]=pid,fx[1]=piq，fx[2]=
    fx[0] = -PMSM.RinvLd*x[0] + PMSM.Ud/PMSM.Ld + x[2]*PMSM.Lq*x[1]/PMSM.Ld;//p_iq
    fx[1] = -PMSM.RinvLq*x[1] + PMSM.Uq/PMSM.Lq - x[2]*(PMSM.Ld*x[0] - flux_f)/PMSM.Lq;//p_iq

    // mechanical model
    PMSM.Tem = 1.5*PMSM.np*((PMSM.Ld-PMSM.Lq)*x[0]*x[1]+x[1]*flux_f);
    fx[2] = (PMSM.Tem - PMSM.Tload-PMSM.F*x[2]/PMSM.np)/PMSM.J;    //  dw/dt
    fx[3] = x[2];//dtheta/dt=omegae
}


#if rk
    void RK_Linear(double t, double *x, double hs){
        #define NS NUMBER_OF_STATES

        double k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
        double fx[NS];//k1，k2,k3,k4表示斜率
        int i;
        //dy/dt=fx[i],斜率乘以步长，k2=h*f(x+h/2,y+k1/2),各个状态量对于时间的导数
        //每次x增加一个步长h
        RK_dynamics(t, x, fx); // timer.t,
        for(i=0;i<NS;++i){        
            k1[i] = fx[i] * hs;                  //k1=h*f(x,y)
            xk[i] = x[i] + k1[i]*0.5;              //y+k1/2
        }
        
        RK_dynamics(t, xk, fx); // timer.t+hs/2.,求取新的fx 
        for(i=0;i<NS;++i){        
            k2[i] = fx[i] * hs;                //k2=h* f(x+h/2,y+k1/2)
            xk[i] = x[i] + k2[i]*0.5;           //y+k2/2
        }
        
        RK_dynamics(t, xk, fx); // timer.t+hs/2., 求取新的fx
        for(i=0;i<NS;++i){        
            k3[i] = fx[i] * hs;              //k3=h*f(x+h/2，y+k2/2)
            xk[i] = x[i] + k3[i];           //y+k3
        }
        
        RK_dynamics(t, xk, fx); // timer.t+hs, 求取新的fx
        for(i=0;i<NS;++i){        
            k4[i] = fx[i] * hs;             //k4=h*(x+h,y+k3)
            x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0;

            // derivatives
            PMSM.dotx[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0 / hs; 
        }
        #undef NS
    }

#else

    void RK_Linear(double t, double *x, double hs){
        #define NS NUMBER_OF_STATES

        double k1[NS], k2[NS], k3[NS], k4[NS],k5[NS], k6[NS],k7[NS],k8[NS],xk[NS];
        double fx[NS];//k1，k2,k3,k4表示斜率
        int i;
        //dy/dt=fx[i],斜率乘以步长，k2=h*f(x+h/2,y+k1/2),各个状态量对于时间的导数
        //每次x增加一个步长h
        RK_dynamics(t, x, fx); // timer.t,
        for(i=0;i<NS;++i){        
            k1[i] = fx[i] * hs;                  //k1=h*f(x,y)
            xk[i] = x[i] + k1[i]/8;              //y+k1/8
        }
        
        RK_dynamics(t, xk, fx); // timer.t+hs/2.,求取新的fx 
        for(i=0;i<NS;++i){        
            k2[i] = fx[i] * hs;                //k2=h* f(x+h/2,y+k1/2)
            xk[i] = x[i] + k1[i]/4;           //y+k2/4
        }
        
        RK_dynamics(t, xk, fx); // timer.t+hs/2., 求取新的fx
        for(i=0;i<NS;++i){        
            k3[i] = fx[i] * hs;              //k3=h*f(x+h/2，y+k2/2)
            xk[i] = x[i] + k1[i]*0.5;           //y+k3
        }
        
        RK_dynamics(t, xk, fx); // timer.t+hs/2., 求取新的fx
        for(i=0;i<NS;++i){        
            k4[i] = fx[i] * hs;              //k3=h*f(x+h/2，y+k2/2)
            xk[i] = x[i] + k1[i]*3/4;           //y+k3
        }
        RK_dynamics(t, xk, fx); // timer.t+hs/2., 求取新的fx
        for(i=0;i<NS;++i){        
            k5[i] = fx[i] * hs;              //k3=h*f(x+h/2，y+k2/2)
            xk[i] = x[i] -3* k1[i]/2+2*k2[i];           //y+k3
        }

        RK_dynamics(t, xk, fx); // timer.t+hs/2., 求取新的fx
        for(i=0;i<NS;++i){        
            k6[i] = fx[i] * hs;              //k3=h*f(x+h/2，y+k2/2)
            xk[i] = x[i] +7* k1[i]/8-14*k2[i]/8+7*k3[i]/8+7*k4[i]/8-7*k5[i]/8+7*k6[i]/8;           //y+k3
        }


        RK_dynamics(t, xk, fx); // timer.t+hs/2., 求取新的fx
        for(i=0;i<NS;++i){        
            k7[i] = fx[i] * hs;              //k3=h*f(x+h/2，y+k2/2)
            xk[i] = x[i] +3* k1[i]/2-4*k2[i]+12*k3[i]-24*k4[i]+48*k5[i]-108*k6[i]+84*k7[i];           //y+k3
        }

        RK_dynamics(t, xk, fx); // timer.t+hs, 求取新的fx
        for(i=0;i<NS;++i){        
            k8[i] = fx[i] * hs;             //k4=h*(x+h,y+k3)
            x[i] = x[i] + (7*k1[i] + 194*k2[i]+32* k3[i] + 12*k4[i]+32*k5[i]+7*k6[i]+392*k7[i]+196*k8[i])/840;

            // derivatives
            PMSM.dotx[i] = (7*k1[i] +194*k2[i]+ 32* k3[i] + 12*k4[i]+32*k5[i]+7*k6[i]+392*k7[i]+196*k8[i])/840 / hs; 
        }
        #undef NS
    }

#endif

int machine_simulation()
{
    I_PARK_REG CurIpark;
    RK_Linear(InverterControl.timebase, PMSM.x, PMSM.Ts);
      PMSM.theta_d = PMSM.x[3];
    if(PMSM.theta_d > PI){
        PMSM.theta_d -= 2*PI;
    }else if(PMSM.theta_d < -PI){
        PMSM.theta_d += 2*PI; // 反转！
    }
    PMSM.x[3] = PMSM.theta_d;

    // currents
    PMSM.Id  = PMSM.x[0];
    PMSM.Iq  = PMSM.x[1];

    CurIpark.Ud = PMSM.Id;
    CurIpark.Uq = PMSM.Iq;
    CurIpark.theta = PMSM.theta_d;
    calcI_ParkReg(&CurIpark);
    PMSM.Ialpha = CurIpark.Alpha;
    PMSM.Ibeta  = CurIpark.Beta;

    // speed
    PMSM.omegae = PMSM.x[2];
    PMSM.rpm    = PMSM.x[2] * rad2rpm / PMSM.np;

    // extended emf
    PMSM.eemf_q  = (PMSM.Ld-PMSM.Lq) * (PMSM.omegae*PMSM.Id - PMSM.dotx[1]) + PMSM.omegae*flux_f;
    PMSM.eemf_al = PMSM.eemf_q * -sin(PMSM.theta_d);
    PMSM.eemf_be = PMSM.eemf_q *  cos(PMSM.theta_d);
    // ACM.theta_d__eemf = atan2(-ACM.eemf_al, ACM.eemf_be);
    PMSM.theta_d__eemf = atan2(-PMSM.eemf_al*sign(PMSM.omegae), PMSM.eemf_be*sign(PMSM.omegae));

    // detect bad simulation
    if(isNumber(PMSM.rpm)){
        return 0;
    }else{
        printf("PMSM.rpm is %g\n", PMSM.rpm);
        return 1;        
    }
}




int main(){
    if(SENSORLESS_CONTROL_HFSI==true){
        printf("Sensorless using HFSI.\n");
    }else{
        if(SENSORLESS_CONTROL==true){
            printf("Sensorless using observer.\n");
        }
    }
    printf("NUMBER_OF_STEPS: %d\n\n", NUMBER_OF_STEPS);

    /* Initialization */
    PMSM_init();
    initDspInterrupt();
    // ob_init();

    FILE *fw;
    fw = fopen(DATA_FILE_NAME, "w");
    printf("%s\n", DATA_FILE_NAME);
    write_header_to_file(fw);

    /* MAIN LOOP */
    clock_t begin, end;
    begin = clock();
    int _; // _ for the outer iteration
    int dfe_counter=0; // dfe_counter for down frequency execution
    for(_=0;_<NUMBER_OF_STEPS;++_){

        // rpm_cmd
        if (InverterControl.timebase>12)
        {
            InverterControl.Targetn  = 0;
        }else if(InverterControl.timebase>12){
            InverterControl.Targetn  = 50;}
        else if(InverterControl.timebase>6)
            InverterControl.Targetn  = -50;
       
        // }else if(InverterControl.timebase>6){
        //     InverterControl.Targetn  = -40;
        // }else if(InverterControl.timebase>3){
        //     InverterControl.Targetn  = -20;
        // }else{
        //     InverterControl.Targetn  = -10;
        // }

        /* Load Torque */
        // ACM.Tload = 0 * sign(ACM.rpm); // No-load test
        // ACM.Tload = ACM.Tem; // Blocked-rotor test
        PMSM.Tload = 2 * sign(PMSM.rpm); // speed-direction-dependent load
        // if (InverterControl.timebase >15)
        // {
        //     PMSM.Tload = 10* sign(PMSM.rpm);
        // }
        
        // ACM.Tload = 2 * ACM.rpm/20; // speed-dependent load

        /* Simulated ACM */
        if(machine_simulation()){ 
            printf("Break the loop.\n");
            break;
        }

        if(++dfe_counter == TS_UPSAMPLING_FREQ_EXE_INVERSE){
            dfe_counter = 0;
            // printf("doing\n");
            /* Time in DSP */
            InverterControl.timebase += CON_TS;

            // observation();

            write_data_to_file(fw);

            calcInverterControl(&InverterControl);

            hfsi_do(&InverterControl.Luenberger_obser);
            
        }

    }
    end = clock(); 
    printf("The simulation in C costs %g sec.\n", (double)(end - begin)/CLOCKS_PER_SEC);
    fclose(fw);

    /* Fade out */
    system("python ./ACMPlot.py"); 
    
    // getch();
    // system("pause");
    // system("exit");
    return 0; 
}

/* Utility */
void write_header_to_file(FILE *fw){
    // no space is allowed!
    fprintf(fw, "t(s),x0(id)[A],x1(iq)[A],x2(speed)[rad/s],x2(speed)[rpm],x3(position)[rad],id_cmd[V],id_fdb[V],n_cmd[rpm],n_fdb[rpm],iq_cmd[A],iq_fdb[A],|theta_d[rad],test_signal_al,test_signal_be,Ialpha_LPF_Euler,Ibeta_LPF_Euler,Ialpha_HPF_Euler,Ibeta_HPF_Euler,Id_hpf,Iq_hpf,HFSI_POS,HFSI_POS_ER,hfsi.theta_d,ER(theta_d),hfsi.omg_elec,ER(omg_elec),hfsi.TL,mismatch,err(theta_d)\n");
    {
        FILE *fw2;
        fw2 = fopen("info.dat", "w");
        fprintf(fw2, "TS,DOWN_SAMPLE,DATA_FILE_NAME\n");
        fprintf(fw2, "%g, %d, %s\n", CON_TS, DOWN_SAMPLE, DATA_FILE_NAME);
        fclose(fw2);
    }
}
void write_data_to_file(FILE *fw){
    static int bool_animate_on = false;
    static int j=0,jj=0; // j,jj for down sampling

    // if(CTRL.timebase>20)
    {
        if(++j == DOWN_SAMPLE)
        {
            j=0;
            fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
                    InverterControl.timebase,
                    PMSM.x[0], PMSM.x[1], PMSM.x[2], PMSM.x[2]*rad2rpm/PMSM.np,PMSM.x[3], 
                    InverterControl.Pid_id.Ref, InverterControl.Pid_id.Fdb, 
                    InverterControl.Pidn.Ref  , InverterControl.Pidn.Fdb, 
                    InverterControl.Pid_iq.Ref,InverterControl.Pid_iq.Fdb,
                    PMSM.theta_d, //ACM.theta_d__eemf,ACM.theta_d-ACM.theta_d__eemf,sin(ACM.theta_d-ACM.theta_d__eemf),
                    InverterControl.Luenberger_obser.test_signal_al,
                    InverterControl.Luenberger_obser.test_signal_be,
                    InverterControl.Luenberger_obser.InvCurPark_LPF.Alpha,
                    InverterControl.Luenberger_obser.InvCurPark_LPF.Beta,
                    InverterControl.Luenberger_obser.InvCurPark_HPF.Alpha,
                    InverterControl.Luenberger_obser.InvCurPark_HPF.Beta,
                    InverterControl.Luenberger_obser.d_hpf, 
                    InverterControl.Luenberger_obser.q_hpf, 
                    InverterControl.Luenberger_obser.theta_d_raw, 
                    sin(PMSM.theta_d-InverterControl.Luenberger_obser.theta_d_raw),
                    InverterControl.Luenberger_obser.theta_d,
                    sin(PMSM.theta_d-InverterControl.Luenberger_obser.theta_d), 
                    InverterControl.Luenberger_obser.omg_elec, 
                    PMSM.omegae-InverterControl.Luenberger_obser.omg_elec, 
                    InverterControl.Luenberger_obser.pseudo_load_torque*PMSM.J/PMSM.np, 
                    InverterControl.Luenberger_obser.mismatch,
                    PMSM.theta_d-InverterControl.Luenberger_obser.theta_d
                    // InverterControl.Luenberger_obser.test_signal_al-InverterControl.Luenberger_obser.InvCurPark_LPF.Alpha-InverterControl.Luenberger_obser.InvCurPark_HPF.Alpha,
                    // InverterControl.Luenberger_obser.test_signal_be-InverterControl.Luenberger_obser.InvCurPark_LPF.Beta-InverterControl.Luenberger_obser.InvCurPark_HPF.Beta
                    );
        }
    }

    // if(bool_animate_on==false){
    //     bool_animate_on = true;
    //     printf("Start ACMAnimate\n");
    //     system("start python ./ACMAnimate.py"); 
    // }
}

int isNumber(double x){
    // This looks like it should always be true, 
    // but it's false if x is an NaN (1.#QNAN0).
    return (x == x); 
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}

