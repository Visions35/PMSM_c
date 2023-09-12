
#include <math.h>
#include "ControlFunction.h"
#include "Observer.h"

void calcPidReg(pPID_REG v)
{
	v->Err = v->Ref - v->Fdb;

	v->PidKpOut = v->Kp * v->Err;

	v->PidKiOut += v->Ki * v->Err * v->Ts;

	v->PidKdOut = v->Kd * (v->Err-v->last_Err)/v->Ts;
	v->last_Err = v->Err;
	
	if (v->PidKiOut > v->LimitHigh)
	{
		v->PidKiOut = v->LimitHigh;
	}
	else if (v->PidKiOut < v->LimitLow)
	{
		v->PidKiOut = v->LimitLow;
	}


	v->PidOut = v->PidKpOut + v->PidKiOut + v->PidKdOut;

	if (v->PidOut > v->LimitHigh)
	{
		v->PidOut = v->LimitHigh;
	}
	else if (v->PidOut < v->LimitLow)
	{
		v->PidOut = v->LimitLow;
	}
}

void restartPidReg(pPID_REG v)
{
	v->PidKiOut = 0;
	v->PidKpOut = 0;
	v->PidKdOut = 0;
	v->PidOut = 0;
}


int sign(float v)
{
	float signal=0.0;
	if (v>0)
		return 1;
	else
	if (v=0)
	{
		return 0;
	}else
		return -1;
}

float sat(float v)
{
	float i;
	if (v>4)
	{
		return 1;
	}
	else if (v<=4 || v>=-4)
	{
		/* code */
		i= v/4;
		return i;
	}
	else
	{
		return -1;
	}
}


void calcSMCReg(pSMC_REG v)
{
	v->Err = (v->Ref - v->Fdb);
	v->sign_Err = sign(v->Err);
	v->fabs_Err = fabs(v->Err);
	v->sum1 = v->kp*sqrt(v->fabs_Err) * v->sign_Err;
	
	v->sum2 = v->last_sign + v->ki * v->sign_Err * v->Ts;

	v->SMC_output =v->sum1 + v->sum2;
	// if (v->SMC_output > v->limit_high)
	// {
	// 	v->SMC_output = v->limit_high;
	// }
	// else if (v->SMC_output < v->limit_low)
	// {
	// 	v->SMC_output = v->limit_low;
	// }
	v->last_sign = v->sum2;

}

void calcLPFFILTER(pLPF_FILTER v)
{
	float RC = 0;
	RC = 1.0 / (2.0 * PI * v->omega_c);
	v->b = RC /(RC + v->Ts);
	v->a = 1.0 - v->b; 
	v->filter_output = v->a * v->last_output + v->filter_input*v->b;
	v->last_output = v->filter_output;
}

void initTWOorderLPF(pTWO_orderLPF v)
{	
	float alpha = 0;
	float a0_inv = 0;
	v->Ts = CON_TS;
	v->omega = omegac;
	v->cutoff_freq = 2.0 * PI * v->omega * v->Ts ;
	
	alpha = sin( v->cutoff_freq )  / (2.0 * 0.7071 );
	a0_inv = 1.0 / ( 1.0 + alpha);

	v->b0 = (1.0 - cos(v->cutoff_freq)) * 0.5 * a0_inv;
	v->b1 = (1.0 - cos(v->cutoff_freq)) * a0_inv;
	v->b2 = (1.0 - cos(v->cutoff_freq)) * 0.5 * a0_inv;
	v->a1 = -2.0 * cos(v->cutoff_freq) * a0_inv;
	v->a2 = (1.0 - alpha) * a0_inv;

	v->x1 = 0;
	v->x2 = 0;
	v->out1 = 0;
	v->out2 = 0;
}

void calcTWOorderLPF(pTWO_orderLPF v)
{
	v->LPF_output = v->b0 * v->LPF_input + v->b1 * v->x1 + v->b2 * v->x2
			       -v->a1 * v->out1 - v->a2 * v->out2;
	v->x2 = v->x1;
	v->x1 = v->LPF_input;
	v->out2 = v->out1;
	v->out1 = v->LPF_output;
}

void restartSMCReg(pSMC_REG v)
{
	v->sum1 = 0;
	v->sum2 = 0;
	v->SMC_output;
}



void calcRmp(pRMP_CTR v)
{
	v->Err = v->Target - v->outputValue;

	if (v->Err > v->MinErr)
	{
		v->outputValue += v->Step;
	}
	else if (v->Err < (-1.0*v->MinErr))
	{
		v->outputValue -= v->Step;
	}
	else
	{
		v->outputValue = v->outputValue;
	}
}

void restartRmp(pRMP_CTR v)
{
	v->outputValue = 0;
}


void calcClarkReg(pCLARK_REG v)
{
	v->Alpha = 0.3333 * (2.0*v->Ua - v->Ub - v->Uc);
	v->Beta = (v->Ub - v->Uc)*0.57735026918963;
	v->Zero = 0.3333 * (v->Ua + v->Ub + v->Uc);
}

void calcI_ClarkReg(pI_CLARK_REG v)
{
	v->Ua = v->Alpha + v->Zero;   
	v->Ub = -0.5*v->Alpha + 0.8660254 * v->Beta + v->Zero;
	v->Uc = -0.5* v->Alpha - 0.8660254 * v->Beta + v->Zero;
}

void calcParkReg(pPARK_REG v)
{
	float Sinwt, Coswt;
	
	Sinwt = sin(v->theta);
	Coswt = cos(v->theta);
	
	v->Ud = v->Alpha * Coswt + v->Beta  * Sinwt;
	v->Uq = v->Beta  * Coswt - v->Alpha * Sinwt;
}

void calcI_ParkReg(pI_PARK_REG v)
{
	float Sinwt, Coswt;
	
	Sinwt = sin(v->theta);
	Coswt = cos(v->theta);
	
	v->Alpha = v->Ud*Coswt - v->Uq*Sinwt;
	v->Beta  = v->Ud*Sinwt + v->Uq*Coswt;
}