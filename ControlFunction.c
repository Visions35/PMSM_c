
#include <math.h>
#include "ControlFunction.h"

void calcPidReg(pPID_REG v)
{
	v->Err = v->Ref - v->Fdb;

	v->PidKpOut = v->Kp * v->Err;

	v->PidKiOut += v->Ki * v->Err * v->Ts;

	if (v->PidKiOut > v->LimitHigh)
	{
		v->PidKiOut = v->LimitHigh;
	}
	else if (v->PidKiOut < v->LimitLow)
	{
		v->PidKiOut = v->LimitLow;
	}

	v->PidOut = v->PidKpOut + v->PidKiOut;

	if (v->PidOut > v->LimitHigh)
	{
		v->PidOut = v->LimitHigh;
	}
	else if (v->PidOut < v->LimitLow)
	{
		v->PidOut = v->LimitLow;
	}
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
void calcSMCReg(pSMC_REG v)
{
	v->Err = rpm2rad * (v->Ref - v->Fdb);
	v->current_Err=v->Err;
	
	v->deri_out=(v->current_Err - v->last_Err)/v->Ts;

	v->sum1 = v->current_Err * v->c + v->deri_out;
	v->sum2 = v->sum1 * v->q + sign(v->deri_out) * v->mu + v->c * v->deri_out;
	v->SMC_output =(v->last_output + v->sum2 * v->Ts) /1.5/Pn/flux*0.003;
	if (v->SMC_output > v->limit_high)
	{
		v->SMC_output = v->limit_high;
	}
	else if (v->SMC_output < v->limit_low)
	{
		v->SMC_output = v->limit_low;
	}
	v->last_Err = v->current_Err;
	v->last_output = v->SMC_output;

}

void restartSMCReg(pSMC_REG v)
{
	v->deri_out = 0;
	v->sum1 = 0;
	v->sum2 = 0;
	v->SMC_output;
}

void restartPidReg(pPID_REG v)
{
	v->PidKiOut = 0;
	v->PidKpOut = 0;
	v->PidOut = 0;
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