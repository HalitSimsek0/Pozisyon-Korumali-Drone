#include "pid.h"
#include "num.h"
#include <math.h>
#include <float.h>
void pidInit(PidObject* pid, const float desired, const float kp,
             const float ki, const float kd, const float dt,
             const float samplingRate, const float cutoffFreq,
             bool enableDFilter)
{
  pid->error         = 0;
  pid->prevError     = 0;
  pid->integ         = 0;
  pid->deriv         = 0;
  pid->desired       = desired;
  pid->kp            = kp;
  pid->ki            = ki;
  pid->kd            = kd;
  pid->kf            = 0.0f;  
  pid->outF          = 0.0f;
  pid->iLimit        = DEFAULT_PID_INTEGRATION_LIMIT;
  pid->outputLimit   = DEFAULT_PID_OUTPUT_LIMIT;
  pid->dt            = dt;
  pid->rate          = (dt > 0.0f) ? (1.0f / dt) : 1.0f;
  pid->pScale        = 1.0f;
  pid->iScale        = 1.0f;
  pid->dScale        = 1.0f;
  pid->fScale        = 1.0f;
  pid->itermRelax    = ITERM_RELAX_OFF;
  pid->itermRelaxFactor = 1.0f;
  pid->itermRelaxBase = 0.0f;
  pid->iTermError    = 0.0f;
  pid->prevMeasurement = 0.0f;
  pid->prevSetpoint    = desired;
  pid->outputSaturated = false;
  pid->enableDFilter = enableDFilter;
  pid->enablePFilter = false;
  pid->enableFFilter = false;
  if (pid->enableDFilter && cutoffFreq > 0.0f)
  {
    lpf2pInit(&pid->dFilter, samplingRate, cutoffFreq);
  }
}
float pidUpdate(PidObject* pid, const float measured, const bool updateError)
{
    float output = 0.0f;
    if (updateError)
    {
        pid->error = pid->desired - measured;
    }
    pid->outP = pid->kp * pid->error * pid->pScale;
    if (pid->enablePFilter)
    {
        pid->outP = lpf2pApply(&pid->ptermFilter, pid->outP);
    }
    output += pid->outP;
    pid->iTermError = pid->error;
    if (pid->ki > 0.0f && pid->iScale > 0.0f)
    {
        if (!pid->outputSaturated)
        {
            if (pid->itermRelax != ITERM_RELAX_OFF)
            {
                const bool increasing = (pid->integ > 0.0f && pid->iTermError > 0.0f) || 
                                        (pid->integ < 0.0f && pid->iTermError < 0.0f);
                const bool incrementOnly = (pid->itermRelax == ITERM_RELAX_RP_INC) || 
                                          (pid->itermRelax == ITERM_RELAX_RPY_INC);
                pid->itermRelaxBase = pid->desired - lpf2pApply(&pid->itermRelaxFilter, pid->desired);
                float itermRelaxBaseDeg = pid->itermRelaxBase * 57.2957795f; 
                pid->itermRelaxFactor = fmaxf(0.0f, 1.0f - fabsf(itermRelaxBaseDeg) * 0.025f);
                if (!incrementOnly || increasing)
                {
                    pid->iTermError *= pid->itermRelaxFactor;
                }
            }
            pid->integ += pid->ki * pid->iScale * pid->iTermError * pid->dt;
            if (pid->iLimit != 0.0f)
            {
                pid->integ = constrain(pid->integ, -pid->iLimit, pid->iLimit);
            }
        }
    }
    else
    {
        pid->integ = 0.0f; 
    }
    pid->outI = pid->ki * pid->integ;
    output += pid->outI;
    if (pid->kd > 0.0f && pid->dScale > 0.0f)
    {
        float dTerm = pid->kd * pid->dScale * ((pid->prevMeasurement - measured) * pid->rate);
        if (pid->enableDFilter)
        {
            dTerm = lpf2pApply(&pid->dtermNotchFilter, dTerm);
            dTerm = lpf2pApply(&pid->dFilter, dTerm);
            dTerm = lpf2pApply(&pid->dFilter2, dTerm);
        }
        if (isnan(dTerm) || !isfinite(dTerm))
        {
            dTerm = 0.0f;
        }
        pid->deriv = dTerm;
        pid->outD = dTerm;
        output += pid->outD;
    }
    else
    {
        pid->deriv = 0.0f;
        pid->outD = 0.0f;
    }
    if (pid->kf > 0.0f && pid->fScale > 0.0f)
    {
        pid->outF = pid->kf * pid->fScale * (pid->desired - pid->prevSetpoint) * pid->rate;
        if (pid->enableFFilter)
        {
            pid->outF = lpf2pApply(&pid->ftermFilter, pid->outF);
        }
        output += pid->outF;
    }
    else
    {
        pid->outF = 0.0f;
    }
    if (pid->outputLimit != 0.0f)
    {
        output = constrain(output, -pid->outputLimit, pid->outputLimit);
    }
    pid->prevError = pid->error;
    pid->prevMeasurement = measured;
    pid->prevSetpoint = pid->desired;
    return output;
}
void pidSetIntegralLimit(PidObject* pid, const float limit) {
    pid->iLimit = limit;
}
void pidReset(PidObject* pid)
{
  pid->error     = 0;
  pid->prevError = 0;
  pid->integ     = 0;
  pid->deriv     = 0;
}
void pidSetError(PidObject* pid, const float error)
{
  pid->error = error;
}
void pidSetDesired(PidObject* pid, const float desired)
{
  pid->desired = desired;
}
float pidGetDesired(PidObject* pid)
{
  return pid->desired;
}
bool pidIsActive(PidObject* pid)
{
  bool isActive = true;
  if (pid->kp < 0.0001f && pid->ki < 0.0001f && pid->kd < 0.0001f)
  {
    isActive = false;
  }
  return isActive;
}
void pidSetKp(PidObject* pid, const float kp)
{
  pid->kp = kp;
}
void pidSetKi(PidObject* pid, const float ki)
{
  pid->ki = ki;
}
void pidSetKd(PidObject* pid, const float kd)
{
  pid->kd = kd;
}
void pidSetDt(PidObject* pid, const float dt) {
    pid->dt = dt;
    pid->rate = (dt > 0.0f) ? (1.0f / dt) : 1.0f;
}
void pidSetKf(PidObject* pid, const float kf) {
    pid->kf = kf;
}
void pidSetItermRelax(PidObject* pid, ItermRelaxType relaxType) {
    pid->itermRelax = relaxType;
}
void pidSetOutputSaturated(PidObject* pid, bool saturated) {
    pid->outputSaturated = saturated;
}
void pidSetScales(PidObject* pid, float pScale, float iScale, float dScale, float fScale) {
    pid->pScale = pScale;
    pid->iScale = iScale;
    pid->dScale = dScale;
    pid->fScale = fScale;
}
void pidInitFilters(PidObject* pid, float samplingRate, float dtermCutoff, float ptermCutoff, float ftermCutoff) {
    if (dtermCutoff > 0.0f && samplingRate > 0.0f)
    {
        lpf2pInit(&pid->dFilter, samplingRate, dtermCutoff);
        lpf2pInit(&pid->dFilter2, samplingRate, dtermCutoff);
        lpf2pInit(&pid->dtermNotchFilter, samplingRate, dtermCutoff * 2.0f);
        pid->enableDFilter = true;
    }
    if (ptermCutoff > 0.0f && samplingRate > 0.0f)
    {
        lpf2pInit(&pid->ptermFilter, samplingRate, ptermCutoff);
        pid->enablePFilter = true;
    }
    if (ftermCutoff > 0.0f && samplingRate > 0.0f)
    {
        lpf2pInit(&pid->ftermFilter, samplingRate, ftermCutoff);
        pid->enableFFilter = true;
    }
    if (samplingRate > 0.0f)
    {
        lpf2pInit(&pid->itermRelaxFilter, samplingRate, 10.0f);
    }
}