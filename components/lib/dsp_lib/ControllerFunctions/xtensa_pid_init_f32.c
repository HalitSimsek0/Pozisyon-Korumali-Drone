#include "xtensa_math.h"
void xtensa_pid_init_f32(
  xtensa_pid_instance_f32 * S,
  int32_t resetStateFlag)
{
  S->A0 = S->Kp + S->Ki + S->Kd;
  S->A1 = (-S->Kp) - ((float32_t) 2.0 * S->Kd);
  S->A2 = S->Kd;
  if (resetStateFlag)
  {
    memset(S->state, 0, 3U * sizeof(float32_t));
  }
}