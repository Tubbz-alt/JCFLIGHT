#ifndef SCHEDULER_H_
#define SCHEDULER_H_
#include "Arduino.h"
typedef struct
{
  uint32_t ActualTime;
  uint32_t StoredTime;
} Scheduler_Struct;
bool SchedulerTimer(Scheduler_Struct *SchedulerPointer, uint32_t RefreshTime);
#endif
