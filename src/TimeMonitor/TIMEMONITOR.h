#ifndef TIMEMONITOR_h
#define TIMEMONITOR_h
#include "Arduino.h"
//#define ENABLE_TIMEMONITOR
enum Function_Names
{
  SLOW_LOOP = 0,
  MEDIUM_LOOP,
  FAST_MEDIUM_LOOP,
  FAST_LOOP,
  TOTAL_LOOP,
  SIZE_LOOPS
};
class AVRTimeMonitor
{
public:
  void MeasuringStartTime(uint8_t FunctionNumber);
  void MeasuringFinishTime();

private:
  float AVRTotalTime = 0;
  uint32_t StartTime = 0;
  uint32_t EndTime = 0;
};
extern AVRTimeMonitor AVRTIMEMONITOR;
#endif
