#include "TIMEMONITOR.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "FunctionsLoop/LOOPS.h"
#include "FunctionsName/FUNCTIONSNAME.h"
#include "FastSerial/PRINTF.h"

AVRTimeMonitor AVRTIMEMONITOR;

bool SafeToPrint = false;
uint16_t FunctionNumberCount = 0;

void AVRTimeMonitor::MeasuringStartTime(uint8_t FunctionNumber)
{
  UpdateFunctionName(FunctionNumber);
  if (FunctionNumber == FunctionNumberCount)
  {
    FunctionNumberCount++;
    SafeToPrint = true;
  }
  if (FunctionNumberCount >= SIZE_LOOPS)
  {
    FunctionNumberCount++;
    if (FunctionNumberCount >= SIZE_LOOPS + 500)
    {
      FastSerialPrintln(PSTR("\n"));
      FunctionNumberCount = 0;
    }
  }
  StartTime = AVRTIME.SchedulerMicros();
  EndTime = 0;
}

void AVRTimeMonitor::MeasuringFinishTime()
{
  if (EndTime == 0)
    EndTime = AVRTIME.SchedulerMicros();
  AVRTotalTime = EndTime - StartTime;
  if (SafeToPrint)
  {
    FastSerialPrintln(PSTR("%-20s\tTempo Gasto:%.5f us\n"),
                      GetFunctionName,
                      AVRTotalTime / 1000);
    SafeToPrint = false;
  }
}