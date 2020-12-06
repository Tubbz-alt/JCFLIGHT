#ifndef LPFACCEF
#define LPFACCEF
#include "Arduino.h"
class LowPassFilter
{
public:
  void Apply(const float Sample, float CutOff_Freq, float DeltaTime);
  float GetOutputFiltered();

private:
  float OutputFiltered;
  float Alpha = 1.0f;
};
#endif
