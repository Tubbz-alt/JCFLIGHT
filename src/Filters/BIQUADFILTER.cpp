#include "BIQUADFILTER.h"
#include "Math/AVRLOWER.h"
#include "Math/AVRMATH.h"

void BiQuadFilter::Settings(int16_t CutOffFreq, int16_t SampleFreq, uint8_t FilterType)
{
  float COF = MIN_FLOAT(float(CutOffFreq), float((SampleFreq / 2) - 1)); //CALCULA A FREQUENCIA DE CORTE
  //GERA OS COEFICIENTES DO FILTRO BIQUADRATICO
  float Omega = tan(3.1415927f * COF / SampleFreq);
  float Normal = 1.0f / (1.0f + Omega / 0.7071f + Omega * Omega);
  switch (FilterType)
  {

  case LPF:
    //CALCULO DE COEFICIENTE PARA FILTRO DO TIPO LOW PASS
    Coeff1 = int16_t(Omega * Omega * Normal * 16384.0f);
    Coeff2 = 2 * Coeff1;
    Coeff3 = Coeff1;
    Coeff4 = int16_t(2.0f * (Omega * Omega - 1.0f) * Normal * 16384.0f);
    Coeff5 = int16_t((1.0f - Omega / 0.7071f + Omega * Omega) * Normal * 16384.0f);
    break;

  case HPF:
    //CALCULO DE COEFICIENTE PARA FILTRO DO TIPO HIGH PASS
    Coeff1 = 1 * Normal * 16384.0f;
    Coeff2 = -2 * Coeff1;
    Coeff3 = Coeff1;
    Coeff4 = 2 * (Omega * Omega - 1) * Normal * 16384.0f;
    Coeff5 = (1 - Omega / 0.7071f + Omega * Omega) * Normal * 16384.0f;
    break;

  case NOTCH:
    //CALCULO DE COEFICIENTE PARA FILTRO DO TIPO NOTCH
    Coeff1 = (1 + Omega * Omega) * Normal * 16384.0f;
    Coeff2 = 2 * (Omega * Omega - 1) * Normal * 16384.0f;
    Coeff3 = Coeff1;
    Coeff4 = Coeff2;
    Coeff5 = (1 - Omega / 0.7071f + Omega * Omega) * Normal * 16384.0f;
    break;
  }
}

int16_t BiQuadFilter::FilterOutput(int16_t DeviveToFilter)
{
  //ENTRADA DO DISPOSITIVO PARA O FILTRO
  //UTILIZA A MULTIPLICAÇÃO AVR EM ASSEMBLY PARA PREVENIR ERROS DE ARREDONDAMENTO DE VALORES
  Result.LongValue = Multiplication32Bits(DeviveToFilter, Coeff1) +
                     Multiplication32Bits(GuardInput1, Coeff2) +
                     Multiplication32Bits(GuardInput2, Coeff3) -
                     Multiplication32Bits(GuardOutput1, Coeff4) -
                     Multiplication32Bits(GuardOutput2, Coeff5);
  Result.LongValue = Result.LongValue << 2;
  GuardInput2 = GuardInput1;
  GuardInput1 = DeviveToFilter;
  GuardOutput2 = GuardOutput1;
  GuardOutput1 = Result.ShortValue[1];
  return Result.ShortValue[1];
}
