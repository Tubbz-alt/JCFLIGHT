#ifndef BIQUADFILTER_h
#define BIQUADFILTER_h
#include "Arduino.h"
enum FilterTypes
{
  LPF = 0,
  HPF,
  NOTCH
};
class BiQuadFilter
{
public:
  void Settings(int16_t CutOffFreq, int16_t SampleFreq, uint8_t FilterType);
  int16_t FilterOutput(int16_t DeviveToFilter);

private:
  //COEFICIENTES PARA O FILTRO
  int16_t Coeff1;
  int16_t Coeff2;
  int16_t Coeff3;
  int16_t Coeff4;
  int16_t Coeff5;

  //VARIAVEIS PARA GUARDAR VALORES LIDOS DO DISPOSITIVO A CADA CICLO DE MAQUINA
  int16_t GuardInput1 = 0;
  int16_t GuardInput2 = 0;
  int16_t GuardOutput1 = 0;
  int16_t GuardOutput2 = 0;

  //CONVERSÃO DE VALORES
  union TypeConverter
  {
    int32_t LongValue;
    int16_t ShortValue[2];
  } Result;
};
#endif
