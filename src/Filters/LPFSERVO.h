/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#ifndef LPFSERVO_H_
#define LPFSERVO_H_
#include "Arduino.h"
typedef struct Struct_LowPassFilter
{
  bool Init_Filter;
  int16_t CutOffFreq;
  float BFilter[3];
  float AFilter[3];
  int64_t B_64_Bits[3];
  int64_t A_64_Bits[3];
  int16_t Coefficient_Bit_Shift;
  int16_t Input_Bit_Shift;
  int32_t Input_Bias;
  int32_t X_32Bits[3];
  int32_t Y_32Bits[3];
} Struct_LowPassFilter;
void LowPassFilterCoefficient(int16_t CutOffFreq, Struct_LowPassFilter *Filter, int16_t InputBias);
int32_t LowPassFilter(Struct_LowPassFilter *Filter, int32_t Input_Device, int16_t CutOffFreq, int16_t InputBiasSet);
#endif
