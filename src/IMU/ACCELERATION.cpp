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

#include "ACCELERATION.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"

float Cosine_Yaw;
float Sine_Yaw;
float AccelerationAdjustBias[3] = {0.0, 0.0, 0.0};
float AccelerationEarthFrame_LPF[3] = {0.0, 0.0, 0.0};
float AccelerationDifference[3] = {0.0, 0.0, 0.0};

void CalculateAccelerationXYZ()
{
  float Cosine_Roll;
  float Sine_Roll;
  float Cosine_Pitch;
  float Sine_Pitch;
  float SinePitch_CosineYaw_Fusion;
  float SinePitch_SineYaw_Fusion;

  //CALCULA O SENO E COSSENO DE CADA EIXO DA ESTIMAÇÃO DE ATTITUDE DADO PELO DCM
  Cosine_Roll = Calculate_Cosine_Approx(ATTITUDE.AngleOut[ROLL]);
  Sine_Roll = Calculate_Sine_Approx(ATTITUDE.AngleOut[ROLL]);
  Cosine_Pitch = Calculate_Cosine_Approx(-ATTITUDE.AngleOut[PITCH]);
  Sine_Pitch = Calculate_Sine_Approx(-ATTITUDE.AngleOut[PITCH]);
  Cosine_Yaw = Calculate_Cosine_Approx(ATTITUDE.CompassHeading);
  Sine_Yaw = Calculate_Sine_Approx(ATTITUDE.CompassHeading);

  //REALIZA A FUSÃO DE ALGUMAS VARIAVEIS PARA OBTER DIFERENTES VALORES DE ACELERAÇÃO EM TORNO DO EIXO Z
  SinePitch_CosineYaw_Fusion = Sine_Pitch * Cosine_Yaw;
  SinePitch_SineYaw_Fusion = Sine_Pitch * Sine_Yaw;

  //ROLL
  INS.AccelerationEarthFrame[0] = -((Cosine_Pitch * Cosine_Yaw) * IMU.AccelerometerRead[PITCH] + (Sine_Roll * SinePitch_CosineYaw_Fusion - Cosine_Roll * Sine_Yaw) * IMU.AccelerometerRead[ROLL] + (Sine_Roll * Sine_Yaw + Cosine_Roll * SinePitch_CosineYaw_Fusion) * IMU.AccelerometerRead[YAW]);

  //PITCH
  INS.AccelerationEarthFrame[1] = -((Cosine_Pitch * Sine_Yaw) * IMU.AccelerometerRead[PITCH] + (Cosine_Roll * Cosine_Yaw + Sine_Roll * SinePitch_SineYaw_Fusion) * IMU.AccelerometerRead[ROLL] + (-Sine_Roll * Cosine_Yaw + Cosine_Roll * SinePitch_SineYaw_Fusion) * IMU.AccelerometerRead[YAW]);

  //YAW
  INS.AccelerationEarthFrame[2] = ((-Sine_Pitch) * IMU.AccelerometerRead[PITCH] + (Sine_Roll * Cosine_Pitch) * IMU.AccelerometerRead[ROLL] + (Cosine_Roll * Cosine_Pitch) * IMU.AccelerometerRead[YAW]) - 512;

  //ROLL
  INS.AccelerationEarthFrame[0] = INS.AccelerationEarthFrame[0] * 1.915361328125f;
  AccelerationDifference[0] = INS.AccelerationEarthFrame[0] - AccelerationAdjustBias[0];
  if (!COMMAND_ARM_DISARM)
  {
    AccelerationAdjustBias[0] = AccelerationAdjustBias[0] * 0.985f + INS.AccelerationEarthFrame[0] * 0.015f;
  }
  else if (ABS_FLOAT(AccelerationDifference[0]) <= 80.0f)
  {
    AccelerationAdjustBias[0] = AccelerationAdjustBias[0] * 0.9987f + INS.AccelerationEarthFrame[0] * 0.0013f;
  }
  INS.AccelerationEarthFrame[0] = AccelerationDifference[0];
  AccelerationEarthFrame_LPF[0] = AccelerationEarthFrame_LPF[0] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[0] * 0.14285714285714285714285714285714f;
  INS.AccelerationEarthFrame_Sum[0] += AccelerationEarthFrame_LPF[0];
  INS.AccelerationEarthFrame_Sum_Count[0]++;

  //PITCH
  INS.AccelerationEarthFrame[1] = INS.AccelerationEarthFrame[1] * 1.915361328125f;
  AccelerationDifference[1] = INS.AccelerationEarthFrame[1] - AccelerationAdjustBias[1];
  if (!COMMAND_ARM_DISARM)
  {
    AccelerationAdjustBias[1] = AccelerationAdjustBias[1] * 0.985f + INS.AccelerationEarthFrame[1] * 0.015f;
  }
  else if (ABS_FLOAT(AccelerationDifference[1]) <= 80.0f)
  {
    AccelerationAdjustBias[1] = AccelerationAdjustBias[1] * 0.9987f + INS.AccelerationEarthFrame[1] * 0.0013f;
  }
  INS.AccelerationEarthFrame[1] = AccelerationDifference[1];
  AccelerationEarthFrame_LPF[1] = AccelerationEarthFrame_LPF[1] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[1] * 0.14285714285714285714285714285714f;
  INS.AccelerationEarthFrame_Sum[1] += AccelerationEarthFrame_LPF[1];
  INS.AccelerationEarthFrame_Sum_Count[1]++;

  //YAW
  INS.AccelerationEarthFrame[2] = INS.AccelerationEarthFrame[2] * 1.915361328125f;
  AccelerationDifference[2] = INS.AccelerationEarthFrame[2] - AccelerationAdjustBias[2];
  if (!COMMAND_ARM_DISARM)
  {
    AccelerationAdjustBias[2] = AccelerationAdjustBias[2] * 0.985f + INS.AccelerationEarthFrame[2] * 0.015f;
  }
  else if (ABS_FLOAT(AccelerationDifference[2]) <= 80.0f)
  {
    AccelerationAdjustBias[2] = AccelerationAdjustBias[2] * 0.9987f + INS.AccelerationEarthFrame[2] * 0.0013f;
  }
  INS.AccelerationEarthFrame[2] = AccelerationDifference[2];
  AccelerationEarthFrame_LPF[2] = AccelerationEarthFrame_LPF[2] * 0.85714285714285714285714285714286f + INS.AccelerationEarthFrame[2] * 0.14285714285714285714285714285714f;
  INS.AccelerationEarthFrame_Sum[2] += AccelerationEarthFrame_LPF[2];
  INS.AccelerationEarthFrame_Sum_Count[2]++;
}
