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

#include "IMUCALGCS.h"
#include "Common/STRUCTS.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"

bool AccCalibratedPosition[6];

int8_t GetAxisInclinedToCalibration(int16_t AccSample[3])
{
  if ((ABS_16BITS(AccSample[ROLL]) / 1.5f) > ABS_16BITS(AccSample[PITCH]) &&
      (ABS_16BITS(AccSample[ROLL]) / 1.5f) > ABS_16BITS(AccSample[YAW]))
  {
    //ROLL
    return (AccSample[ROLL] > 0) ? 2 : 3;
  }
  else if ((ABS_16BITS(AccSample[PITCH]) / 1.5f) > ABS_16BITS(AccSample[ROLL]) &&
           (ABS_16BITS(AccSample[PITCH]) / 1.5f) > ABS_16BITS(AccSample[YAW]))
  {
    //PITCH
    return (AccSample[PITCH] > 0) ? 4 : 5;
  }
  else if ((ABS_16BITS(AccSample[YAW]) / 1.5f) > ABS_16BITS(AccSample[ROLL]) &&
           (ABS_16BITS(AccSample[YAW]) / 1.5f) > ABS_16BITS(AccSample[PITCH]))
  {
    //YAW
    return (AccSample[YAW] > 0) ? 0 : 1;
  }
  else
    return -1;
}

uint8_t GetImageToGCS(void)
{
  static const uint8_t ImageBitMap[6] = {0, 1, 2, 3, 4, 5};
  uint8_t FlagCheck = 0;

  if (CALIBRATION.AccelerometerCalibration[ROLL] != 0 &&
      CALIBRATION.AccelerometerCalibration[PITCH] != 0 &&
      CALIBRATION.AccelerometerCalibration[YAW] != 0 &&
      CALIBRATION.AccelerometerCalibrationScale[ROLL] != 0 &&
      CALIBRATION.AccelerometerCalibrationScale[PITCH] != 0 &&
      CALIBRATION.AccelerometerCalibrationScale[YAW] != 0)
    return 0x3F;

  if (AccCalibratedPosition[0])
    FlagCheck |= (1 << ImageBitMap[0]);

  if (AccCalibratedPosition[1])
    FlagCheck |= (1 << ImageBitMap[1]);

  if (AccCalibratedPosition[2])
    FlagCheck |= (1 << ImageBitMap[2]);

  if (AccCalibratedPosition[3])
    FlagCheck |= (1 << ImageBitMap[3]);

  if (AccCalibratedPosition[4])
    FlagCheck |= (1 << ImageBitMap[4]);

  if (AccCalibratedPosition[5])
    FlagCheck |= (1 << ImageBitMap[5]);

  return FlagCheck;
}