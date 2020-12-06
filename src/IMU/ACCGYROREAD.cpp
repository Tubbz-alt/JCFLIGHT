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

#include "ACCGYROREAD.h"
#include "I2C/I2C.h"
#include "Common/VARIABLES.h"
#include "IMUCALIBRATE.h"
#include "Filters/KALMANFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Math/AVRMATH.h"
#include "Filters/PT1.h"
#include "Compass/COMPASSREAD.h"

#define ACTUAL_MACHINE_CYCLE 2000 //2uS - 500HZ

static FilterApplyFnPTR AccelerometerLPFApplyFn;
static FilterApplyFnPTR GyroscopeLPFApplyFn;
static PT1Filter_Struct AccelerometerLPFState[3];
static PT1Filter_Struct GyroscopeLPFState[3];

bool ActiveKalman = false;
uint8_t GyroLPF = 0;
int16_t Acc_LPF = 0;
int16_t Gyro_LPF = 0;
int16_t Acc_Notch = 0;
int16_t Gyro_Notch = 0;
int16_t Acc_LPFStoredInEEPROM = 0;
int16_t Gyro_LPFStoredInEEPROM = 0;
int16_t Acc_NotchStoredInEEPROM = 0;
int16_t Gyro_NotchStoredInEEPROM = 0;
int16_t AccelerometerToFilter[3];
int16_t GyroscopeToFilter[3];

static void AccFilterLPF_Initialization(FilterApplyFnPTR *ApplyFn, PT1Filter_Struct State[], uint16_t CutOff)
{
  *ApplyFn = (FilterApplyFnPTR)PT1FilterApply2;
  PT1FilterInit(&State[ROLL].PT1, CutOff, ACTUAL_MACHINE_CYCLE * 1e-6f);
  PT1FilterInit(&State[PITCH].PT1, CutOff, ACTUAL_MACHINE_CYCLE * 1e-6f);
  PT1FilterInit(&State[YAW].PT1, CutOff, ACTUAL_MACHINE_CYCLE * 1e-6f);
}

static void GyroFilterLPF_Initialization(FilterApplyFnPTR *ApplyFn, PT1Filter_Struct State[], uint16_t CutOff)
{
  *ApplyFn = (FilterApplyFnPTR)PT1FilterApply2;
  PT1FilterInit(&State[ROLL].PT1, CutOff, ACTUAL_MACHINE_CYCLE * 1e-6f);
  PT1FilterInit(&State[PITCH].PT1, CutOff, ACTUAL_MACHINE_CYCLE * 1e-6f);
  PT1FilterInit(&State[YAW].PT1, CutOff, ACTUAL_MACHINE_CYCLE * 1e-6f);
}

void IMU_Filters_Initialization()
{
  //CARREGA O VALOR GUARDADO DO ESTADO DO KALMAN
  if (STORAGEMANAGER.Read_8Bits(821) == 0)
    ActiveKalman = false;
  else
    ActiveKalman = true;
  //CARREGA OS VALORES GUARDADOS DO LPF
  Acc_LPF = STORAGEMANAGER.Read_16Bits(822);
  Gyro_LPF = STORAGEMANAGER.Read_16Bits(824);
  //CARREGA OS VALORES GUARDADOS DO NOTCH
  Acc_Notch = STORAGEMANAGER.Read_16Bits(826);
  Gyro_Notch = STORAGEMANAGER.Read_16Bits(828);
  //GERA UM COEFICIENTE PARA O LPF DO ACELEROMETRO
  AccFilterLPF_Initialization(&AccelerometerLPFApplyFn, AccelerometerLPFState, Acc_LPF);
  //GERA UM COEFICIENTE PARA O LPF DO GYROSCOPIO
  GyroFilterLPF_Initialization(&GyroscopeLPFApplyFn, GyroscopeLPFState, Gyro_LPF);
}

void IMU_Filters_Update()
{
  //ATUALIZA O VALOR GUARDADO DO ESTADO DO KALMAN
  if (STORAGEMANAGER.Read_8Bits(821) == 0)
    ActiveKalman = false;
  else
    ActiveKalman = true;
  //ATUALIZA OS VALORES GUARDADOS DO LPF
  Acc_LPFStoredInEEPROM = STORAGEMANAGER.Read_16Bits(822);
  Gyro_LPFStoredInEEPROM = STORAGEMANAGER.Read_16Bits(824);
  //ATUALIZA OS VALORES GUARDADOS DO NOTCH
  Acc_NotchStoredInEEPROM = STORAGEMANAGER.Read_16Bits(826);
  Gyro_NotchStoredInEEPROM = STORAGEMANAGER.Read_16Bits(828);
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O LPF NO ACC
  if (Acc_LPFStoredInEEPROM != Acc_LPF)
  {
    AccFilterLPF_Initialization(&AccelerometerLPFApplyFn, AccelerometerLPFState, Acc_LPFStoredInEEPROM);
    Acc_LPF = Acc_LPFStoredInEEPROM;
  }
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O LPF NO GYRO
  if (Gyro_LPFStoredInEEPROM != Gyro_LPF)
  {
    GyroFilterLPF_Initialization(&GyroscopeLPFApplyFn, GyroscopeLPFState, Gyro_LPFStoredInEEPROM);
    Gyro_LPF = Gyro_LPFStoredInEEPROM;
  }
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O NOTCH NO ACC
  if (Acc_NotchStoredInEEPROM != Acc_Notch)
  {

    Acc_Notch = Acc_NotchStoredInEEPROM;
  }
  //SE NECESSARIO GERA UM NOVO COEFICIENTE PARA O NOTCH NO GYRO
  if (Gyro_NotchStoredInEEPROM != Gyro_Notch)
  {

    Gyro_Notch = Gyro_NotchStoredInEEPROM;
  }
}

void Acc_Initialization()
{
  I2C.WriteRegister(0x68, 0x1C, 0x10);
  if (Compass_Type == COMPASS_HMC5883 && COMPASS.FakeHMC5883Address != 0x0D && I2C.CompassFound)
  {
    I2C.WriteRegister(0x68, 0x6A, 0b00100000);
    I2C.WriteRegister(0x68, 0x37, 0x00);
    I2C.WriteRegister(0x68, 0x24, 0x0D);
    I2C.WriteRegister(0x68, 0x25, 0x80 | MagAddress);
    I2C.WriteRegister(0x68, 0x26, MagRegister);
    I2C.WriteRegister(0x68, 0x27, 0x86);
  }
}

void Gyro_Initialization()
{
  I2C.WriteRegister(0x68, 0x6B, 0x80);
  AVRTIME.SchedulerSleep(50);
  I2C.WriteRegister(0x68, 0x6B, 0x03);
  GyroLPF = STORAGEMANAGER.Read_8Bits(816);
  switch (GyroLPF)
  {

  case 0:
    I2C.WriteRegister(0x68, 0x1A, 4); //20HZ
    break;

  case 1:
    I2C.WriteRegister(0x68, 0x1A, 3); //42HZ
    break;

  case 2:
    I2C.WriteRegister(0x68, 0x1A, 2); //98HZ
    break;

  case 3:
    I2C.WriteRegister(0x68, 0x1A, 1); //188HZ
    break;

  case 4:
    I2C.WriteRegister(0x68, 0x1A, 0); //256HZ
    break;
  }
  I2C.WriteRegister(0x68, 0x1B, 0x18);
  if (I2C.CompassFound)
    I2C.WriteRegister(0x68, 0x37, 0x02);
}

void Acc_ReadBufferData()
{
  I2C.SensorsRead(0x68, 0x3B);
  IMU.AccelerometerRead[ROLL] = ((BufferData[0] << 8) | BufferData[1]) >> 3;
  IMU.AccelerometerRead[PITCH] = -((BufferData[2] << 8) | BufferData[3]) >> 3;
  IMU.AccelerometerRead[YAW] = ((BufferData[4] << 8) | BufferData[5]) >> 3;

  //CALIBRAÇÃO DO ACELEROMETRO
  Accelerometer_Calibration();

  if (CalibratingAccelerometer > 0)
    return;

  //KALMAN
  if (ActiveKalman)
    KALMAN.Apply_In_Acc(IMU.AccelerometerRead);

  //LPF
  if (Acc_LPFStoredInEEPROM > 0)
  {
    //GUARDA O VALOR ATUAL DO ACELEROMETRO
    AccelerometerToFilter[ROLL] = IMU.AccelerometerRead[ROLL];
    AccelerometerToFilter[PITCH] = IMU.AccelerometerRead[PITCH];
    AccelerometerToFilter[YAW] = IMU.AccelerometerRead[YAW];
    //APLICA O FILTRO
    IMU.AccelerometerRead[ROLL] = AccelerometerLPFApplyFn((PT1Filter_Struct *)&AccelerometerLPFState[ROLL], AccelerometerToFilter[ROLL]);
    IMU.AccelerometerRead[PITCH] = AccelerometerLPFApplyFn((PT1Filter_Struct *)&AccelerometerLPFState[PITCH], AccelerometerToFilter[PITCH]);
    IMU.AccelerometerRead[YAW] = AccelerometerLPFApplyFn((PT1Filter_Struct *)&AccelerometerLPFState[YAW], AccelerometerToFilter[YAW]);
  }
  //NOTCH
  if (Acc_NotchStoredInEEPROM > 0)
  {
  }
}

void Gyro_ReadBufferData()
{
  I2C.SensorsRead(0x68, 0x43);
  IMU.GyroscopeRead[ROLL] = -((BufferData[0] << 8) | BufferData[1]) >> 2;
  IMU.GyroscopeRead[PITCH] = ((BufferData[2] << 8) | BufferData[3]) >> 2;
  IMU.GyroscopeRead[YAW] = -((BufferData[4] << 8) | BufferData[5]) >> 2;

  //CALIBRAÇÃO DO GYROSCOPIO
  Gyroscope_Calibration();

  if (CalibratingGyroscope > 0)
    return;

  //KALMAN
  if (ActiveKalman)
    KALMAN.Apply_In_Gyro(IMU.GyroscopeRead);

  //LPF
  if (Gyro_LPFStoredInEEPROM > 0)
  {
    //GUARDA O VALOR ATUAL DO GYROSCOPIO
    GyroscopeToFilter[ROLL] = IMU.GyroscopeRead[ROLL];
    GyroscopeToFilter[PITCH] = IMU.GyroscopeRead[PITCH];
    GyroscopeToFilter[YAW] = IMU.GyroscopeRead[YAW];
    //APLICA O FILTRO
    IMU.GyroscopeRead[ROLL] = GyroscopeLPFApplyFn((PT1Filter_Struct *)&GyroscopeLPFState[ROLL], GyroscopeToFilter[ROLL]);
    IMU.GyroscopeRead[PITCH] = GyroscopeLPFApplyFn((PT1Filter_Struct *)&GyroscopeLPFState[PITCH], GyroscopeToFilter[PITCH]);
    IMU.GyroscopeRead[YAW] = GyroscopeLPFApplyFn((PT1Filter_Struct *)&GyroscopeLPFState[YAW], GyroscopeToFilter[YAW]);
  }
  //NOTCH
  if (Gyro_NotchStoredInEEPROM > 0)
  {
  }
}