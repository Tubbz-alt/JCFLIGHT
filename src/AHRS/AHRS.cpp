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

#include "AHRS.h"
#include "Math/AVRLOWER.h"
#include "Common/STRUCTS.h"
#include "Filters/PT1.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "I2C/I2C.h"
#include "BAR/BAR.h"
#include "Yaw/YAWMANIPULATION.h"
#include "QUATERNION.h"

#define SPIN_RATE_LIMIT 20     //VALOR DE GYRO^2 PARA CORTAR A CORREÇÃO DO INTEGRAL NO AHRS
#define MAX_ACC_SQ_NEARNESS 25 //25% (0.87G - 1.12G)
#define NEARNESS 100.0f        //FATOR DE CORREÇÃO DO ACELEROMETRO NO AHRS

#define ACC_1G 512                //1G DA IMU - RETIRADO DO DATASHEET E COM BASE NA CONFIGURAÇÃO APLICADA
#define GYRO_SCALE (1.0f / 16.4f) //16.4 - RETIRADO DO DATASHEET E COM BASE NA CONFIGURAÇÃO APLICADA
#define GRAVITY_CMSS 980.665f     //VALOR DA GRAVIDADE EM CM/S^2

//FUNÇÕES MATEMATICAS PARA CALCULO DO AHRS
//PODEMOS TROCAR PARA UMA MATEMATICA EXPRESSA EM LINHAS DE CODIGO
//SEM TER A NECESSIDADE DE DEPENDER DA OTIMIZAÇÃO DO GCC
//EM BREVE EU ADICIONO ISSO
#define sin_approx(x) sinf(x)
#define cos_approx(x) cosf(x)
#define atan2_approx(y, x) atan2f(y, x)
#define acos_approx(x) acosf(x)

typedef struct
{
  float kP_Accelerometer = 0.25f;
  float kI_Accelerometer = 0.0050f;
  float kP_Magnetometer = 1;
  float ki_Magnetometer = 0;
  float Acc_Ignore_Rate = 0;
  float Acc_Ignore_Slope = 0;
} Struct_IMURuntimeConfiguration;

Struct_Vector3x3 BodyFrameAcceleration;
Struct_Vector3x3 BodyFrameRotation;
Struct_Quaternion Orientation;

static Struct_Vector3x3 CorrectedMagneticFieldNorth;
static Struct_IMURuntimeConfiguration IMURuntimeConfiguration;
static PT1_Filter_Struct RotationRateFilter;

static bool GPSHeadingInitialized = false;

float RotationMath[3][3];

static void ComputeRotationMatrix(void)
{
  float q1q1 = Orientation.q1 * Orientation.q1;
  float q2q2 = Orientation.q2 * Orientation.q2;
  float q3q3 = Orientation.q3 * Orientation.q3;
  float q0q1 = Orientation.q0 * Orientation.q1;
  float q0q2 = Orientation.q0 * Orientation.q2;
  float q0q3 = Orientation.q0 * Orientation.q3;
  float q1q2 = Orientation.q1 * Orientation.q2;
  float q1q3 = Orientation.q1 * Orientation.q3;
  float q2q3 = Orientation.q2 * Orientation.q3;
  RotationMath[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
  RotationMath[0][1] = 2.0f * (q1q2 + -q0q3);
  RotationMath[0][2] = 2.0f * (q1q3 - -q0q2);
  RotationMath[1][0] = 2.0f * (q1q2 - -q0q3);
  RotationMath[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
  RotationMath[1][2] = 2.0f * (q2q3 + -q0q1);
  RotationMath[2][0] = 2.0f * (q1q3 + -q0q2);
  RotationMath[2][1] = 2.0f * (q2q3 - -q0q1);
  RotationMath[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void AHRS_Initialization(void)
{
  //CALCULA A DECLINAÇÃO MAGNETICA
  const int16_t Degrees = ((int16_t)(STORAGEMANAGER.Read_Float(DECLINATION_ADDR) * 100)) / 100;
  const int16_t Remainder = ((int16_t)(STORAGEMANAGER.Read_Float(DECLINATION_ADDR) * 100)) % 100;
  const float CalcedvalueInRadians = -ConvertToRadians(Degrees + Remainder / 60.0f);
  CorrectedMagneticFieldNorth.Roll = cos_approx(CalcedvalueInRadians);
  CorrectedMagneticFieldNorth.Pitch = sin_approx(CalcedvalueInRadians);
  CorrectedMagneticFieldNorth.Yaw = 0;
  //RESETA O QUATERNION,A MATRIX E O FILTRO
  QuaternionInitUnit(&Orientation);
  ComputeRotationMatrix();
  RotationRateFilter.State = 0;
}

static bool ValidateQuaternion(const Struct_Quaternion *Quaternion)
{
  const float CheckAbsoluteValue = ABS_FLOAT(Quaternion->q0) +
                                   ABS_FLOAT(Quaternion->q1) +
                                   ABS_FLOAT(Quaternion->q2) +
                                   ABS_FLOAT(Quaternion->q3);
  if (!isnan(CheckAbsoluteValue) && !isinf(CheckAbsoluteValue))
    return true;
  const float normSq = QuaternionNormalizedSquared(&Orientation);
  if (normSq > (1.0f - 1e-6f) && normSq < (1.0f + 1e-6f))
    return true;
  return false;
}

static void ResetOrientationQuaternion(const Struct_Vector3x3 *AccelerationBodyFrame)
{
  const float accNorm = sqrtf(VectorNormSquared(AccelerationBodyFrame));
  Orientation.q0 = AccelerationBodyFrame->Yaw + accNorm;
  Orientation.q1 = AccelerationBodyFrame->Pitch;
  Orientation.q2 = -AccelerationBodyFrame->Roll;
  Orientation.q3 = 0.0f;
  QuaternionNormalize(&Orientation, &Orientation);
}

static void CheckAndResetOrientationQuaternion(const Struct_Quaternion *Quaternion,
                                               const Struct_Vector3x3 *AccelerationBodyFrame)
{
  //CHECA SE O QUATERNION ESTÁ NORMAL
  if (ValidateQuaternion(&Orientation))
    return;
  //ORIENTAÇÃO INVALIDA,É NECESSARIO RESETAR O QUATERNION
  if (ValidateQuaternion(Quaternion))
  {
    //OBTÉM O VALOR ANTERIOR VALIDO
    Orientation = *Quaternion;
  }
  else
  {
    //REFERENCIA INVALIDA,O ACELEROMETRO PODE ESTAR RUIM
    ResetOrientationQuaternion(AccelerationBodyFrame);
  }
}

static void MahonyAHRSUpdate(float DeltaTime,
                             const Struct_Vector3x3 *RotationBodyFrame,
                             const Struct_Vector3x3 *AccelerationBodyFrame,
                             const Struct_Vector3x3 *MagnetometerBodyFrame,
                             bool GPS_HeadingState, float CourseOverGround,
                             float AccelerometerWeightScaler,
                             float MagnetometerWeightScaler)
{
  static Struct_Vector3x3 GyroDriftEstimate = {0};

  Struct_Quaternion PreviousOrientation = Orientation;
  Struct_Vector3x3 RotationRate = *RotationBodyFrame;

  //CALCULA O VALOR DO SPIN RATE EM RADIANOS/S
  const float Spin_Rate_Square = VectorNormSquared(&RotationRate);

  //CORREÇÃO DO ROLL E PITCH USANDO O VETOR DO ACELEROMETRO
  if (AccelerationBodyFrame)
  {
    static const Struct_Vector3x3 Gravity = {.Vector = {0.0f,
                                                        0.0f,
                                                        1.0f}};
    Struct_Vector3x3 EstimatedGravity,
        AccelerationVector,
        VectorError;

    //ESTIMA A GRAVIDADE NO BODY FRAME
    QuaternionRotateVector(&EstimatedGravity, &Gravity, &Orientation);

    //ESTIMA A DIREÇÃO DA GRAVIDADE
    VectorNormalize(&AccelerationVector, AccelerationBodyFrame);
    VectorCrossProduct(&VectorError, &AccelerationVector, &EstimatedGravity);

    //CALCULA E APLICA O FEEDBACK INTEGRAL
    if (IMURuntimeConfiguration.kI_Accelerometer > 0.0f)
    {
      //DESATIVA A INTEGRAÇÃO SE O SPIN RATE ULTRAPASSAR UM CERTO VALOR
      if (Spin_Rate_Square < sq(ConvertToRadians(SPIN_RATE_LIMIT)))
      {
        Struct_Vector3x3 OldVector;
        //CALCULA O ERRO ESCALADO POR Ki
        VectorScale(&OldVector, &VectorError, IMURuntimeConfiguration.kI_Accelerometer * DeltaTime);
        VectorAdd(&GyroDriftEstimate, &GyroDriftEstimate, &OldVector);
      }
    }
    //CALCULA O GANHO DE Kp E APLICA O FEEDBACK PROPORCIONAL
    VectorScale(&VectorError, &VectorError, IMURuntimeConfiguration.kP_Accelerometer * AccelerometerWeightScaler);
    VectorAdd(&RotationRate, &RotationRate, &VectorError);
  }

  //CORREÇÃO DO YAW
  if (MagnetometerBodyFrame || GPS_HeadingState)
  {
    static const Struct_Vector3x3 Forward = {.Vector = {1.0f,
                                                        0.0f,
                                                        0.0f}};

    Struct_Vector3x3 VectorError = {.Vector = {0.0f,
                                               0.0f,
                                               0.0f}};

    if (MagnetometerBodyFrame && VectorNormSquared(MagnetometerBodyFrame) > 0.01f)
    {
      Struct_Vector3x3 MagnetormeterVector;

      //CALCULA O NORTE MAGNETICO
      QuaternionRotateVectorInv(&MagnetormeterVector, MagnetometerBodyFrame, &Orientation);

      //IGNORA A INCLINAÇÃO Z DO MAGNETOMETRO
      MagnetormeterVector.Yaw = 0.0f;

      //VERIFICA SE O MAGNETOMETRO^2 ESTÁ OK
      if (VectorNormSquared(&MagnetormeterVector) > 0.01f)
      {
        //NORMALIZA O VETOR DO MAGNETOMETRO
        VectorNormalize(&MagnetormeterVector, &MagnetormeterVector);

        //CALCULA A REFERENCIA DO COMPASS
        VectorCrossProduct(&VectorError, &MagnetormeterVector, &CorrectedMagneticFieldNorth);

        //CALCULA O ERRO DE ROTAÇÃO NO BODY FRAME
        QuaternionRotateVector(&VectorError, &VectorError, &Orientation);
      }
    }
    else if (GPS_HeadingState)
    {
      Struct_Vector3x3 HeadingEarthFrame;

      //USE O COG DO GPS PARA CALCULAR O HEADING
      while (CourseOverGround > 3.14159265358979323846f)
        CourseOverGround -= (2.0f * 3.14159265358979323846f);

      while (CourseOverGround < -3.14159265358979323846f)
        CourseOverGround += (2.0f * 3.14159265358979323846f);

      //CALCULA O VALOR DE HEADING COM BASE NO COG
      Struct_Vector3x3 vCoG = {.Vector = {-cos_approx(CourseOverGround),
                                          sin_approx(CourseOverGround), 0.0f}};

      //ROTACIONA O VETOR DO BODY FRAME PARA EARTH FRAME
      QuaternionRotateVectorInv(&HeadingEarthFrame, &Forward, &Orientation);
      HeadingEarthFrame.Yaw = 0.0f;

      //CORRIJA APENAS SE O SQUARE FOR POSITIVO E MAIOR QUE ZERO
      if (VectorNormSquared(&HeadingEarthFrame) > 0.01f)
      {
        //NORMALIZA O VETOR
        VectorNormalize(&HeadingEarthFrame, &HeadingEarthFrame);

        //CALCULA O ERRO
        VectorCrossProduct(&VectorError, &vCoG, &HeadingEarthFrame);

        //ROTACIONA O ERRO NO BODY FRAME
        QuaternionRotateVector(&VectorError, &VectorError, &Orientation);
      }
    }

    //CALCULA E APLICA O FEEDBACK INTEGRAL
    if (IMURuntimeConfiguration.ki_Magnetometer > 0.0f)
    {
      //PARE A INTEGRAÇÃO SE O SPIN RATE FOR MAIOR QUE O LIMITE
      if (Spin_Rate_Square < sq(ConvertToRadians(SPIN_RATE_LIMIT)))
      {
        Struct_Vector3x3 OldVector;

        //CALCULA O ERRO INTEGRAL ESCALADO POR Ki
        VectorScale(&OldVector, &VectorError, IMURuntimeConfiguration.ki_Magnetometer * DeltaTime);
        VectorAdd(&GyroDriftEstimate, &GyroDriftEstimate, &OldVector);
      }
    }

    //CALCULA O GANHO DE Kp E APLICA O FEEDBACK DO PROPORCIONAL
    VectorScale(&VectorError, &VectorError, IMURuntimeConfiguration.kP_Magnetometer * MagnetometerWeightScaler);
    VectorAdd(&RotationRate, &RotationRate, &VectorError);
  }

  //APLICA A CORREÇÃO DE DRIFT DO GYROSCOPIO
  VectorAdd(&RotationRate, &RotationRate, &GyroDriftEstimate);

  //TAXA DE MUDANÇA DO QUATERNION
  Struct_Vector3x3 Theta; //THETA É A ROTAÇÃO DE EIXO/ÂNGULO.
  Struct_Quaternion QuaternionDelta;

  VectorScale(&Theta, &RotationRate, 0.5f * DeltaTime);
  QuaternionInitFromVector(&QuaternionDelta, &Theta);
  const float ThetaSquared = VectorNormSquared(&Theta);

  //ATUALIZE O QUATERNION APENAS SE A ROTAÇÃO FOR MAIOR QUE ZERO
  if (ThetaSquared >= 1e-20)
  {
    //CONFORME THETA SE APROXIMA DE ZERO,AS OPERAÇÕES DE SENO E COSENO SE TORNAM CADA VEZ MAIS INSTAVEIS,
    //PARA CONTORNAR ISSO,USAMOS A SÉRIE DE TAYLOR,VERIFICAMOS SE O SQUARE DE THETA É MENOR QUE A PRECISÃO
    //FLOAT DO MICROCONTROLADOR (FLOAT = 24 BITS COM EXPONENTE DE 8 BITS),SENDO ASSIM SER POSSIVEL CALCULAR
    //COM SEGURANÇA O VALOR DE UM ÂNGULO PEQUENO SEM PERDER A PRECISÃO NUMÉRICA.
    if (ThetaSquared < sqrtf(24.0f * 1e-6f))
    {
      QuaternionScale(&QuaternionDelta, &QuaternionDelta, 1.0f - ThetaSquared / 6.0f);
      QuaternionDelta.q0 = 1.0f - ThetaSquared / 2.0f;
    }
    else
    {
      const float thetaMagnitude = sqrtf(ThetaSquared);
      QuaternionScale(&QuaternionDelta, &QuaternionDelta, sin_approx(thetaMagnitude) / thetaMagnitude);
      QuaternionDelta.q0 = cos_approx(thetaMagnitude);
    }

    //CALCULA O VALOR FINA DA ORIENTAÇÃO E RENORMALIZA O QUATERNION
    QuaternionMultiply(&Orientation, &Orientation, &QuaternionDelta);
    QuaternionNormalize(&Orientation, &Orientation);
  }

  //CHECA SE O NOVO VALOR DO QUATERNION É VALIDO,SE SIM RESETA O VALOR ANTIGO
  CheckAndResetOrientationQuaternion(&PreviousOrientation, AccelerationBodyFrame);

  //CALCULA A MATRIX ROTATIVA PARA O QUATERNION
  ComputeRotationMatrix();
}

static float CalculateAccelerometerWeight(const float DeltaTime)
{
  float AccelerometerMagnitudeSquare = 0;
  //CALCULA A RAIZ QUADRADA DE TODOS OS EIXOS DO ACELEROMETRO PARA EXTRAIR A MAGNITUDE
  AccelerometerMagnitudeSquare += SquareFloat((float)IMU.AccelerometerRead[ROLL] / ACC_1G);
  AccelerometerMagnitudeSquare += SquareFloat((float)IMU.AccelerometerRead[PITCH] / ACC_1G);
  AccelerometerMagnitudeSquare += SquareFloat((float)IMU.AccelerometerRead[YAW] / ACC_1G);
  //CALCULA A MAGNITUDE DO ACELEROMETRO EM %
  const float Nearness = ABS_FLOAT(100 - (AccelerometerMagnitudeSquare * 100));
  const float AccWeight_Nearness = (Nearness > MAX_ACC_SQ_NEARNESS) ? 0.0f : NEARNESS;

  float AccWeight_RateIgnore = 1.0f;

  //A VELOCIDADE EM AEROS É BEM SUPERIOR A DOS DRONES,QUANTO MAIOR A VELOCIDADE,MAIOR A FORÇA CENTRÍFUGA,
  //SENDO ASSIM MAIOR SERÁ O ERRO DE ÂNGULO NO ACELEROMETRO,POR EXEMPLO,
  //EM UMA CURVA DE 5 GRAUS COM UMA VELOCIDADE DE 20M/S FACILMENTE TERIAMOS O DOBRO DE ERRO DE INCLINAÇÃO,
  //PARA CONTORNAR ISSO,A TAXA DE MAGNITUDE DO ACELEROMETRO É FILTRADA POR UM LPF COM FREQUENCIA DE 1HZ,
  //E SE O VALOR FOR MAIOR QUE O LIMITE O ACELEROMETRO DEIXA DE SER USADO PELO AHRS.
  if (COMMAND_ARM_DISARM && (FrameType == 3 || FrameType == 4 || FrameType == 5) && IMURuntimeConfiguration.Acc_Ignore_Rate != 0)
  {
    const float RotationRateMagnitude = sqrtf(VectorNormSquared(&BodyFrameRotation));
    const float RotationRateMagnitudeFiltered = PT1FilterApply(&RotationRateFilter, RotationRateMagnitude, 1.0f, DeltaTime);
    if (IMURuntimeConfiguration.Acc_Ignore_Slope != 0)
    {
      const float RateSlopeMin = ConvertToRadians((IMURuntimeConfiguration.Acc_Ignore_Rate - IMURuntimeConfiguration.Acc_Ignore_Slope));
      const float RateSlopeMax = ConvertToRadians((IMURuntimeConfiguration.Acc_Ignore_Rate + IMURuntimeConfiguration.Acc_Ignore_Slope));
      AccWeight_RateIgnore = Map_Float(Constrain_Float(RotationRateMagnitudeFiltered, RateSlopeMin, RateSlopeMax), RateSlopeMin, RateSlopeMax, 1.0f, 0.0f);
    }
    else
    {
      if (RotationRateMagnitudeFiltered > ConvertToRadians(IMURuntimeConfiguration.Acc_Ignore_Rate))
      {
        AccWeight_RateIgnore = 0.0f;
      }
    }
  }

  return AccWeight_Nearness * AccWeight_RateIgnore;
}

static void ComputeQuaternionFromRPY(int16_t initialRoll, int16_t initialPitch, int16_t initialYaw)
{
  if (initialRoll > 1800)
    initialRoll -= 3600;

  if (initialPitch > 1800)
    initialPitch -= 3600;

  if (initialYaw > 1800)
    initialYaw -= 3600;

  const float cosRoll = cos_approx(ConvertDeciDegreesToRadians(initialRoll) * 0.5f);
  const float sinRoll = sin_approx(ConvertDeciDegreesToRadians(initialRoll) * 0.5f);

  const float cosPitch = cos_approx(ConvertDeciDegreesToRadians(initialPitch) * 0.5f);
  const float sinPitch = sin_approx(ConvertDeciDegreesToRadians(initialPitch) * 0.5f);

  const float cosYaw = cos_approx(ConvertDeciDegreesToRadians(-initialYaw) * 0.5f);
  const float sinYaw = sin_approx(ConvertDeciDegreesToRadians(-initialYaw) * 0.5f);

  Orientation.q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
  Orientation.q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  Orientation.q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  Orientation.q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

  ComputeRotationMatrix();
}

void GetMeasuredAcceleration(Struct_Vector3x3 *MeasureAcceleration)
{
  MeasureAcceleration->Vector[ROLL] = ((float)IMU.AccelerometerRead[ROLL] / ACC_1G) * GRAVITY_CMSS;
  MeasureAcceleration->Vector[PITCH] = ((float)IMU.AccelerometerRead[PITCH] / ACC_1G) * GRAVITY_CMSS;
  MeasureAcceleration->Vector[YAW] = ((float)IMU.AccelerometerRead[YAW] / ACC_1G) * GRAVITY_CMSS;
}

void GetMeasuredRotationRate(Struct_Vector3x3 *MeasureRotation)
{
  MeasureRotation->Vector[ROLL] = ConvertToRadians(((float)IMU.GyroscopeRead[ROLL] * GYRO_SCALE));
  MeasureRotation->Vector[PITCH] = ConvertToRadians(((float)IMU.GyroscopeRead[PITCH] * GYRO_SCALE));
  MeasureRotation->Vector[YAW] = ConvertToRadians(((float)IMU.GyroscopeRead[YAW] * GYRO_SCALE));
}

void AHRS_Update()
{
  bool SafeToUseCompass = false;
  bool GPS_HeadingState = false;
  float CourseOverGround = 0;
  static uint32_t previousIMUUpdateTimeUs;
  const float DeltaTime = (AVRTIME.SchedulerMicros() - previousIMUUpdateTimeUs) * 1e-6;
  previousIMUUpdateTimeUs = AVRTIME.SchedulerMicros();

  GetMeasuredRotationRate(&BodyFrameRotation);     //CALCULA A ROTAÇÃO DA IMU EM RADIANOS/S
  GetMeasuredAcceleration(&BodyFrameAcceleration); //CALCULA A ACELERAÇÃO DA IMU EM CM/S^2

  if (FrameType == 3 || FrameType == 4 || FrameType == 5)
  {
    //SLIP ANGLE PARA O MODO PLANE
    SlipAngleForAirPlane = ApproximationOfAtan2(IMU.AccelerometerRead[ROLL], ABS_16BITS(IMU.AccelerometerRead[YAW]));
    SlipAngleForAirPlane = Constrain_16Bits(SlipAngleForAirPlane, -900, 900); //APLICA LIMITES DE -90 Á +90 GRAUS

    bool canUseCOG = (GPS_NumberOfSatellites >= 6 && GPS_Ground_Speed >= 300);

    if (I2C.CompassFound)
    {
      SafeToUseCompass = true;
      GPSHeadingInitialized = true;
    }
    else if (canUseCOG)
    {
      if (GPSHeadingInitialized)
      {
        CourseOverGround = ConvertDeciDegreesToRadians(GPS_Ground_Course);
        GPS_HeadingState = true;
      }
      else
      {
        ComputeQuaternionFromRPY(ATTITUDE.AngleOut[ROLL], ATTITUDE.AngleOut[PITCH], GPS_Ground_Course);
        GPSHeadingInitialized = true;
        ForceYawReset = true;
      }
    }
  }
  else
  {
    if (I2C.CompassFound)
    {
      SafeToUseCompass = true;
    }
  }

  Struct_Vector3x3 MagnetometerBodyFrame = {.Vector = {(float)IMU.CompassRead[ROLL],
                                                       (float)IMU.CompassRead[PITCH],
                                                       (float)IMU.CompassRead[YAW]}};

  const float CalcedCompassWeight = NEARNESS;
  const float CalcedAccelerometerWeight = CalculateAccelerometerWeight(DeltaTime);
  const bool SafeToUseAccelerometer = (CalcedAccelerometerWeight > 0.001f);

  MahonyAHRSUpdate(DeltaTime, &BodyFrameRotation,
                   SafeToUseAccelerometer ? &BodyFrameAcceleration : NULL,
                   SafeToUseCompass ? &MagnetometerBodyFrame : NULL,
                   GPS_HeadingState, CourseOverGround,
                   CalcedAccelerometerWeight,
                   CalcedCompassWeight);

  //SAÍDA DOS EIXOS DO APÓS O AHRS
  //ROLL
  ATTITUDE.AngleOut[ROLL] = ConvertRadiansToDeciDegrees(atan2_approx(RotationMath[2][1], RotationMath[2][2]));
  //PITCH
  ATTITUDE.AngleOut[PITCH] = ConvertRadiansToDeciDegrees((0.5f * 3.14159265358979323846f) - acos_approx(-RotationMath[2][0]));
  //YAW
  ATTITUDE.CompassHeading = ConvertRadiansToDeciDegrees(-atan2_approx(RotationMath[1][0], RotationMath[0][0]));
  //CONVERTE O VALOR DE COMPASS HEADING PARA O VALOR ACEITAVEL
  if (ATTITUDE.CompassHeading < 0)
    ATTITUDE.CompassHeading += 3600;
  ATTITUDE.CalculedHeading = ConvertDeciDegreesToDegrees(ATTITUDE.CompassHeading);
}

bool CheckAnglesInclination(int16_t Angle)
{
  Angle = Angle * 10;
  if (ATTITUDE.AngleOut[ROLL] > Angle || ATTITUDE.AngleOut[PITCH] > Angle ||
      ATTITUDE.AngleOut[ROLL] < (-Angle) || ATTITUDE.AngleOut[PITCH] < (-Angle))
    return true;
  return false;
}