#include "CHECK2D.h"
#include "Math/AVRMATH.h"

bool CrashCheck2D(int16_t ValueA, int16_t ValueB, float CrashAngle)
{
  //NÃO ERA NECESSARIO ESSAS DUAS VARIAVIES,MAS EU COLOQUEI POR QUE SOU UM BOBÃO
  static int16_t ConstrainAngleRoll = 0;
  static int16_t ConstrainAnglePitch = 0;
  CrashAngle = CrashAngle * (3.14f / 180.0f) * 1000.0f;
  ConstrainAngleRoll = Constrain_16Bits(ValueA, -1000, 1000);  //ANGULO MAXIMO DE -1000 A +1000
  ConstrainAnglePitch = Constrain_16Bits(ValueB, -1000, 1000); //ANGULO MAXIMO DE -1000 A +1000
  if ((ConstrainAngleRoll > CrashAngle) || (ConstrainAngleRoll < (-CrashAngle)) ||
      (ConstrainAnglePitch > CrashAngle) || (ConstrainAnglePitch < (-CrashAngle)))
  {
    return true;
  }
  return false;
}
