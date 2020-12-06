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

#include "PIDMIXING.h"
#include "Common/VARIABLES.h"
#include "AirPlane/AIRPLANE.h"

uint8_t NumberOfMotors = 4; //APENAS PARA INICIALIZAR

void MixingSelectPID()
{
    switch (FrameType)
    {

    case 0:
    {
        //QUAD-X
        MotorControl[MOTOR1] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -1 +
                               PIDControllerApply[PITCH] * +1 + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR2] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -1 +
                               PIDControllerApply[PITCH] * -1 + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR3] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +1 +
                               PIDControllerApply[PITCH] * +1 + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR4] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +1 +
                               PIDControllerApply[PITCH] * -1 + 1 * PIDControllerApply[YAW] * -1;
        NumberOfMotors = 4;
        return;
    }

    case 1:
    {
        //HEXA-X
        MotorControl[MOTOR1] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -0.8f +
                               PIDControllerApply[PITCH] * +0.9f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR2] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -0.8f +
                               PIDControllerApply[PITCH] * -0.9f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR3] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0.8f +
                               PIDControllerApply[PITCH] * +0.9f + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR4] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0.8f +
                               PIDControllerApply[PITCH] * -0.9f + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR5] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -0.8f +
                               PIDControllerApply[PITCH] * +0 + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR6] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0.8f +
                               PIDControllerApply[PITCH] * +0 + 1 * PIDControllerApply[YAW] * +1;
        NumberOfMotors = 6;
        return;
    }

    case 2:
    {
        //HEXA+
        MotorControl[MOTOR1] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -0.9f +
                               PIDControllerApply[PITCH] * +0.8f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR2] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -0.9f +
                               PIDControllerApply[PITCH] * -0.8f + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR3] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0.9f +
                               PIDControllerApply[PITCH] * +0.8f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR4] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0.9f +
                               PIDControllerApply[PITCH] * -0.8f + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR5] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0 +
                               PIDControllerApply[PITCH] * -0.8f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR6] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0 +
                               PIDControllerApply[PITCH] * +0.8f + 1 * PIDControllerApply[YAW] * -1;
        NumberOfMotors = 6;
        return;
    }

    case 3:
    {
        //AEROMODELO
        AirPlane_Mode_ConventionalPlane_Run();
        MotorControl[MOTOR5] = 1000;
        NumberOfMotors = 0;
        return;
    }

    case 4:
    {
        //ASA-FIXA
        AirPlane_Mode_FixedWing_Run();
        MotorControl[MOTOR1] = 1000;
        MotorControl[MOTOR6] = 1000;
        MotorControl[MOTOR5] = 1000;
        NumberOfMotors = 0;
        return;
    }

    case 5:
    {
        //AERO TIPO V-TAIL
        AirPlane_Mode_PlaneVTail_Run();
        MotorControl[MOTOR5] = 1000;
        NumberOfMotors = 0;
        return;
    }

    case 6:
    {
        //ZMR250
        MotorControl[MOTOR1] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -1 +
                               PIDControllerApply[PITCH] * +0.772f + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR2] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -1 +
                               PIDControllerApply[PITCH] * -0.772f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR3] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +1 +
                               PIDControllerApply[PITCH] * +0.772f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR4] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +1 +
                               PIDControllerApply[PITCH] * -0.772f + 1 * PIDControllerApply[YAW] * -1;
        NumberOfMotors = 4;
        return;
    }

    case 7:
    {
        //DIATONE WHITE SHEEP
        MotorControl[MOTOR1] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -0.848f +
                               PIDControllerApply[PITCH] * +0.647f + 1 * PIDControllerApply[YAW] * -1;
        MotorControl[MOTOR2] = RCController[THROTTLE] + PIDControllerApply[ROLL] * -1 +
                               PIDControllerApply[PITCH] * -0.647f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR3] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +0.848f +
                               PIDControllerApply[PITCH] * +0.647f + 1 * PIDControllerApply[YAW] * +1;
        MotorControl[MOTOR4] = RCController[THROTTLE] + PIDControllerApply[ROLL] * +1 +
                               PIDControllerApply[PITCH] * -0.647f + 1 * PIDControllerApply[YAW] * -1;
        NumberOfMotors = 4;
        return;
    }
    }
}