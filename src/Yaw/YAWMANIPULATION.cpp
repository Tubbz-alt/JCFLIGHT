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

#include "YAWMANIPULATION.h"
#include "Common/VARIABLES.h"
#include "Math/AVRMATH.h"
#include "AHRS/AHRS.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Filters/PT1.h"

bool ForceYawReset = false;

#define HEADING_HOLD_ERROR_LPF_FREQ 2

static PT1_Filter_Struct HeadingHoldRateFilter;

void YawManipulationUpdate()
{
  uint8_t Heading_Hold_Rate_Limit = 90;
  float HeadingHoldRate;

  if (!COMMAND_ARM_DISARM || ForceYawReset) //NÃO APLICA A CORREÇÃO DO YAW COM A CONTROLADORA DESARMADA
  {
    HeadingHoldTarget = ATTITUDE.CalculedHeading; //OBTÉM UM NOVO VALOR INICIAL PARA HEADING HOLD TARGET
    HeadingHoldRateFilter.State = 0.0f;           //RESETA O FILTRO
    if (ForceYawReset)
      ForceYawReset = false;
    return;
  }

  if (CheckAnglesInclination(25)) //NÃO APLICA A CORREÇÃO DO YAW SE OS ANGULOS FOREM MAIOR QUE 25 GRAUS
    return;

  if (GPS_Flight_Mode == 0) //NÃO APLICA A CORREÇÃO DO YAW SE NENHUM MODO DE VOO USANDO O GPS ESTIVER ATIVO
    return;

  if (!GuardActualHeading) //NÃO APLICA A CORREÇÃO DO YAW SE A ATUALIZAÇÃO DO VALOR DE HEADING NÃO ACONTECER
    return;

  if (ABS_16BITS(RCController[YAW]) > 10) //NÃO APLICA A CORREÇÃO DO YAW SE O USUARIO MANIPULAR O STICK YAW DO RADIO
    return;

  int16_t YawError = ATTITUDE.CalculedHeading - HeadingHoldTarget; //CALCULA O ERRO / DIFERENÇA

  //CALCULA O VALOR RELATIVO DO ERRO / DIFERENÇA E CONVERTE PARA UM VALOR ACEITAVEL POR HEADING
  if (YawError <= -180)
  {
    YawError += 360;
  }

  if (YawError >= +180)
  {
    YawError -= 360;
  }

  //CALCULA O VALOR DO RATE
  HeadingHoldRate = YawError * PID[PIDYAWVELOCITY].ProportionalVector / 30.0f;
  //APLICA LIMITES MIN E MAX NO RATE
  HeadingHoldRate = Constrain_Float(HeadingHoldRate, -Heading_Hold_Rate_Limit, Heading_Hold_Rate_Limit);
  //REALIZA FILTRAGEM DO RATE COM O PT1
  HeadingHoldRate = PT1FilterApply(&HeadingHoldRateFilter, HeadingHoldRate, HEADING_HOLD_ERROR_LPF_FREQ, AVRTIME.SchedulerMicros() * 1e-6);
  //APLICA O CONTROLE DO YAW
  RCController[YAW] -= HeadingHoldRate; //APLICA O CONTROLE DO YAW
}