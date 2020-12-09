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

#ifndef BUZZER_H_
#define BUZZER_H_

#include "Arduino.h"

typedef enum
{
  BEEPER_CALIBRATION_DONE = 0,
  BEEPER_DISARMING,
  BEEPER_BAT_CRIT_LOW,
  BEEPER_ACTION_SUCCESS,
  BEEPER_ACTION_FAIL,
  BEEPER_ARM,
  BEEPER_ALGORITHM_INIT,
  BEEPER_AUTOLAUNCH,
  BEEPER_LAUNCHED,
  BEEPER_FMU_INIT,
  BEEPER_FMU_SAFE_TO_ARM
} Beeper_Mode;

class BEEPERCLASS
{
public:
  uint8_t SafeToOthersBeepsCounter;
  void Run();
  void BeeperPlay(Beeper_Mode Mode);
  void BeeperSilence();

private:
  void BeeperUpdate();
  void BeeperProcessCommand();
};
extern BEEPERCLASS BEEPER;
#endif
