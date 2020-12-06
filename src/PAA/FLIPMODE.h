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

#ifndef FLIPMODE_H_
#define FLIPMODE_H_
#include "Arduino.h"
extern bool LockPitchAndRollRC;
extern bool ApplyFlipRoll;
extern bool ApplyFlipPitch;
extern int16_t SampleRoll;
extern int16_t SamplePitch;
extern int16_t FlipAngleValue;
extern uint16_t ValueOfFlipToRoll;
extern uint16_t ValueOfFlipToPitch;
void FlipModeRun();
void FlipRollPitchAxis(bool _Do_Flip, uint8_t Axis);
#endif
