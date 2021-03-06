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

#ifndef TIMEMONITOR_h
#define TIMEMONITOR_h
#include "Arduino.h"
//#define ENABLE_TIMEMONITOR
enum Function_Names
{
  SLOW_LOOP = 0,
  MEDIUM_LOOP,
  FAST_MEDIUM_LOOP,
  FAST_LOOP,
  TOTAL_LOOP,
  SIZE_LOOPS
};
class AVRTimeMonitor
{
public:
  void MeasuringStartTime(uint8_t FunctionNumber);
  void MeasuringFinishTime();

private:
  float AVRTotalTime = 0;
  uint32_t StartTime = 0;
  uint32_t EndTime = 0;
};
extern AVRTimeMonitor AVRTIMEMONITOR;
#endif
