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

#ifndef COMPASSREAD_H_
#define COMPASSREAD_H_
#include "Arduino.h"
class CompassReadClass
{
public:
  uint8_t FakeHMC5883Address = 0;
  void Constant_Read();
  void Initialization();
  void SetOrientation(uint8_t Orientation, uint8_t _CompassType);
  void Rotate();

private:
  bool PushBias(uint8_t bias);
  void InitialReadBufferData();
  void ReadBufferData();
};
extern CompassReadClass COMPASS;
#endif
