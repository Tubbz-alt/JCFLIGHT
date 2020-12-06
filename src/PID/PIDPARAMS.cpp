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

#include "PIDPARAMS.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "IOMCU/IOMCU.h"
#include "BAR/BAR.h"

void LoadPID()
{
    PID[ROLL].ProportionalVector = 35;
    PID[ROLL].IntegratorVector = 25;
    PID[ROLL].DerivativeVector = 26;
    PID[PITCH].ProportionalVector = 35;
    PID[PITCH].IntegratorVector = 25;
    PID[PITCH].DerivativeVector = 26;
    PID[YAW].ProportionalVector = 69;
    PID[YAW].IntegratorVector = 50;
    PID[YAW].DerivativeVector = 0;
    PID[PIDAUTOLEVEL].ProportionalVector = 80;
    PID[PIDAUTOLEVEL].IntegratorVector = 3;
    PID[PIDAUTOLEVEL].DerivativeVector = 100;
    PID[PIDALTITUDE].ProportionalVector = 50;
    PID[PIDALTITUDE].IntegratorVector = 20;
    PID[PIDALTITUDE].DerivativeVector = 16;
    PID[PIDGPSPOSITION].ProportionalVector = 1.0 * 100;
    PID[PIDGPSPOSITION].IntegratorVector = 0.9 * 100;
    PID[PIDGPSPOSITION].DerivativeVector = 0;
    PID[PIDGPSPOSITIONRATE].ProportionalVector = 7.0 * 10;
    PID[PIDGPSPOSITIONRATE].IntegratorVector = 0.2 * 100;
    PID[PIDGPSPOSITIONRATE].DerivativeVector = 0.02 * 1000;
    PID[PIDGPSNAVIGATIONRATE].ProportionalVector = 2.5 * 10;
    PID[PIDGPSNAVIGATIONRATE].IntegratorVector = 0.33 * 100;
    PID[PIDGPSNAVIGATIONRATE].DerivativeVector = 0.083 * 1000;
    PID[PIDYAWVELOCITY].ProportionalVector = 40;
}

void UpdateValuesOfPID()
{
    static bool ValuesUpdated = false;
    if (ValuesUpdated && GCS.UpdatePID)
        return;
    if ((STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != PID[ROLL].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) != 0))
        PID[ROLL].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KP_ROLL_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KP_ROLL_ADDR, PID[ROLL].ProportionalVector);

    if ((STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != PID[ROLL].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) != 0))
        PID[ROLL].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KI_ROLL_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KI_ROLL_ADDR, PID[ROLL].IntegratorVector);

    if ((STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != PID[ROLL].DerivativeVector) &&
        (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) != 0))
        PID[ROLL].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KD_ROLL_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KD_ROLL_ADDR, PID[ROLL].DerivativeVector);

    if ((STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != PID[PITCH].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) != 0))
        PID[PITCH].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KP_PITCH_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KP_PITCH_ADDR, PID[PITCH].ProportionalVector);

    if ((STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != PID[PITCH].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) != 0))
        PID[PITCH].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KI_PITCH_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KI_PITCH_ADDR, PID[PITCH].IntegratorVector);

    if ((STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != PID[PITCH].DerivativeVector) &&
        (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) != 0))
        PID[PITCH].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KD_PITCH_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KD_PITCH_ADDR, PID[PITCH].DerivativeVector);

    if ((STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != PID[YAW].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) != 0))
        PID[YAW].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KP_YAW_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KP_YAW_ADDR, PID[YAW].ProportionalVector);

    if ((STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != PID[YAW].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) != 0))
        PID[YAW].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KI_YAW_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KI_YAW_ADDR, PID[YAW].IntegratorVector);

    if ((STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != PID[YAW].DerivativeVector) &&
        (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) != 0))
        PID[YAW].DerivativeVector = STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KD_YAW_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KD_YAW_ADDR, PID[YAW].DerivativeVector);

    if ((STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != PID[PIDALTITUDE].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) != 0))
        PID[PIDALTITUDE].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KP_ALTITUDE_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KP_ALTITUDE_ADDR, PID[PIDALTITUDE].ProportionalVector);

    if ((STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != PID[PIDGPSPOSITION].ProportionalVector) &&
        (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) != 0))
        PID[PIDGPSPOSITION].ProportionalVector = STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KP_GPSPOS_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KP_GPSPOS_ADDR, PID[PIDGPSPOSITION].ProportionalVector);

    if ((STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != PID[PIDGPSPOSITION].IntegratorVector) &&
        (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) != 0))
        PID[PIDGPSPOSITION].IntegratorVector = STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR);

    if (STORAGEMANAGER.Read_8Bits(KI_GPSPOS_ADDR) == 0)
        STORAGEMANAGER.Write_8Bits(KI_GPSPOS_ADDR, PID[PIDGPSPOSITION].IntegratorVector);

    GCS.UpdatePID = ValuesUpdated = true;
}
