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

#include "SWITCHFLAG.h"
#include "FlightModes/AUXFLIGHT.h"
#include "Common/VARIABLES.h"
#include "AirPlane/SERVOTRIM.h"
#include "Scheduler/SCHEDULERTIME.h"

//********************************************************************************************
//ATIVAÇÃO PARA O SAVE-TRIM & CALIBRÇÃO DO MAG VIA CHAVE AUX DO MODO IOC
//SAVE-TRIM IMPLEMENTADO  >> 07/05/2020 (4 TOQUES PARA ATIVAR,2 TOQUES PARA DESATIVAR)
//CALIB MAG IMPLEMENTADO  >> 28/06/2020 (8 TOQUES PARA ATIVAR)
//SERVO-TRIM IMPLEMENTADO >> 24/09/2020 (4 TOQUES PARA ATIVAR,2 TOQUES PARA DESATIVAR)
//OBS:
//SERVO-TRIM SÓ FUNCIONA COM A CONTROLADORA DESARMADA E COM O PERFIL DE AERO
//SAVE-TRIM SÓ FUNCIONA COM A CONTROLADORA ARMADA E EM VOO
//********************************************************************************************

uint8_t SaveTrimState;
uint8_t FlagParameterFunction;
uint8_t GuardValue;
float CloseReset;
uint32_t TimerFunction;
uint32_t CR_Clear;

void Switch_Flag(void)
{
  //INICIA A CONTAGEM DA FLAG PRINCIPAL
  if (IOCControlAux)
  {
    if ((AVRTIME.SchedulerMillis() - TimerFunction) > 50)
    { //DEBOUNCE
      FlagParameterFunction += 1;
      if (GuardValue >= 4 && GuardValue <= 12)
        GuardValue += 1;
    }
    CloseReset = 5; //5 SEGUNDOS
    TimerFunction = AVRTIME.SchedulerMillis();
  }
  //DELAY PARA RESETAR A FLAG PRINCIPAL
  if (CloseReset > 0 && (AVRTIME.SchedulerMillis() - CR_Clear) > 100)
  {
    CloseReset -= 0.10f;
    CR_Clear = AVRTIME.SchedulerMillis();
  }
  if (CloseReset < 0)
    CloseReset = 0; //EVITA GUARDAR VALORES NEGATIVOS CAUSADO PELA DECREMENTAÇÃO DA FUNÇÃO ACIMA
  //RESETA A FLAG SE O VALOR DELA FOR IGUAL A 8,E A CHAVE AUX DO MODO IOC FOR FALSA
  if (FlagParameterFunction == 8 && !IOCControlAux)
    FlagParameterFunction = 0;
  //ESPERA A DECREMENTAÇÃO DA VARIAVEL ACABAR E RESETA A FLAG PRINCIPAL
  if (!IOCControlAux && CloseReset == 0)
    FlagParameterFunction = 0;
  //FLAG PRINCIPAL IGUAL A 4?CHAVE AUX ATIVADA?CAL DO MAG ACABOU?SIM...GUARDE O VALOR DA FLAG PRINCIPAL NA VARIAVEL "GUARDVALUE"
  if (FlagParameterFunction == 4 && IOCControlAux && !CalibratingCompass)
    GuardValue = FlagParameterFunction;
  //FLAG PRINCIPAL IGUAL A 8?CHAVE AUX ATIVADA?SIM...GUARDE O VALOR DA FLAG PRINCIPAL NA VARIAVEL "GUARDVALUE"
  if (FlagParameterFunction == 8 && IOCControlAux)
    GuardValue = FlagParameterFunction;
  if (COMMAND_ARM_DISARM)
  { //CONTROLADORA ARMADA?SIM...
    //O VALOR GUARDADO É IGUAL A 4?E A DECREMENTAÇÃO ACABOU?SIM...INICIA O SAVE-TRIM
    if (GuardValue == 4 && CloseReset < 2.51f)
      SaveTrimState = 1;
    //O VALOR GUARDADO É IGUAL A 6?E A DECREMENTAÇÃO ACABOU?SIM...DESATIVA O SAVE-TRIM
    if (GuardValue == 6 && CloseReset < 2.51f)
      GuardValue = SaveTrimState = 0;
  }
  else
  { //CONTROLADORA DESARMADA?SIM...
    if (GuardValue == 8 && CloseReset > 2 && CloseReset < 5)
      CalibratingCompass = true; //O VALOR GUARDADO É IGUAL A 8?E A DECREMENTAÇÃO ACABOU?SIM...INICIA A CALIBRAÇÃO DO COMPASS
    if (GuardValue == 8 && CloseReset == 2)
      GuardValue = 0;
    //ATIVA O SERVOTRIM
    if (GuardValue == 4 && CloseReset < 2.51f && (FrameType == 3 || FrameType == 4 || FrameType == 5))
      OkToTrimServo = true;
    //DESATIVA O SERVOTRIM
    if (GuardValue == 6 && CloseReset < 2.51f && (FrameType == 3 || FrameType == 4 || FrameType == 5))
      GuardValue = OkToTrimServo = 0;
  }
  //O VALOR GUARDADO É IGUAL A 12?E A DECREMENTAÇÃO ACABOU?SIM...SE O SAVE-TRIM ESTIVER ATIVADO NÃO LIMPA A FLAG,CASO CONTRARIO LIMPA
  if (GuardValue == 12 && CloseReset == 0)
  {
    if (SaveTrimState == 1 || OkToTrimServo)
      GuardValue = 4;
    else
      GuardValue = 0;
  }
  ////////////////////////////////////////////////////////////////////////////////
  /////////////////////////////LIMPEZA DE FLAG///////////////////////////////////
  if (GuardValue == 5 && CloseReset == 0)
    GuardValue = 0;
  else if (GuardValue == 7 && CloseReset == 0)
    GuardValue = 0;
  else if (GuardValue == 9 && CloseReset == 0)
    GuardValue = 0;
  else if (GuardValue == 10 && CloseReset == 0)
    GuardValue = 0;
  else if (GuardValue == 11 && CloseReset == 0)
    GuardValue = 0;
  else if (GuardValue == 13 && CloseReset == 0)
    GuardValue = 0;
}
