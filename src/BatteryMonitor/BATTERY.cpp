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

#include "BATTERY.h"
#include "AnalogDigitalConverter/ADC.h"
#include "Common/VARIABLES.h"
#include "Filters/AVERAGEFILTER.h"
#include "Scheduler/SCHEDULERTIME.h"
#include "Buzzer/BUZZER.h"

BATT BATTERY;

AverageFilterFloat_Size12 Voltage_Filter; //ISTANCIA DO FILTRO AVERAGE PARA A TENSÃO,TAMANHO = 12 ITERAÇÕES
AverageFilterFloat_Size12 Current_Filter; //ISTANCIA DO FILTRO AVERAGE PARA A CORRENTE,TAMANHO = 12 ITERAÇÕES

#define THIS_LOOP_RATE 100     //HZ
#define TIMER_TO_DETECT_BATT 3 //SEGUNDOS
#define POWER_MODULE_3DR       //TIPO DE POWER-MODULE USADO,JCFLIGHT ONBOARD OU 3DR

#ifndef POWER_MODULE_3DR
float BattVoltageFactor = 222.889f; //VALOR DE CALIBRAÇÃO PARA O DIVISOR RESISTIVO COM R1 DE 10K E R2 DE 1K
float Amps_Per_Volt = 17.0f;        //FATOR DE DIVISÃO DA TENSÃO DO PINO ANALOGICO PARA CONVERTER EM CORRENTE (AMPERES)
#else
float BattVoltageFactor = 237.489f; //VALOR DE CALIBRAÇÃO PARA O DIVISOR RESISTIVO COM R1 DE 13.7K E R2 DE 1.5K
float Amps_Per_Volt = 355;          //FATOR DE DIVISÃO DA TENSÃO DO PINO ANALOGICO PARA CONVERTER EM CORRENTE (AMPERES)
#endif

float Amps_OffSet = 0.00f; //TENSÃO DE OFFSET (AJUSTE FINO DA CORRENTE)

void BATT::Read_Voltage(void)
{
  //FILTRO COMPLEMENTAR PARA REDUÇÃO DE NOISE NA LEITURA DA TENSÃO (10 BITS ADC É TERRIVEL)
  Voltage = Voltage_Filter.Apply(Voltage * 0.92f + (float)(ADCPIN.Read(ADC0) / BattVoltageFactor));
  //TENSÃO DA BATERIA ACIMA DE 5V?SIM...
  if (Voltage > 5)
  {
    if (BATTERY.GetPercentage() < 20)
    { //MENOR QUE 20%
      LowBattPreventArm = true;
    }
    else
    {
      if (BEEPER.SafeToOthersBeepsCounter > 200)
        BEEPER.BeeperPlay(BEEPER_BAT_CRIT_LOW);
      LowBattPreventArm = false;
    }
  }
  else
  {
    LowBattPreventArm = false;
  }
}

uint8_t BATT::CalculatePercentage(float BattVoltage, float BattMinVolt, float BattMaxVolt)
{
  BATTERY.Percentage = (BattVoltage - BattMinVolt) / (BattMaxVolt - BattMinVolt);
  if (BATTERY.Percentage < 0)
    BATTERY.Percentage = 0;
  else if (BATTERY.Percentage > 1)
    BATTERY.Percentage = 1;
  return 100 * BATTERY.Percentage;
}

float BATT::AutoBatteryMin(float BattVoltage)
{
  if (BattVoltage > 5)
  {
    if (BattMinVoltageSelect == 1)
      return 10.8f;
    else if (BattMinVoltageSelect == 2)
      return 14.4f;
    else if (BattMinVoltageSelect == 3)
      return 21.6f;
    if (BattVoltage > 10.0f && BattVoltage < 13.0f)
    { //BATERIA 3S (3.6 x 3 = 10.8v)
      if (BattMinCount++ >= THIS_LOOP_RATE * TIMER_TO_DETECT_BATT)
        BattMinVoltageSelect = 1;
      return 10.8f;
    }
    else if (BattVoltage > 14.0f && BattVoltage < 17.0f)
    { //BATERIA 4S (3.6 x 4 = 14.4v)
      if (BattMinCount++ >= THIS_LOOP_RATE * TIMER_TO_DETECT_BATT)
        BattMinVoltageSelect = 2;
      return 14.4f;
    }
    else if (BattVoltage > 21.0f && BattVoltage < 26.0f)
    { //BATERIA 6S (3.6 x 6 = 21.6v)
      if (BattMinCount++ >= THIS_LOOP_RATE * TIMER_TO_DETECT_BATT)
        BattMinVoltageSelect = 3;
      return 21.6f;
    }
  }
  BattMinCount = 0;
  BattMinVoltageSelect = 0;
  return 0;
}

float BATT::AutoBatteryMax(float BattVoltage)
{
  if (BattVoltage > 5)
  {
    if (BattMaxVoltageSelect == 1)
      return 12.6f;
    else if (BattMaxVoltageSelect == 2)
      return 16.8f;
    else if (BattMaxVoltageSelect == 3)
      return 25.2f;
    if (BattVoltage > 10.0f && BattVoltage < 13.0f)
    { //BATERIA 3S (4.2 x 3 = 12.6v)
      if (BattMaxCount++ >= THIS_LOOP_RATE * TIMER_TO_DETECT_BATT)
        BattMaxVoltageSelect = 1;
      return 12.6f;
    }
    else if (BattVoltage > 14.0f && BattVoltage < 18.0f)
    { //BATERIA 4S (4.2 x 4 = 16.8v)
      if (BattMaxCount++ >= THIS_LOOP_RATE * TIMER_TO_DETECT_BATT)
        BattMaxVoltageSelect = 2;
      return 16.8f;
    }
    else if (BattVoltage > 21.0f && BattVoltage < 26.0f)
    { //BATERIA 6S (4.2 x 6 = 25.2v)
      if (BattMaxCount++ >= THIS_LOOP_RATE * TIMER_TO_DETECT_BATT)
        BattMaxVoltageSelect = 3;
      return 25.2f;
    }
  }
  BattMaxCount = 0;
  BattMaxVoltageSelect = 0;
  return 0;
}

uint8_t BATT::GetPercentage()
{
  return BATTERY.CalculatePercentage(Voltage, BATTERY.AutoBatteryMin(Voltage), BATTERY.AutoBatteryMax(Voltage));
}

void BATT::Do_RTH_With_Low_Batt(bool FailSafeBatt)
{
  //ENTRA EM MODO RTH OU LAND (SE ESTIVER PROXIMO DO HOME-POINT) SE A BATERIA ESTIVER COM A TENSÃO ABAIXO DE 20% DA CARGA TOTAL
  static bool FailSafeBattDetect = false;
  if (FailSafeBatt)
  {
    //EVITA COM QUE UMA TROCA RAPIDA DE TRUE PARA FALSE OCORRA,A FIM DE NÃO INTERFERIR NO FUNCIONAMENTO DESSA FUNÇÃO
    FailSafeBattDetect = true;
  }
  if (!COMMAND_ARM_DISARM)
  {
    FailSafeBattDetect = false;
    return;
  }
  if (!FailSafeBattDetect)
  {
    return;
  }
}

void BATT::Read_Current(void)
{
  Total_Current = 0;
  //FAZ A LEITURA DO SENSOR DE CORRENTE

  //ARDUPILOT
  //Total_Current = ((ADCPIN.Read(ADC1)) - Amps_OffSet) * Amps_Per_Volt;

  //INAV
  //int32_t ADC_mV = ((uint32_t)ADCPIN.Read(ADC1) * 5000 * 100) / 1024 * 10 - (int32_t)Amps_OffSet * 100;
  //Total_Current = Current_Filter.Apply((ADC_mV / Amps_Per_Volt) / 10);
}

void BATT::Calculate_Total_Mah(void)
{
  uint32_t TimeNow = AVRTIME.SchedulerMicros();
  static uint32_t Last_Time_Stored;
  float Delta_Time = TimeNow - Last_Time_Stored;
  if (Last_Time_Stored != 0 && Delta_Time < 2000000.0f)
  {
    //0.0002778 É 1/3600 (CONVERSÃO PRA HORAS)
    TotalCurrentInMah += Total_Current * Delta_Time * 0.0000002778f;
  }
  Last_Time_Stored = TimeNow;
}

float BATT::Get_Current_In_Mah()
{
  return (TotalCurrentInMah / 1000);
}

uint16_t BATT::GetWatts()
{
  return Total_Current * Voltage;
}