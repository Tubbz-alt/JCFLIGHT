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

#include "AUXFLIGHT.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "Common/VARIABLES.h"
#include "WayPointNavigation/WAYPOINT.h"
#include "SETFLIGHTMODE.h"
#include "BAR/BAR.h"

//***********************************************************
//CONFIGURAÇÃO DAS CHAVES AUXILIARES PARA OS MODOS DE VOO
//***********************************************************

AUXFLIGHTCLASS AUXFLIGHT;

uint8_t GPSHoldConfig,
    RTHConfig,
    IOCConfig,
    AltitudeHoldConfig,
    AcroConfig,
    SportConfig,
    AutoFlipConfig,
    AutoPilotConfig,
    FlightMode,
    ReceiverModel,
    ArmDisarmConfig,
    AutoLandConfig,
    ParachuteDetectTrigger;
int16_t AltitudeHoldControlAux,
    GPSHoldControlAux,
    RTHControlAux,
    IOCControlAux,
    GimbalControlAux,
    AcroControlAux,
    SportControlAux,
    AutoFlipControlAux,
    AutoPilotControlAux,
    ArmDisarmControlAux,
    AutoLandControlAux;

void AUXFLIGHTCLASS::LoadEEPROM(void)
{
  IOCConfig = STORAGEMANAGER.Read_8Bits(IOC_ADDR);                    //CHAVE AUX ATRIBUIDA PARA O MODO IOC
  AltitudeHoldConfig = STORAGEMANAGER.Read_8Bits(ALT_HOLD_ADDR);      //CHAVE AUX ATRIBUIDA PARA O MODO ALT-HOLD
  GPSHoldConfig = STORAGEMANAGER.Read_8Bits(GPS_HOLD_ADDR);           //CHAVE AUX ATRIBUIDA PARA O MODO GPS-HOLD
  RTHConfig = STORAGEMANAGER.Read_8Bits(RTH_ADDR);                    //CHAVE AUX ATRIBUIDA PARA O MODO RTH
  AcroConfig = STORAGEMANAGER.Read_8Bits(STABLIZE_ADDR);              //CHAVE AUX ATRIBUIDA PARA O MODO ACRO
  SportConfig = STORAGEMANAGER.Read_8Bits(ATACK_ADDR);                //CHAVE AUX ATRIBUIDA PARA O MODO SPORT
  ParachuteDetectTrigger = STORAGEMANAGER.Read_8Bits(PARACHUTE_ADDR); //CONFIGURAÇÃO DO PARACHUTE
  AutoFlipConfig = STORAGEMANAGER.Read_8Bits(AUTOFLIP_ADDR);          //CHAVE AUX ATRIBUIDA PARA O MODO AUTO-FLIP
  GimbalControlAux = STORAGEMANAGER.Read_8Bits(GIMBAL_ADDR);          //CANAL AUX ATRIBUIDO PARA O CONTROLE DO GIMBAL
  if (FrameType != STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR))
  {
    FrameType = STORAGEMANAGER.Read_8Bits(FRAMETYPE_ADDR); //TIPO DE FRAME SELECIONADO
  }
  if (ReceiverModel != STORAGEMANAGER.Read_8Bits(RECEIVER_ADDR))
  {
    ReceiverModel = STORAGEMANAGER.Read_8Bits(RECEIVER_ADDR); //MODELO DO RADIO
  }
  ArmDisarmConfig = STORAGEMANAGER.Read_8Bits(ARMDISARM_ADDR);   //CHAVE ATRIBUIDA AO ARMDISARM VIA CHAVE AUX
  AutoPilotConfig = STORAGEMANAGER.Read_8Bits(AUTOMISSION_ADDR); //CHAVE ATRIBUIDA AO MODO WAYPOINT
  AutoLandConfig = STORAGEMANAGER.Read_8Bits(AUTOLAND_ADDR);     //CHAVE ATRIBUIDA AO AUTO LAND
}

void AUXFLIGHTCLASS::SelectMode(void)
{
  //INTELLIGENT ORIENTATION CONTROL
  switch (IOCConfig)
  {

  case AUXONELOW:
    IOCControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    IOCControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    IOCControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    IOCControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    IOCControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    IOCControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    IOCControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    IOCControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    IOCControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    IOCControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    IOCControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    IOCControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    IOCControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    IOCControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    IOCControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    IOCControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    IOCControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    IOCControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    IOCControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    IOCControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    IOCControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    IOCControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    IOCControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    IOCControlAux = AUX8_HIGH;
    break;
  }

  //ALT-HOLD
  switch (AltitudeHoldConfig)
  {

  case AUXONELOW:
    AltitudeHoldControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    AltitudeHoldControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    AltitudeHoldControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    AltitudeHoldControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    AltitudeHoldControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    AltitudeHoldControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    AltitudeHoldControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    AltitudeHoldControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    AltitudeHoldControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    AltitudeHoldControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    AltitudeHoldControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    AltitudeHoldControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    AltitudeHoldControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    AltitudeHoldControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    AltitudeHoldControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    AltitudeHoldControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    AltitudeHoldControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    AltitudeHoldControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    AltitudeHoldControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    AltitudeHoldControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    AltitudeHoldControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    AltitudeHoldControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    AltitudeHoldControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    AltitudeHoldControlAux = AUX8_HIGH;
    break;
  }

  //GPS-HOLD
  switch (GPSHoldConfig)
  {

  case AUXONELOW:
    GPSHoldControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    GPSHoldControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    GPSHoldControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    GPSHoldControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    GPSHoldControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    GPSHoldControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    GPSHoldControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    GPSHoldControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    GPSHoldControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    GPSHoldControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    GPSHoldControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    GPSHoldControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    GPSHoldControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    GPSHoldControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    GPSHoldControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    GPSHoldControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    GPSHoldControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    GPSHoldControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    GPSHoldControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    GPSHoldControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    GPSHoldControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    GPSHoldControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    GPSHoldControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    GPSHoldControlAux = AUX8_HIGH;
    break;
  }

  //RETURN TO HOME
  switch (RTHConfig)
  {

  case AUXONELOW:
    RTHControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    RTHControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    RTHControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    RTHControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    RTHControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    RTHControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    RTHControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    RTHControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    RTHControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    RTHControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    RTHControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    RTHControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    RTHControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    RTHControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    RTHControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    RTHControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    RTHControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    RTHControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    RTHControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    RTHControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    RTHControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    RTHControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    RTHControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    RTHControlAux = AUX8_HIGH;
    break;
  }

  //ACRO
  switch (AcroConfig)
  {

  case AUXONELOW:
    AcroControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    AcroControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    AcroControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    AcroControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    AcroControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    AcroControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    AcroControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    AcroControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    AcroControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    AcroControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    AcroControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    AcroControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    AcroControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    AcroControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    AcroControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    AcroControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    AcroControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    AcroControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    AcroControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    AcroControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    AcroControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    AcroControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    AcroControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    AcroControlAux = AUX8_HIGH;
    break;
  }

  //SPORT
  switch (SportConfig)
  {

  case AUXONELOW:
    SportControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    SportControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    SportControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    SportControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    SportControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    SportControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    SportControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    SportControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    SportControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    SportControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    SportControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    SportControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    SportControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    SportControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    SportControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    SportControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    SportControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    SportControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    SportControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    SportControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    SportControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    SportControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    SportControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    SportControlAux = AUX8_HIGH;
    break;
  }

  //AUTOFLIP
  switch (AutoFlipConfig)
  {

  case AUXONELOW:
    AutoFlipControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    AutoFlipControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    AutoFlipControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    AutoFlipControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    AutoFlipControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    AutoFlipControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    AutoFlipControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    AutoFlipControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    AutoFlipControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    AutoFlipControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    AutoFlipControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    AutoFlipControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    AutoFlipControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    AutoFlipControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    AutoFlipControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    AutoFlipControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    AutoFlipControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    AutoFlipControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    AutoFlipControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    AutoFlipControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    AutoFlipControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    AutoFlipControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    AutoFlipControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    AutoFlipControlAux = AUX8_HIGH;
    break;
  }

  //AUTO
  switch (AutoPilotConfig)
  {

  case AUXONELOW:
    AutoPilotControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    AutoPilotControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    AutoPilotControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    AutoPilotControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    AutoPilotControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    AutoPilotControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    AutoPilotControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    AutoPilotControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    AutoPilotControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    AutoPilotControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    AutoPilotControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    AutoPilotControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    AutoPilotControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    AutoPilotControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    AutoPilotControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    AutoPilotControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    AutoPilotControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    AutoPilotControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    AutoPilotControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    AutoPilotControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    AutoPilotControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    AutoPilotControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    AutoPilotControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    AutoPilotControlAux = AUX8_HIGH;
    break;
  }

  //ARMDISARM
  switch (ArmDisarmConfig)
  {

  case AUXONELOW:
    ArmDisarmControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    ArmDisarmControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    ArmDisarmControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    ArmDisarmControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    ArmDisarmControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    ArmDisarmControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    ArmDisarmControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    ArmDisarmControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    ArmDisarmControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    ArmDisarmControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    ArmDisarmControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    ArmDisarmControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    ArmDisarmControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    ArmDisarmControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    ArmDisarmControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    ArmDisarmControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    ArmDisarmControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    ArmDisarmControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    ArmDisarmControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    ArmDisarmControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    ArmDisarmControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    ArmDisarmControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    ArmDisarmControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    ArmDisarmControlAux = AUX8_HIGH;
    break;
  }

  //AUTO LAND
  switch (AutoLandConfig)
  {

  case AUXONELOW:
    AutoLandControlAux = AUX1_LOW;
    break;

  case AUXONEMIDDLE:
    AutoLandControlAux = AUX1_MID;
    break;

  case AUXONEHIGH:
    AutoLandControlAux = AUX1_HIGH;
    break;

  case AUXTWOLOW:
    AutoLandControlAux = AUX2_LOW;
    break;

  case AUXTWOMIDDLE:
    AutoLandControlAux = AUX2_MID;
    break;

  case AUXTWOHIGH:
    AutoLandControlAux = AUX2_HIGH;
    break;

  case AUXTHREELOW:
    AutoLandControlAux = AUX3_LOW;
    break;

  case AUXTHREEMIDDLE:
    AutoLandControlAux = AUX3_MID;
    break;

  case AUXTHREEHIGH:
    AutoLandControlAux = AUX3_HIGH;
    break;

  case AUXFOURLOW:
    AutoLandControlAux = AUX4_LOW;
    break;

  case AUXFOURMIDDLE:
    AutoLandControlAux = AUX4_MID;
    break;

  case AUXFOURHIGH:
    AutoLandControlAux = AUX4_HIGH;
    break;

  case AUXFIVELOW:
    AutoLandControlAux = AUX5_LOW;
    break;

  case AUXFIVEMIDDLE:
    AutoLandControlAux = AUX5_MID;
    break;

  case AUXFIVEHIGH:
    AutoLandControlAux = AUX5_HIGH;
    break;

  case AUXSIXLOW:
    AutoLandControlAux = AUX6_LOW;
    break;

  case AUXSIXMIDDLE:
    AutoLandControlAux = AUX6_MID;
    break;

  case AUXSIXHIGH:
    AutoLandControlAux = AUX6_HIGH;
    break;

  case AUXSEVENLOW:
    AutoLandControlAux = AUX7_LOW;
    break;

  case AUXSEVENMIDDLE:
    AutoLandControlAux = AUX7_MID;
    break;

  case AUXSEVENHIGH:
    AutoLandControlAux = AUX7_HIGH;
    break;

  case AUXEIGHTLOW:
    AutoLandControlAux = AUX8_LOW;
    break;

  case AUXEIGHTMIDDLE:
    AutoLandControlAux = AUX8_MID;
    break;

  case AUXEIGHTHIGH:
    AutoLandControlAux = AUX8_HIGH;
    break;
  }
}

void AUXFLIGHTCLASS::FlightModesAuxSelect(void)
{
  SetFlightModeToGCS();
  if (Fail_Safe_Event)
    return;
  //ACRO
  if (AcroControlAux)
    SetFlightModes[STABILIZE_MODE] = 0;
  else
    SetFlightModes[STABILIZE_MODE] = 1;
  //IOC
  if (IOCControlAux)
    SetFlightModes[IOC_MODE] = 1;
  else
    SetFlightModes[IOC_MODE] = 0;
  //ALTITUDE-HOLD
  if (AltitudeHoldControlAux)
    SetFlightModes[ALTITUDEHOLD_MODE] = 1;
  else
    SetFlightModes[ALTITUDEHOLD_MODE] = 0;
  //GPS-HOLD
  if (GPSHoldControlAux)
    SetFlightModes[GPSHOLD_MODE] = 1;
  else
    SetFlightModes[GPSHOLD_MODE] = 0;
  //AUTO LAND
  if (AutoLandControlAux)
    SetFlightModes[LAND_MODE] = 1;
  else
    SetFlightModes[LAND_MODE] = 0;
  //RTH
  if (RTHControlAux)
    SetFlightModes[RTH_MODE] = 1;
  else
    SetFlightModes[RTH_MODE] = 0;
  //SPORT
  if (SportControlAux)
    SetFlightModes[ATACK_MODE] = 1;
  else
    SetFlightModes[ATACK_MODE] = 0;
  //AUTO-FLIP
  if (AutoFlipControlAux)
    Flip_Mode = true;
  else
    Flip_Mode = false;
  //AUTO
  if (AutoPilotControlAux)
    Do_WayPoint = true;
  else
    Do_WayPoint = false;

  //Throttle_LockUp(true); //TRAVA DO ACELERADOR PARA TER FACILIDADE DE MOVER NO YAW
}
