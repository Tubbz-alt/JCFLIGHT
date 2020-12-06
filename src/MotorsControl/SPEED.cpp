#include "SPEED.h"
#include "Common/VARIABLES.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

SPEEDCLASS SPEEDMOTORS;

void SPEEDCLASS::LoadEEPROM()
{
    if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == 0)
        MotorSpeed = 1000;
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == 1)
        MotorSpeed = 1050;
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == 2)
        MotorSpeed = 1100;
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == 3)
        MotorSpeed = 1150;
    else if (STORAGEMANAGER.Read_8Bits(MOTORSPEED_ADDR) == 4)
        MotorSpeed = 1200;
}