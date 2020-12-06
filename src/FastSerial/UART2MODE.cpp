#include "UART2MODE.h"
#include "StorageManager/EEPROMSTORAGE.h"
#include "BAR/BAR.h"

void UART2Mode_Initialization()
{
    DDRA |= (1 << DDD0); //DEFINE A PORTA DIGITAL 22 COMO SAIDA
    if (STORAGEMANAGER.Read_8Bits(UART2_ADDR) != 1)
        PORTA |= 1 << 0; //ATIVA OS TRASISTORES DE CORTE E O DE BY-PASS
    else
        PORTA &= ~(1 << 0); //PREPARA OS TRANSISTORES PARA O MODO SBUS
}