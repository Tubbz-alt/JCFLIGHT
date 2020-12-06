#include "LOOPS.h"
#include "Common/COMMON.h"

//FUNÇÕES QUE CONTÉM "WHILE" FORAM MOVIDAS PARA O LOOP DE 500HZ
//ANTES ELAS ESTAVAM NO LOOP INTEGRAL E,ESTAVAM CAUSANDO INSTABILIDADE NAS FUNÇÕES MICROS E MILLIS
#define I2C_AND_SERIAL_500HZ

void Slow_Loop()
{
        static Scheduler_Struct SlowLoop;
        if (SchedulerTimer(&SlowLoop, 100000)) //10HZ
        {

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(SLOW_LOOP);
#endif

                Pre_Arm();
                CurvesRC_Update();
                AUXFLIGHT.LoadEEPROM();
                RTH_Altitude_EEPROM();
                IMU_Filters_Update();
                UpdateValuesOfPID();
                GCS.UpdateParametersToGCS();
                PushWayPointParameters();

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif
        }
}

void Medium_Loop()
{
        static Scheduler_Struct MediumLoop;
        if (SchedulerTimer(&MediumLoop, 20000)) //50HZ
        {

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(MEDIUM_LOOP);
#endif

                DecodeAllReceiverChannels();
                RCCONFIG.Set_Pulse();
                RCCONFIG.Update_Channels();
                Desarm_LowThrottle();
                FailSafeCheck();
                RCSticks_Update();
                AUXFLIGHT.SelectMode();
                AUXFLIGHT.FlightModesAuxSelect();
                FlightModesUpdate();
                PrintlnParameters();
                Trim_Servo_Update();
                Auto_Throttle_Flight_Mode(SetFlightModes[ALTITUDEHOLD_MODE]);

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif
        }
}

void Fast_Medium_Loop()
{
        static Scheduler_Struct FastMediumLoop;
        if (SchedulerTimer(&FastMediumLoop, 10000)) //100HZ
        {

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(FAST_MEDIUM_LOOP);
#endif

                BEEPER.Run();
                BATTERY.Read_Voltage();
                BATTERY.Read_Current();
                Pre_Arm_Leds();
                GimbalControll();
                CrashCheck();
                PARACHUTE.Manual_Detect_Channel();
                PARACHUTE.Manual_Do_Now();
                FlipModeRun();
                WayPointRun();
                GCS.Serial_Parse_Protocol();

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif
        }
}

void Fast_Loop()
{
        static Scheduler_Struct FastLoop;
        if (SchedulerTimer(&FastLoop, 2000)) //500HZ
        {

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringStartTime(FAST_LOOP);
#endif

                Update_PrecisionLand();
#ifdef I2C_AND_SERIAL_500HZ
                SBUS_Update();
                IBUS_Update();
                Acc_ReadBufferData();
                Gyro_ReadBufferData();
                COMPASS.Constant_Read();
                Barometer_Update();
                GPS_Serial_Read();
                AHRS_Update();
#endif

#ifdef ENABLE_TIMEMONITOR
                AVRTIMEMONITOR.MeasuringFinishTime();
#endif
        }
}

void Total_Loop()
{
#ifdef ENABLE_TIMEMONITOR
        AVRTIMEMONITOR.MeasuringStartTime(TOTAL_LOOP);
#endif

        RGB.Update();
#ifndef I2C_AND_SERIAL_500HZ
        SBUS_Update();
        IBUS_Update();
        Acc_ReadBufferData();
        Gyro_ReadBufferData();
        GPS_Serial_Read();
        COMPASS.Constant_Read();
        Barometer_Update();
#endif
        DynamicPID();
        Auto_Launch_Update();
        GPS_Compute();
        CalculateAccelerationXYZ();
        INS_Calculate_AccelerationZ();
        CalculateXY_INS();
        AirSpeed_Update();
        YawManipulationUpdate();
        Apply_Controll_For_Throttle();
        GPS_Orientation_Update();
        PID_Time();
        PID_Update();
        PID_Reset_Accumulators();
        PIDMixMotors();
        Servo_Rate_Adjust();
        ApplyPWMInAllComponents();
        Switch_Flag();
        BATTERY.Calculate_Total_Mah();

#ifdef ENABLE_TIMEMONITOR
        AVRTIMEMONITOR.MeasuringFinishTime();
#endif
}