#pragma once

#include <stdint.h>

typedef union
{
    float Vector[3];
    struct
    {
        float Roll;
        float Pitch;
        float Yaw;
    };
} Struct_Vector3x3;

static inline float VectorNormSquared(const Struct_Vector3x3 *Vector)
{
    return sq(Vector->Roll) + sq(Vector->Pitch) + sq(Vector->Yaw);
}

static inline Struct_Vector3x3 *VectorNormalize(Struct_Vector3x3 *Result,
                                                const Struct_Vector3x3 *Vector)
{
    float length = sqrtf(VectorNormSquared(Vector));
    if (length != 0)
    {
        Result->Roll = Vector->Roll / length;
        Result->Pitch = Vector->Pitch / length;
        Result->Yaw = Vector->Yaw / length;
    }
    else
    {
        Result->Roll = 0;
        Result->Pitch = 0;
        Result->Yaw = 0;
    }
    return Result;
}

static inline Struct_Vector3x3 *VectorCrossProduct(Struct_Vector3x3 *Result,
                                                   const Struct_Vector3x3 *a,
                                                   const Struct_Vector3x3 *b)
{
    Struct_Vector3x3 ab;
    ab.Roll = a->Pitch * b->Yaw - a->Yaw * b->Pitch;
    ab.Pitch = a->Yaw * b->Roll - a->Roll * b->Yaw;
    ab.Yaw = a->Roll * b->Pitch - a->Pitch * b->Roll;
    *Result = ab;
    return Result;
}

static inline Struct_Vector3x3 *VectorAdd(Struct_Vector3x3 *Result,
                                          const Struct_Vector3x3 *a,
                                          const Struct_Vector3x3 *b)
{
    Struct_Vector3x3 ab;
    ab.Roll = a->Roll + b->Roll;
    ab.Pitch = a->Pitch + b->Pitch;
    ab.Yaw = a->Yaw + b->Yaw;
    *Result = ab;
    return Result;
}

static inline Struct_Vector3x3 *VectorScale(Struct_Vector3x3 *Result,
                                            const Struct_Vector3x3 *a,
                                            const float b)
{
    Struct_Vector3x3 ab;
    ab.Roll = a->Roll * b;
    ab.Pitch = a->Pitch * b;
    ab.Yaw = a->Yaw * b;
    *Result = ab;
    return Result;
}