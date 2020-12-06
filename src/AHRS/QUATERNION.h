#pragma once

#include <stdint.h>
#include "VECTOR.h"

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Struct_Quaternion;

static inline Struct_Quaternion *QuaternionInitUnit(Struct_Quaternion *Result)
{
    Result->q0 = 1.0f;
    Result->q1 = 0.0f;
    Result->q2 = 0.0f;
    Result->q3 = 0.0f;
    return Result;
}

static inline float QuaternionNormalizedSquared(const Struct_Quaternion *Quaternion)
{
    return sq(Quaternion->q0) + sq(Quaternion->q1) + sq(Quaternion->q2) + sq(Quaternion->q3);
}

static inline Struct_Quaternion *QuaternionNormalize(Struct_Quaternion *Result,
                                                     const Struct_Quaternion *Quaternion)
{
    float CheckSquare = sqrtf(QuaternionNormalizedSquared(Quaternion));
    if (CheckSquare < 1e-6f)
    {
        Result->q0 = 1;
        Result->q1 = 0;
        Result->q2 = 0;
        Result->q3 = 0;
    }
    else
    {
        Result->q0 = Quaternion->q0 / CheckSquare;
        Result->q1 = Quaternion->q1 / CheckSquare;
        Result->q2 = Quaternion->q2 / CheckSquare;
        Result->q3 = Quaternion->q3 / CheckSquare;
    }
    return Result;
}

static inline Struct_Quaternion *QuaternionMultiply(Struct_Quaternion *Result,
                                                    const Struct_Quaternion *a,
                                                    const Struct_Quaternion *b)
{
    Struct_Quaternion p;
    p.q0 = a->q0 * b->q0 - a->q1 * b->q1 - a->q2 * b->q2 - a->q3 * b->q3;
    p.q1 = a->q0 * b->q1 + a->q1 * b->q0 + a->q2 * b->q3 - a->q3 * b->q2;
    p.q2 = a->q0 * b->q2 - a->q1 * b->q3 + a->q2 * b->q0 + a->q3 * b->q1;
    p.q3 = a->q0 * b->q3 + a->q1 * b->q2 - a->q2 * b->q1 + a->q3 * b->q0;
    *Result = p;
    return Result;
}

static inline Struct_Quaternion *QuaternionConjugate(Struct_Quaternion *Result,
                                                     const Struct_Quaternion *Quaternion)
{
    Result->q0 = Quaternion->q0;
    Result->q1 = -Quaternion->q1;
    Result->q2 = -Quaternion->q2;
    Result->q3 = -Quaternion->q3;
    return Result;
}

static inline Struct_Vector3x3 *QuaternionRotateVector(Struct_Vector3x3 *Result,
                                                       const Struct_Vector3x3 *Vector,
                                                       const Struct_Quaternion *Reference)
{
    Struct_Quaternion QuaternionVector,
        ReferenceConjugate;
    QuaternionVector.q0 = 0;
    QuaternionVector.q1 = Vector->Roll;
    QuaternionVector.q2 = Vector->Pitch;
    QuaternionVector.q3 = Vector->Yaw;
    QuaternionConjugate(&ReferenceConjugate, Reference);
    QuaternionMultiply(&QuaternionVector, &ReferenceConjugate, &QuaternionVector);
    QuaternionMultiply(&QuaternionVector, &QuaternionVector, Reference);
    Result->Roll = QuaternionVector.q1;
    Result->Pitch = QuaternionVector.q2;
    Result->Yaw = QuaternionVector.q3;
    return Result;
}

static inline Struct_Vector3x3 *QuaternionRotateVectorInv(Struct_Vector3x3 *Result,
                                                          const Struct_Vector3x3 *Vector,
                                                          const Struct_Quaternion *Reference)
{
    Struct_Quaternion QuaternionVector,
        ReferenceConjugate;
    QuaternionVector.q0 = 0;
    QuaternionVector.q1 = Vector->Roll;
    QuaternionVector.q2 = Vector->Pitch;
    QuaternionVector.q3 = Vector->Yaw;
    QuaternionConjugate(&ReferenceConjugate, Reference);
    QuaternionMultiply(&QuaternionVector, Reference, &QuaternionVector);
    QuaternionMultiply(&QuaternionVector, &QuaternionVector, &ReferenceConjugate);
    Result->Roll = QuaternionVector.q1;
    Result->Pitch = QuaternionVector.q2;
    Result->Yaw = QuaternionVector.q3;
    return Result;
}

static inline Struct_Quaternion *QuaternionInitFromVector(Struct_Quaternion *Result,
                                                          const Struct_Vector3x3 *v)
{
    Result->q0 = 0.0f;
    Result->q1 = v->Roll;
    Result->q2 = v->Pitch;
    Result->q3 = v->Yaw;
    return Result;
}

static inline Struct_Quaternion *QuaternionScale(Struct_Quaternion *Result,
                                                 const Struct_Quaternion *a,
                                                 const float b)
{
    Struct_Quaternion p;
    p.q0 = a->q0 * b;
    p.q1 = a->q1 * b;
    p.q2 = a->q2 * b;
    p.q3 = a->q3 * b;
    *Result = p;
    return Result;
}