#pragma once

#include "arm_math.h"

/*
 * Auto-generated trigonometric gravity model.
 * 当前版本建议优先部署 Joint2 / Joint3。
 */

#define MEC_ARM_GRAVITY_J1_TERM_NUM 3
static const float kMecArmGravityJ1Coeff[3] = {
    4.148014439e+00f,
    -7.189343360e-01f,
    -4.140839585e+00f,
};

/* Joint1 term order:
 * 0: 1
 * 1: sin(q1)
 * 2: cos(q1)
 */

static inline float MecArm_Gravity_J1(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ1Coeff[0] * (1.0f) +
        kMecArmGravityJ1Coeff[1] * (arm_sin_f32(q1)) +
        kMecArmGravityJ1Coeff[2] * (arm_cos_f32(q1));
}

#define MEC_ARM_GRAVITY_J2_TERM_NUM 9
static const float kMecArmGravityJ2Coeff[9] = {
    5.184500723e+00f,
    -7.220903413e+00f,
    -9.491881042e+00f,
    2.326655681e+00f,
    1.056691469e+01f,
    -3.037735440e+00f,
    -7.383647714e+00f,
    -1.880101716e+00f,
    1.427937856e+00f,
};

/* Joint2 term order:
 * 0: 1
 * 1: sin(q2)
 * 2: cos(q2)
 * 3: sin(q2+q3)
 * 4: cos(q2+q3)
 * 5: sin(q2+q3+q4)
 * 6: cos(q2+q3+q4)
 * 7: sin(q2+q3+q4+q5)
 * 8: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J2(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ2Coeff[0] * (1.0f) +
        kMecArmGravityJ2Coeff[1] * (arm_sin_f32(q2)) +
        kMecArmGravityJ2Coeff[2] * (arm_cos_f32(q2)) +
        kMecArmGravityJ2Coeff[3] * (arm_sin_f32(q2 + q3)) +
        kMecArmGravityJ2Coeff[4] * (arm_cos_f32(q2 + q3)) +
        kMecArmGravityJ2Coeff[5] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ2Coeff[6] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ2Coeff[7] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ2Coeff[8] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J3_TERM_NUM 7
static const float kMecArmGravityJ3Coeff[7] = {
    -1.452858038e+00f,
    9.353567262e+00f,
    7.786619106e+00f,
    -7.708371599e+00f,
    1.742749409e+00f,
    -1.525626037e+00f,
    -7.611308696e+00f,
};

/* Joint3 term order:
 * 0: 1
 * 1: sin(q2+q3)
 * 2: cos(q2+q3)
 * 3: sin(q2+q3+q4)
 * 4: cos(q2+q3+q4)
 * 5: sin(q2+q3+q4+q5)
 * 6: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J3(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ3Coeff[0] * (1.0f) +
        kMecArmGravityJ3Coeff[1] * (arm_sin_f32(q2 + q3)) +
        kMecArmGravityJ3Coeff[2] * (arm_cos_f32(q2 + q3)) +
        kMecArmGravityJ3Coeff[3] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ3Coeff[4] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ3Coeff[5] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ3Coeff[6] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J4_TERM_NUM 7
static const float kMecArmGravityJ4Coeff[7] = {
    8.332342156e-02f,
    1.250488933e+00f,
    -1.267955741e+00f,
    5.269305893e-02f,
    8.663269262e-02f,
    1.258076598e+00f,
    9.696598305e-01f,
};

/* Joint4 term order:
 * 0: 1
 * 1: sin(q4)
 * 2: cos(q4)
 * 3: sin(q2+q3+q4)
 * 4: cos(q2+q3+q4)
 * 5: sin(q2+q3+q4+q5)
 * 6: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J4(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ4Coeff[0] * (1.0f) +
        kMecArmGravityJ4Coeff[1] * (arm_sin_f32(q4)) +
        kMecArmGravityJ4Coeff[2] * (arm_cos_f32(q4)) +
        kMecArmGravityJ4Coeff[3] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ4Coeff[4] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ4Coeff[5] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ4Coeff[6] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J5_TERM_NUM 5
static const float kMecArmGravityJ5Coeff[5] = {
    3.714552748e-01f,
    -1.746693416e-01f,
    -1.426566670e-01f,
    7.257087090e-01f,
    -2.640566329e-01f,
};

/* Joint5 term order:
 * 0: 1
 * 1: sin(q5)
 * 2: cos(q5)
 * 3: sin(q2+q3+q4+q5)
 * 4: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J5(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ5Coeff[0] * (1.0f) +
        kMecArmGravityJ5Coeff[1] * (arm_sin_f32(q5)) +
        kMecArmGravityJ5Coeff[2] * (arm_cos_f32(q5)) +
        kMecArmGravityJ5Coeff[3] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ5Coeff[4] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J6_TERM_NUM 3
static const float kMecArmGravityJ6Coeff[3] = {
    -1.134763806e+01f,
    -8.360055488e-01f,
    1.135885408e+01f,
};

/* Joint6 term order:
 * 0: 1
 * 1: sin(q6)
 * 2: cos(q6)
 */

static inline float MecArm_Gravity_J6(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ6Coeff[0] * (1.0f) +
        kMecArmGravityJ6Coeff[1] * (arm_sin_f32(q6)) +
        kMecArmGravityJ6Coeff[2] * (arm_cos_f32(q6));
}
