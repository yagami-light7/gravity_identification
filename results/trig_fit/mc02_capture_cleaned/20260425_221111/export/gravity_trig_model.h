#pragma once

#include "arm_math.h"

/*
 * Auto-generated trigonometric gravity model.
 * 当前版本建议优先部署 Joint2 / Joint3。
 */

#define MEC_ARM_GRAVITY_J2_TERM_NUM 11
static const float kMecArmGravityJ2Coeff[11] = {
    1.120454103e+00f,
    4.523385130e+00f,
    4.577314390e+00f,
    1.301573246e+01f,
    -1.044797376e+01f,
    -2.380461773e+01f,
    5.175598430e-01f,
    9.102121994e+00f,
    -9.457237637e-02f,
    2.418170185e+00f,
    7.954793466e+00f,
};

/* Joint2 term order:
 * 0: 1
 * 1: sin(q2)
 * 2: cos(q2)
 * 3: sin(q3)
 * 4: cos(q3)
 * 5: sin(q2+q3)
 * 6: cos(q2+q3)
 * 7: sin(q2+q3+q4)
 * 8: cos(q2+q3+q4)
 * 9: sin(q2+q3+q4+q5)
 * 10: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J2(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ2Coeff[0] * (1.0f) +
        kMecArmGravityJ2Coeff[1] * (arm_sin_f32(q2)) +
        kMecArmGravityJ2Coeff[2] * (arm_cos_f32(q2)) +
        kMecArmGravityJ2Coeff[3] * (arm_sin_f32(q3)) +
        kMecArmGravityJ2Coeff[4] * (arm_cos_f32(q3)) +
        kMecArmGravityJ2Coeff[5] * (arm_sin_f32(q2 + q3)) +
        kMecArmGravityJ2Coeff[6] * (arm_cos_f32(q2 + q3)) +
        kMecArmGravityJ2Coeff[7] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ2Coeff[8] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ2Coeff[9] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ2Coeff[10] * (arm_cos_f32(q2 + q3 + q4 + q5));
}

#define MEC_ARM_GRAVITY_J3_TERM_NUM 11
static const float kMecArmGravityJ3Coeff[11] = {
    1.683098117e+00f,
    4.980901171e+00f,
    -1.367514007e+00f,
    -6.855180761e-01f,
    -4.494974829e+00f,
    -9.510927508e+00f,
    1.320087625e+01f,
    6.427907957e+00f,
    -1.609102728e+00f,
    3.753344994e+00f,
    3.774618979e+00f,
};

/* Joint3 term order:
 * 0: 1
 * 1: sin(q2)
 * 2: cos(q2)
 * 3: sin(q3)
 * 4: cos(q3)
 * 5: sin(q2+q3)
 * 6: cos(q2+q3)
 * 7: sin(q2+q3+q4)
 * 8: cos(q2+q3+q4)
 * 9: sin(q2+q3+q4+q5)
 * 10: cos(q2+q3+q4+q5)
 */

static inline float MecArm_Gravity_J3(float q2, float q3, float q4, float q5)
{
    return
        kMecArmGravityJ3Coeff[0] * (1.0f) +
        kMecArmGravityJ3Coeff[1] * (arm_sin_f32(q2)) +
        kMecArmGravityJ3Coeff[2] * (arm_cos_f32(q2)) +
        kMecArmGravityJ3Coeff[3] * (arm_sin_f32(q3)) +
        kMecArmGravityJ3Coeff[4] * (arm_cos_f32(q3)) +
        kMecArmGravityJ3Coeff[5] * (arm_sin_f32(q2 + q3)) +
        kMecArmGravityJ3Coeff[6] * (arm_cos_f32(q2 + q3)) +
        kMecArmGravityJ3Coeff[7] * (arm_sin_f32(q2 + q3 + q4)) +
        kMecArmGravityJ3Coeff[8] * (arm_cos_f32(q2 + q3 + q4)) +
        kMecArmGravityJ3Coeff[9] * (arm_sin_f32(q2 + q3 + q4 + q5)) +
        kMecArmGravityJ3Coeff[10] * (arm_cos_f32(q2 + q3 + q4 + q5));
}
