#pragma once

#include "arm_math.h"

/*
 * Auto-generated trigonometric gravity model.
 * 当前版本建议优先部署 Joint2 / Joint3。
 */

#define MEC_ARM_GRAVITY_J1_TERM_NUM 3
static const float kMecArmGravityJ1Coeff[3] = {
    1.495815696e-03f,
    -1.211703605e-01f,
    -2.019208384e-01f,
};

/* Joint1 term order:
 * 0: 1
 * 1: sin(q1)
 * 2: cos(q1)
 */

static inline float MecArm_Gravity_J1(float q1)
{
    return
        kMecArmGravityJ1Coeff[0] * (1.0f) +
        kMecArmGravityJ1Coeff[1] * (arm_sin_f32(q1)) +
        kMecArmGravityJ1Coeff[2] * (arm_cos_f32(q1));
}

#define MEC_ARM_GRAVITY_J2_TERM_NUM 9
static const float kMecArmGravityJ2Coeff[9] = {
    3.855526049e+00f,
    -5.660498271e+00f,
    -9.476304467e+00f,
    6.952303730e-02f,
    6.330403998e+00f,
    7.890071533e-01f,
    1.538842550e+00f,
    -4.214075888e+00f,
    2.132532789e+00f,
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
    2.624880712e-01f,
    4.679072506e-01f,
    6.438723264e+00f,
    9.863051554e-02f,
    1.154954655e+00f,
    -5.510141352e-02f,
    -1.497708241e+00f,
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
    8.607648524e-03f,
    4.442293701e-01f,
    -2.652171705e-01f,
    5.260169873e-03f,
    -5.377623626e-02f,
    2.853556901e-01f,
    2.325774994e-01f,
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
    6.000370100e-01f,
    3.499588473e-01f,
    -3.349878833e-01f,
    1.866990266e-02f,
    4.108392234e-01f,
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
    5.511823838e-01f,
    7.307347555e-01f,
    -4.937291188e-01f,
};

/* Joint6 term order:
 * 0: 1
 * 1: sin(q6)
 * 2: cos(q6)
 */

static inline float MecArm_Gravity_J6(float q6)
{
    return
        kMecArmGravityJ6Coeff[0] * (1.0f) +
        kMecArmGravityJ6Coeff[1] * (arm_sin_f32(q6)) +
        kMecArmGravityJ6Coeff[2] * (arm_cos_f32(q6));
}
