#pragma once

#include "arm_math.h"

/*
 * Auto-generated trigonometric gravity model.
 * 当前版本建议优先部署 Joint2 / Joint3。
 */

#define MEC_ARM_GRAVITY_J1_TERM_NUM 3
static const float kMecArmGravityJ1Coeff[3] = {
    -7.302170328e+00f,
    1.832775284e+00f,
    7.594317486e+00f,
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
    -3.459610966e+00f,
    -7.386153972e+00f,
    -7.925290342e+00f,
    9.789500372e-01f,
    5.834226806e+00f,
    -6.605008118e-01f,
    4.531961929e+00f,
    -1.393155023e+00f,
    -1.831639281e+00f,
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
    -1.014722576e+00f,
    5.395405163e+00f,
    6.627182177e+00f,
    -4.765533565e+00f,
    2.103588395e-01f,
    2.030809616e+00f,
    -4.220304702e+00f,
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
    3.751123464e-01f,
    -1.034780588e+00f,
    -1.254464909e+00f,
    -7.875946335e-03f,
    -5.868534407e-02f,
    7.714070804e-01f,
    -1.317395119e+00f,
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
    9.959834525e-01f,
    -4.158235586e-01f,
    -1.237164671e-01f,
    2.676069209e-01f,
    -3.525406127e-01f,
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
    -5.078323746e+00f,
    2.813506161e+00f,
    4.939398096e+00f,
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
