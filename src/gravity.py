'''
利用Pinocchio接口计算重力项

功能：
1. 根据urdf文件计算当前姿态下的重力项 用于真值模型验证
2. 计算静态下的力矩回归矩阵Y_g

'''

import numpy as np
import pinocchio as pin

from urdf_import import urdf_import, build_q, urdf_path


# @brief    根据urdf文件计算当前姿态下的重力项(静止状态 dq=dqq=0)
#           pinocchio真值模型 仅用作demo_synthetic.py中验证辨识方案
# @retval   none
def pin_compute_gravity(model, data, q):
    tau_g = pin.computeGeneralizedGravity(model, data, q)
    return tau_g


# @brief    得到静态下的力矩回归矩阵，此处不涉及科里奥利力和转动惯量，仅重力辨识
# @retval   力矩回归矩阵Y_gravity
def pin_compute_gravity_regressor(model, data, q):  
    v = np.zeros(model.nv)
    a = np.zeros(model.nv)

    Y_g = pin.computeJointTorqueRegressor(model, data, q, v , a)
    
    return Y_g
