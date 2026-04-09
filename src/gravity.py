import numpy as np
import pinocchio as pin

from urdf_import import urdf_import, build_q, urdf_path

# @brief    根据urdf文件计算当前姿态下的重力项(静止状态 dq=dqq=0)
#           不采用
# @retval   none
def compute_gravity(model, data, q):
    tau_g = pin.computeGeneralizedGravity(model, data, q)
    return tau_g


# @brief    得到静态下的力矩回归矩阵，此处不涉及科里奥利力和转动惯量，仅重力辨识
# @retval   力矩回归矩阵
def compute_gravity_regressor(model, data, q):  
    v = np.zeros(model.nv)
    a = np.zeros(model.nv)

    Y_g = pin.computeJointTorqueRegressor(model, data, q, v , a)
    
    return Y_g

# if __name__ == "__main__":
#     model, data = urdf_import(urdf_path)

#     theta = [0.3, 0.2, -0.1, 0.4, -0.2, 0.1]
#     q = build_q(model, theta)

#     tau_g = compute_gravity(model, data, q)
#     Y_g = compute_gravity_regressor(model, data, q)

#     print("q =", q)
#     print("tau_g =", tau_g)
#     print("tau_g shape =", tau_g.shape)
#     print("Y_g shape =", Y_g.shape)