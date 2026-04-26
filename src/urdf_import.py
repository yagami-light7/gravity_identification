'''

功能：
1.导入urdf文件 创建模型和机器人数据
2.根据关节角度向量构建q向量

'''

import pinocchio as pin
from pathlib import Path
import math

urdf_path = Path(r"E:\RoboMaster\mec_arm\mec_arm_model\urdf\mec_arm.urdf")


# @brief    导入urdf文件并创建模型和数据
# @retval   模型和数据
def urdf_import(urdf_path):
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    return model, data


# @brief    由theata向量构建q向量
# @retval   q向量
def build_q(model, theta):
    q = pin.neutral(model)

    if len(theta) != 6:
        raise ValueError("theta must contain 6 joint angles")

    q[0] = theta[0]
    q[1] = theta[1]
    q[2] = theta[2]
    q[3] = theta[3]
    q[4] = theta[4]
    q[5] = theta[5]

    return q

