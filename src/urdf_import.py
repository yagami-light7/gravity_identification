import pinocchio as pin
from pathlib import Path
import math

urdf_path = Path(r"E:\RoboMaster\mec_arm\rm_arm_2025_last\urdf\rm_arm_2025_last.urdf")


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

    # J1 J4 J6 continuous
    q[0] = math.cos(theta[0])
    q[1] = math.sin(theta[0])

    q[2] = theta[1]

    q[3] = theta[2]

    q[4] = math.cos(theta[3])
    q[5] = math.sin(theta[3])

    q[6] = theta[4]

    q[7] = math.cos(theta[5])
    q[8] = math.sin(theta[5])

    return q

