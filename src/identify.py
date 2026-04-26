'''
重力辨识模块

功能：
1.通过样本数据构造 力矩回归矩阵Y_g 和 力矩向量 并拼接为 辨识矩阵A 
2.根据辨识矩阵A和力矩向量b 使用最小二乘得到辨识向量pi_hat
3.根据辨识向量pi_hat和力矩回归矩阵 得到重力补偿力矩预测

补充：
1.本文件未完成基向量辨识 为全参数辨识
2.最终实际部署时需先利用base_params.py进行基参数辨识 剔除低奇异值方向后 最小二乘得到辨识向量pi_hat
3.identify_gravity_params()实际部署时被identify_base_params()代替

'''

import numpy as np

from dataset import GravitySample
from gravity import pin_compute_gravity_regressor
from urdf_import import build_q

# @brief    通过样本数据构建辨识矩阵和向量
# @retval   返回辨识矩阵和向量
def build_identification_matrices(model, data, samples):    
    # 长度检查
    if len(samples) == 0:
        raise ValueError("No samples provided for identification")
    
    A_blocks = []
    b_blocks = []

    for sample in samples:
        # 类型检查
        if not isinstance(sample, GravitySample):
            raise TypeError("All samples must be instances of GravitySample")
    
        # 构造q向量
        q = build_q(model, sample.theta)

        # 计算静态下的力矩回归矩阵
        Y_g = pin_compute_gravity_regressor(model, data, q)

        # 构造待辨识矩阵/向量
        A_blocks.append(Y_g)
        b_blocks.append(sample.tau)
    
    # 按行拼接 A.shape = (6N, 60) b.shape = (6N,)
    A = np.vstack(A_blocks)
    b = np.concatenate(b_blocks)

    return A, b 


# @brief    辨识重力参数
#           全参数辨识，已经被base_params基参数辨识替代            
# @retval   pi_hat参数估计向量, residuals残差平方和, rank秩, singular_values奇异值
def identify_gravity_params(model, data, samples):
    # 构造辨识矩阵和向量
    A, b = build_identification_matrices(model, data, samples)

    # 求最小二乘解 pi_hat = arg min ||A*pi - b||^2
    pi_hat, residuals, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)

    # 欠定/秩亏时 residuals常常无输出
    return pi_hat, residuals, rank, singular_values


# @brief    计算重力补偿力矩向量
# @retval   重力补偿力矩向量tau_pred
def predict_gravity(model, data, q, pi_hat):
    # 构造静态力矩回归矩阵
    Y_g = pin_compute_gravity_regressor(model, data, q)

    # 计算预测的重力项
    tau_pred = Y_g @ pi_hat

    return tau_pred
