'''
demo_synthetic.py

对 pinocchio真值模型 进行辨识 以验证链路是否完善

功能：
1.使用 URDF 模型制作数据集
2.利用数据集构造 力矩回归矩阵A 和 力矩向量b
3.对力矩回归矩阵 A 进行基参数辨识得到 A_base 再利用力矩向量 b 最小二乘得到辨识向量 pi_hat
4.利用测试集分别计算真实力矩（pinocchio真值）和预测力矩（pi_hat辨识结果）

若二者差距较小 可证实当前系统辨识方案可行

'''

from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from dataset import GravitySample
from gravity import pin_compute_gravity
from identify import build_identification_matrices, predict_gravity
from base_params import build_base_parameterization, identify_base_params, reconstruct_full_params
from urdf_import import build_q, urdf_import, urdf_path

# @brief    制造数据集
# @retval   返回合成样本列表
def make_synthetic_samples(model, data):
    # 手动编写角度列表
    theta_list = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.2, -0.1, 0.3, 0.1, -0.2, 0.2],
        [-0.3, 0.4, -0.2, 0.5, 0.1, -0.4],
        [0.5, 0.2, -0.4, -0.3, 0.3, 0.1],
        [-0.2, -0.3, 0.2, 0.4, -0.1, 0.5],
    ]

    samples = []

    # 制作数据样本（部署时需手动采集）
    for theta in theta_list:
        # 构造q向量
        q = build_q(model, theta)
        # 计算力矩（urdf前向）
        tau = pin_compute_gravity(model, data, q)
        # 构造样本
        sample = GravitySample(theta=theta, tau=tau)
        samples.append(sample)

    return samples

if __name__ == "__main__":
    model, data = urdf_import(urdf_path)    # 利用urdf构造模型和数据

    samples = make_synthetic_samples(model, data)   # 制作数据集

    A,b = build_identification_matrices(model, data, samples)   # 构造待辨识矩阵和向量（回归矩阵和力矩观测向量）

    A_base, V_base, s, rank = build_base_parameterization(A, tol=1e-10) #构造基参数

    alpha_hat, A_base, V_base, residuals, rank, rank_base, singular_values, s_all = identify_base_params(A, b, tol=1e-10) #及参数辨识

    pi_hat = reconstruct_full_params(V_base, alpha_hat) # 返回辨识向量

    print("number of samples =", len(samples))
    print("A shape =", A.shape)
    print("b shape =", b.shape)
    print("A_base shape =", A_base.shape)
    print("V_base shape =", V_base.shape)
    print("alpha_hat shape =", alpha_hat.shape)
    print("pi_hat shape =", pi_hat.shape)
    print("rank =", rank)
    print("rank_base =", rank_base)
    print("residuals =", residuals)

    # 测试集
    theta_test = [0.3, 0.1, -0.2, 0.2, -0.1, 0.4]
    q_test = build_q(model, theta_test)

    tau_true = pin_compute_gravity(model, data, q_test)
    tau_pred = predict_gravity(model, data, q_test, pi_hat)

    print("tau_true =", tau_true)
    print("tau_pred =", tau_pred)
    print("prediction error norm =", np.linalg.norm(tau_true - tau_pred))
