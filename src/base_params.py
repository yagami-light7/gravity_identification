'''
基于thin SVD 的截断子空间基参数辨识

功能：
1.对原始辨识矩阵A进行thin SVD剔除零空间 将问题转为低维列满秩的最小二乘问题A_base
2.对基参数辨识后的辨识矩阵进行最小二乘 求解出可辨识空间下的辨识投影向量alpha_hat
3.将辨识向量alpha_hat投影回原始参数空间，得到完整的辨识向量pi_hat

消除了原始零空间带来的不唯一性，并显著提升了辨识的数值稳定性
去掉零空间和近零空间分量后，辨识向量通常会更稳定，求解更简单，并且往往具有更好的测试集稳健性和泛化能力，但前提是奇异值截断阈值选择合理

'''



import numpy as np

# @brief    对力矩回归矩阵A进行SVD分析
# @retval   返回SVD分解结果和有效秩
def analyze_regressor(A, tol=1e-10):
    # thin SVD (full_matrices=False)
    U, s, Vt = np.linalg.svd(A, full_matrices=False)

    if len(s) == 0:
        rank = 0
    else:
        rank = np.sum(s > tol * s[0])

    return U, s, Vt, rank

# @brief    对右奇异向量矩阵V进行截断，构造基参数化
# @retval   返回基参数化结果
def build_base_parameterization(A, tol=1e-10):
    U, s, Vt, rank = analyze_regressor(A, tol)

    # A.shape = (6N, 60)
    # thin SVD 后：
    # U.shape  = (6N, 60)   假设 6N > 60
    # s.shape  = (60,)
    # Vt.shape = (60, 60)
    #
    # rank = r，为根据奇异值阈值得到的有效秩，r <= 60

    # 取前 r 个右奇异向量，构造可辨识参数子空间基
    # Vt[:rank, :].shape = (r, 60)
    # V_base.shape       = (60, r)
    V_base = Vt[:rank, :].T

    # 将原始回归矩阵投影到可辨识参数子空间
    # A.shape      = (6N, 60)
    # V_base.shape = (60, r)
    # A_base.shape = (6N, r)
    #
    # A_base 通常为列满秩矩阵，列数从 60 降为 r
    A_base = A @ V_base

    return A_base, V_base, s, rank

# @brief    识别基参数
# @retval   返回识别结果
def identify_base_params(A, b, tol=1e-10):
    A_base, V_base, s, rank = build_base_parameterization(A, tol)

    # 最小二乘法（伪逆）求辨识向量
    # A_base.shape = (6N, rank)列满秩/高矩阵 因此使用最小二乘（伪逆）
    alpha_hat, residuals, rank_base, singular_values = np.linalg.lstsq(A_base, b, rcond=None)

    return alpha_hat, A_base, V_base, residuals, rank, rank_base, singular_values, s

# @brief    利用截断后的基参数重构辨识向量
# @retval   辨识向量pi_hat
def reconstruct_full_params(V_base, alpha_hat):
    pi_hat = V_base @ alpha_hat
    return pi_hat