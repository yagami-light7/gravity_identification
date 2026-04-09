import numpy as np

"基于thin SVD 的截断子空间基参数辨识"
"去掉零空间和近零空间分量后，辨识向量通常会更稳定，求解更简单，并且往往具有更好的测试集稳健性和泛化能力，但前提是奇异值截断阈值选择合理"

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

    V_base = Vt[:rank, :].T #rank = 10

    A_base = A @ V_base # A_base.shape = (6N, rank)列满秩/高矩阵  A.shape = (6N, 10)  V_base.shape = (10, rank) 满秩 

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