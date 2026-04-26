'''
run_identification.py 

重力参数辨识核心文件

功能：
1.提取样本csv文件中的数据 构建辨识矩阵 力矩向量
2.对辨识矩阵进行基参数分析 得到可辨识空间下的基参数 并最小二乘得到辨识向量
3.利用辨识向量预测重力力矩并与实际观测值进行对比分析.
4.导出辨识结果 包括数据、图片、报告等

本文件用于验证URDF模型能否大致解释真机数据 并且 在真机部署前分析样本数据可靠性 

实际部署在launch\fit_trig_gravity_model.py

'''


from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from urdf_import import urdf_path, urdf_import, build_q
from csv_dataset import load_gravity_samples_from_csv
from identify import build_identification_matrices, predict_gravity
from base_params import identify_base_params, reconstruct_full_params

from results_io import (
    create_result_dir,
    save_summary_json,
    save_identification_npz,
    save_sample_predictions_csv,
)
from report_plots import (
    save_singular_values_svg,
    save_sample_error_norms_svg,
    save_joint_mean_abs_error_svg,
)
from report_writer import write_identification_report

# 这个函数用于“回代验证”：
# 已知辨识得到的 pi_hat，再逐条样本预测重力力矩，
# 最后统计每条样本的误差和整体误差水平。
# @brief    根据辨识向量预测重力力矩，并和 csv 文件中的 tau 进行对比
# @retval   预测力矩、误差、误差范数、平均误差、最大误差
def evaluate_samples(model, data, samples, pi_hat):
    tau_preds = []
    errors = []
    error_norms = []

    for sample in samples:
        # 把样本中的 6 维物理关节角转换为 Pinocchio 的 q 向量
        q = build_q(model, sample.theta)
        # 用辨识结果预测当前姿态下的重力力矩
        tau_pred = predict_gravity(model, data, q, pi_hat)
        # 误差定义为“预测值 - 观测值”
        error = tau_pred - sample.tau
        error_norm = np.linalg.norm(error)

        tau_preds.append(tau_pred)
        errors.append(error)
        error_norms.append(error_norm)

    tau_preds = np.asarray(tau_preds, dtype=float)
    errors = np.asarray(errors, dtype=float)
    error_norms = np.asarray(error_norms, dtype=float)

    mean_error = np.mean(error_norms)
    max_error = np.max(error_norms)

    return tau_preds, errors, error_norms, mean_error, max_error

if __name__ == "__main__":
    # 脚本所在目录，用于组织结果输出
    script_dir = Path(__file__).resolve().parent
    # 数据集路径
    csv_path = PROJECT_ROOT / "data" / "mc02_capture_cleaned.csv"
    # 所有辨识结果统一保存到 results 目录
    results_root = PROJECT_ROOT / "results"
    # 当前运行名称，这里直接使用 CSV 文件名
    run_name = csv_path.stem
    # 导入 urdf
    model, data = urdf_import(urdf_path)
    # 提取数据样本
    samples = load_gravity_samples_from_csv(csv_path)
    # 构建回归矩阵和力矩观测向量
    A, b = build_identification_matrices(model, data, samples)
    # 基参数辨识
    alpha_hat, A_base, V_base, residuals, rank, rank_base, singular_values, s_all = identify_base_params(A, b)
    # 构建辨识向量
    pi_hat = reconstruct_full_params(V_base, alpha_hat)
    # 计算重力力矩观测和预测误差
    tau_preds, errors, error_norms, mean_error, max_error = evaluate_samples(model, data, samples, pi_hat)
    # 打印输出

    # 为本次运行创建独立结果目录，避免覆盖历史结果
    result_dir = create_result_dir(results_root, run_name)

    # 保存摘要信息：样本数、矩阵尺寸、秩、误差指标
    summary_path = save_summary_json(
        result_dir=result_dir,
        csv_path=csv_path,
        num_samples=len(samples),
        A_shape=A.shape,
        b_shape=b.shape,
        A_base_shape=A_base.shape,
        V_base_shape=V_base.shape,
        rank=rank,
        rank_base=rank_base,
        mean_error=mean_error,
        max_error=max_error,
    )

    # 保存辨识核心数组，后续可直接加载复现实验
    npz_path = save_identification_npz(
        result_dir=result_dir,
        A=A,
        b=b,
        A_base=A_base,
        V_base=V_base,
        alpha_hat=alpha_hat,
        pi_hat=pi_hat,
        residuals=residuals,
        singular_values=singular_values,
        s_all=s_all,
        error_norms=error_norms,
    )

    # 保存逐样本预测结果，便于后续误差分析和画图
    predictions_csv_path = save_sample_predictions_csv(
        result_dir=result_dir,
        samples=samples,
        tau_preds=tau_preds,
        errors=errors,
    )

    # 自动生成三张展示图
    singular_svg_path = save_singular_values_svg(result_dir, s_all)
    sample_error_svg_path = save_sample_error_norms_svg(result_dir, error_norms)
    joint_error_svg_path = save_joint_mean_abs_error_svg(result_dir, errors)

    # 自动生成 Markdown 报告，便于 Obsidian 和面试展示
    report_path = write_identification_report(
        result_dir=result_dir,
        csv_path=csv_path,
        num_samples=len(samples),
        A_shape=A.shape,
        b_shape=b.shape,
        A_base_shape=A_base.shape,
        V_base_shape=V_base.shape,
        rank=rank,
        rank_base=rank_base,
        mean_error=mean_error,
        max_error=max_error,
    )

    # 终端输出本次辨识的关键信息与结果文件路径
    print("csv_path =", csv_path)
    print("number of samples =", len(samples))
    print("A shape =", A.shape)
    print("b shape =", b.shape)
    print("A_base shape =", A_base.shape)
    print("V_base shape =", V_base.shape)
    print("rank =", rank)
    print("rank_base =", rank_base)
    print("alpha_hat shape =", alpha_hat.shape)
    print("pi_hat shape =", pi_hat.shape)
    print("residuals =", residuals)
    print("mean error =", mean_error)
    print("max error =", max_error)
    print("result_dir =", result_dir)
    print("summary_path =", summary_path)
    print("npz_path =", npz_path)
    print("predictions_csv_path =", predictions_csv_path)
    print("singular_svg_path =", singular_svg_path)
    print("sample_error_svg_path =", sample_error_svg_path)
    print("joint_error_svg_path =", joint_error_svg_path)
    print("report_path =", report_path)
