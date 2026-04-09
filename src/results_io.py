from pathlib import Path
from datetime import datetime
import json
import csv
import numpy as np


# @brief    创建一次辨识运行的结果目录
#           目录结构固定为 data / figures / report，便于后续管理和展示
# @retval   返回本次运行的根结果目录
def create_result_dir(base_dir, run_name):
    base_dir = Path(base_dir)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = base_dir / run_name / timestamp
    (result_dir / "data").mkdir(parents=True, exist_ok=True)
    (result_dir / "figures").mkdir(parents=True, exist_ok=True)
    (result_dir / "report").mkdir(parents=True, exist_ok=True)
    return result_dir


# @brief    保存本次辨识的摘要信息
#           主要保存矩阵尺寸、秩信息和整体误差，便于快速查看
# @retval   返回 summary.json 路径
def save_summary_json(
    result_dir,
    csv_path,
    num_samples,
    A_shape,
    b_shape,
    A_base_shape,
    V_base_shape,
    rank,
    rank_base,
    mean_error,
    max_error,
):
    summary = {
        "csv_path": str(csv_path),
        "num_samples": int(num_samples),
        "A_shape": list(A_shape),
        "b_shape": list(b_shape),
        "A_base_shape": list(A_base_shape),
        "V_base_shape": list(V_base_shape),
        "rank": int(rank),
        "rank_base": int(rank_base),
        "mean_error": float(mean_error),
        "max_error": float(max_error),
    }

    summary_path = Path(result_dir) / "data" / "summary.json"

    # ensure_ascii=False 用于直接保留中文，避免写成 Unicode 转义
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    return summary_path


# @brief    保存辨识过程中的核心数值结果
#           便于后续复现实验、再次分析或重新画图
# @retval   返回 identification_result.npz 路径
def save_identification_npz(
    result_dir,
    A,
    b,
    A_base,
    V_base,
    alpha_hat,
    pi_hat,
    residuals,
    singular_values,
    s_all,
    error_norms,
):
    npz_path = Path(result_dir) / "data" / "identification_result.npz"

    np.savez(
        npz_path,
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

    return npz_path


# @brief    保存逐样本预测结果
#           每一行同时记录观测力矩、预测力矩、逐关节误差和误差范数
# @retval   返回 sample_predictions.csv 路径
def save_sample_predictions_csv(result_dir, samples, tau_preds, errors):
    csv_path = Path(result_dir) / "data" / "sample_predictions.csv"

    header = [
        "sample_id",
        "theta1", "theta2", "theta3", "theta4", "theta5", "theta6",
        "tau_meas1", "tau_meas2", "tau_meas3", "tau_meas4", "tau_meas5", "tau_meas6",
        "tau_pred1", "tau_pred2", "tau_pred3", "tau_pred4", "tau_pred5", "tau_pred6",
        "err1", "err2", "err3", "err4", "err5", "err6",
        "error_norm",
    ]

    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for i, sample in enumerate(samples):
            # 每条样本单独计算一次整体误差范数，方便后续直接排序或画图
            error_norm = np.linalg.norm(errors[i])

            row = (
                [i]
                + sample.theta.tolist()
                + sample.tau.tolist()
                + tau_preds[i].tolist()
                + errors[i].tolist()
                + [float(error_norm)]
            )

            writer.writerow(row)

    return csv_path
