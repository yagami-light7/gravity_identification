'''
fit_trig_gravity_model.py

将清洗后的准静态 q/tau 数据进一步拟合为 MCU 侧易部署的轻量重力补偿函数

对每个关节，用符合机械臂重力结构的三角基函数构造线性回归模型，再用带 Ridge 正则的最小二乘求系数，得到 MCU 可直接运行的轻量重力补偿函数。

功能：
1. 提取样本csv文件中的数据 使用基于三角函数的基函数 进行最小二乘拟合
2. 拟合完成之后生成信息、报告以及用于MCU的头文件

在run_identification.py完成辨识之后 如果报告误差Mean_Absolute_Error较小再运行此文件
如果Mean_Absolute_Error误差较大 请考虑重新采集数据（是否远离关节限位 是否做到准静态）

运行后重点关注的指标：
    - MAE      : 平均绝对误差
    - RMSE     : 均方根误差
    - MaxAbs   : 最大绝对误差
    - MAE Ratio: 平均误差相对于平均力矩幅值的比例
    - Corr     : 预测趋势和真实趋势的一致程度

    （以我运行生成的Joint2结果为例）：

    - MAE: `0.622510`
    - RMSE: `0.824332`
    - MaxAbs: `3.429673`
    - MAE Ratio = MAE / mean(|tau|): `0.11822872452546547`
    - Corr: `0.9913223238290636`

平均误差0.6NM 方均根误差0.82NM 最大样本误差3.43NM
平均样本偏差11.8% 拟合的相关系数到达了99% 

'''

"""


核心数学形式：
    对某个关节 j，选择若干个三角基函数 phi_k(q)，建立线性模型：

        tau_j(q) ≈ sum_k c_{j,k} * phi_k(q)

    这里：
        - q 是 6 维关节角向量
        - tau_j 是第 j 个关节的真实重力力矩标签
        - c_{j,k} 是待求系数

    由于模型对系数 c 是线性的，因此可以写成标准线性回归问题：

        y ≈ Phi * c

    其中：
        - y   : 所有样本的真实力矩标签向量
        - Phi : 设计矩阵，每一列是一种基函数在所有样本上的取值
        - c   : 待拟合系数向量

Ridge 正则化：
    为了防止某些系数被局部脏数据拉得过大，导致 MCU 上出现力矩尖峰，
    本脚本在普通最小二乘基础上加入 Ridge 正则项：

        min ||Phi*c - y||^2 + lambda * ||c||^2

    实际实现时：
        - 常数项不参与正则惩罚
        - 三角项参与正则惩罚

    这样做的工程意义是：
        - 降低局部姿态补偿过冲
        - 提高泛化能力
        - 让部署到 MCU 的前馈模型更加保守和稳定


"""


from pathlib import Path
from datetime import datetime
import csv
import json
import sys

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DATA_CSV = PROJECT_ROOT / "data" / "mc02_capture_cleaned.csv"
RESULTS_ROOT = PROJECT_ROOT / "results" / "trig_fit"

# 允许对 6 个关节分别拟合
FIT_JOINTS = [1, 2, 3, 4, 5, 6]

# 每 5 个样本取 1 个做测试集
TEST_STRIDE = 5

# Ridge 正则化系数。
# 值越大，系数越保守，模型越不容易在局部姿态出现很大的力矩尖峰
RIDGE_LAMBDA = 1.0e-2

# 分关节定义更接近物理结构的三角基函数

JOINT_TERMS = {
    # Joint1 理论上纯重力项应较小，这里只保留偏置和自身一阶谐波。
    1: [
        "1",
        "sin(q1)",
        "cos(q1)",
    ],

    # Joint2 主要由整条后续链条对其产生重力矩。
    2: [
        "1",
        "sin(q2)",
        "cos(q2)",
        "sin(q2+q3)",
        "cos(q2+q3)",
        "sin(q2+q3+q4)",
        "cos(q2+q3+q4)",
        "sin(q2+q3+q4+q5)",
        "cos(q2+q3+q4+q5)",
    ],

    # Joint3 用从 Joint3 起往后的累计角来描述。
    3: [
        "1",
        "sin(q2+q3)",
        "cos(q2+q3)",
        "sin(q2+q3+q4)",
        "cos(q2+q3+q4)",
        "sin(q2+q3+q4+q5)",
        "cos(q2+q3+q4+q5)",
    ],
    # Joint4 为腕部近端关节，保留自身姿态和累计腕部姿态项。
    4: [
        "1",
        "sin(q4)",
        "cos(q4)",
        "sin(q2+q3+q4)",
        "cos(q2+q3+q4)",
        "sin(q2+q3+q4+q5)",
        "cos(q2+q3+q4+q5)",
    ],
    # Joint5 主要受自身和最终俯仰累计角影响。
    5: [
        "1",
        "sin(q5)",
        "cos(q5)",
        "sin(q2+q3+q4+q5)",
        "cos(q2+q3+q4+q5)",
    ],
    # Joint6 理论纯重力项也应较小，先用最保守的一阶谐波拟合。
    6: [
        "1",
        "sin(q6)",
        "cos(q6)",
    ],
}


def load_clean_csv(csv_path: Path):
    """
    读取清洗后的辨识 CSV，返回 theta/tau 数组。

    输入 CSV 至少需要包含：
        theta1 ~ theta6
        tau1   ~ tau6

    返回：
        theta : shape = (N, 6)
        tau   : shape = (N, 6)
    """
    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        theta_rows = []
        tau_rows = []

        for row in reader:
            theta_rows.append([float(row[f"theta{i}"]) for i in range(1, 7)])
            tau_rows.append([float(row[f"tau{i}"]) for i in range(1, 7)])

    theta = np.asarray(theta_rows, dtype=float)
    tau = np.asarray(tau_rows, dtype=float)

    if theta.shape[0] == 0:
        raise ValueError(f"CSV contains no samples: {csv_path}")

    return theta, tau


def split_train_test(num_samples: int, test_stride: int):
    """
    按固定步长划分训练集和测试集。

    例如 test_stride = 5 时：
        第 0、5、10、15... 条样本进入测试集，
        其余样本进入训练集。

    这样做的优点：
        - 不需要完全随机打乱数据
        - 能保留连续采样数据的一部分时间结构
        - 仍然可以评估模型对未参与拟合样本的泛化能力
    """
    indices = np.arange(num_samples, dtype=int)
    test_mask = (indices % test_stride) == 0
    train_mask = ~test_mask
    return train_mask, test_mask


def parse_sum_expression(sum_expr: str, q: np.ndarray) -> float:
    """
    将形如 q2+q3+q4 的表达式解析为角度和。

    例如：
        "q2+q3+q4" -> q[1] + q[2] + q[3]

    注意：
        表达式中的 q1~q6 使用 1-based 关节编号，
        而 numpy 数组 q 使用 0-based 存储。
    """
    angle = 0.0
    parts = [part.strip() for part in sum_expr.split("+")]

    for part in parts:
        if not part.startswith("q"):
            raise ValueError(f"Unsupported angle token: {part}")

        joint_id = int(part[1:])
        if joint_id < 1 or joint_id > 6:
            raise ValueError(f"Joint index out of range in term: {part}")

        angle += q[joint_id - 1]

    return angle


def term_value(term: str, q: np.ndarray) -> float:
    """
    计算单个基函数项在姿态 q 下的数值。
    """
    if term == "1":
        return 1.0

    if term.startswith("sin(") and term.endswith(")"):
        return np.sin(parse_sum_expression(term[4:-1], q))

    if term.startswith("cos(") and term.endswith(")"):
        return np.cos(parse_sum_expression(term[4:-1], q))

    raise ValueError(f"Unsupported basis term: {term}")


def build_design_matrix(theta: np.ndarray, terms: list[str]) -> np.ndarray:
    """
    构造设计矩阵 Phi。

    数学上：
        若共有 N 条样本、m 个基函数，则
            Phi 的维度为 (N, m)

    其中：
        - 第 i 行：第 i 条样本姿态下，各个基函数的取值
        - 第 j 列：第 j 个基函数在所有样本上的取值
    """
    phi = np.zeros((theta.shape[0], len(terms)), dtype=float)

    for row_idx, q in enumerate(theta):
        for col_idx, term in enumerate(terms):
            phi[row_idx, col_idx] = term_value(term, q)

    return phi


def build_ridge_penalty(terms: list[str]) -> np.ndarray:
    """
    构造 Ridge 正则项对应的对角惩罚向量。

    返回一个一维 penalty，
    后续通过 np.diag(penalty) 转成对角矩阵。

    """
    penalty = np.ones(len(terms), dtype=float)

    for idx, term in enumerate(terms):
        if term == "1":
            penalty[idx] = 0.0

    return penalty


def fit_linear_model(phi: np.ndarray, y: np.ndarray, terms: list[str], ridge_lambda: float) -> np.ndarray:
    """
    带 Ridge 正则化的线性最小二乘拟合。

    当 ridge_lambda <= 0 时：
        退化为普通最小二乘
            min ||Phi*c - y||^2

    当 ridge_lambda > 0 时：
        求解 Ridge 回归
            min ||Phi*c - y||^2 + lambda * c^T D c

        对应正规方程：
            (Phi^T Phi + lambda * D) c = Phi^T y

    其中：
        D 是对角惩罚矩阵，由 build_ridge_penalty(...) 给出。
    """
    if ridge_lambda <= 0.0:
        coeff, _, _, _ = np.linalg.lstsq(phi, y, rcond=None)
        return coeff

    penalty = build_ridge_penalty(terms)
    normal_mat = phi.T @ phi + ridge_lambda * np.diag(penalty)
    rhs = phi.T @ y
    return np.linalg.solve(normal_mat, rhs)


def evaluate_model(phi: np.ndarray, y: np.ndarray, coeff: np.ndarray):
    """
    计算拟合误差指标。

    返回值含义：
        y_pred          : 模型预测值
        err             : 逐样本误差
        mae             : 平均绝对误差
        rmse            : 均方根误差
        max_abs         : 最大绝对误差
        mean_abs_target : 真实力矩平均绝对值
        mae_ratio       : 平均误差占真实力矩平均幅值的比例
        corr            : 预测值与真实值的相关系数
    """
    y_pred = phi @ coeff
    err = y_pred - y

    mae = float(np.mean(np.abs(err)))
    rmse = float(np.sqrt(np.mean(err ** 2)))
    max_abs = float(np.max(np.abs(err)))
    mean_abs_target = float(np.mean(np.abs(y)))
    corr = None
    if np.std(y) > 1.0e-12 and np.std(y_pred) > 1.0e-12:
        corr = float(np.corrcoef(y, y_pred)[0, 1])

    return {
        "y_pred": y_pred,
        "err": err,
        "mae": mae,
        "rmse": rmse,
        "max_abs": max_abs,
        "mean_abs_target": mean_abs_target,
        "mae_ratio": (mae / mean_abs_target) if mean_abs_target > 1.0e-12 else None,
        "corr": corr,
    }


def create_result_dir(results_root: Path, run_name: str) -> Path:
    """按时间戳创建结果目录，避免覆盖历史结果。"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    result_dir = results_root / run_name / timestamp
    (result_dir / "data").mkdir(parents=True, exist_ok=True)
    (result_dir / "export").mkdir(parents=True, exist_ok=True)
    (result_dir / "report").mkdir(parents=True, exist_ok=True)
    return result_dir


def c_expr_for_term(term: str) -> str:
    """
    将 Python 侧的基函数项转换为 MCU 端 C 表达式。

    例如：
        sin(q2+q3) -> arm_sin_f32(q2 + q3)
        cos(q4)    -> arm_cos_f32(q4)

    这里使用 CMSIS DSP 的 arm_sin_f32 / arm_cos_f32，
    使导出的头文件可以直接用于嵌入式工程。
    """
    if term == "1":
        return "1.0f"

    if term.startswith("sin(") and term.endswith(")"):
        expr = term[4:-1].replace("+", " + ")
        return f"arm_sin_f32({expr})"

    if term.startswith("cos(") and term.endswith(")"):
        expr = term[4:-1].replace("+", " + ")
        return f"arm_cos_f32({expr})"

    raise ValueError(f"Unsupported term: {term}")


def extract_required_joint_ids(terms: list[str]) -> list[int]:
    """
    从基函数项中提取导出函数真正需要的关节变量。

    不同关节的模型使用的变量不完全一样，例如：
        - J2 可能需要 q2, q3, q4, q5
        - J6 可能只需要 q6

    这个函数用于自动生成正确的 C 函数参数列表，
    避免出现“函数内部用了 q1，但签名里没有 q1”的问题。
    """
    joint_ids = set()

    for term in terms:
        if term == "1":
            continue

        if not ((term.startswith("sin(") or term.startswith("cos(")) and term.endswith(")")):
            raise ValueError(f"Unsupported term while extracting joint ids: {term}")

        expr = term[4:-1]
        parts = [part.strip() for part in expr.split("+")]
        for part in parts:
            if not part.startswith("q"):
                raise ValueError(f"Unsupported angle token while extracting joint ids: {part}")
            joint_id = int(part[1:])
            if joint_id < 1 or joint_id > 6:
                raise ValueError(f"Joint index out of range while extracting joint ids: {part}")
            joint_ids.add(joint_id)

    return sorted(joint_ids)


def write_export_header(export_path: Path, fit_result: dict):
    """
    导出 MCU 可直接参考的头文件。

    导出的内容包括：
        1. 每个关节的系数数组
        2. 每个关节的基函数顺序注释
        3. 每个关节的静态 inline 计算函数

    注意：
        导出使用的是 coeff_full，
        即“全量样本重新拟合后的最终系数”，而不是 coeff_train。
    """
    lines = []

    lines.append("#pragma once")
    lines.append("")
    lines.append('#include "arm_math.h"')
    lines.append("")
    lines.append("/*")
    lines.append(" * Auto-generated trigonometric gravity model.")
    lines.append(" */")
    lines.append("")

    for joint_id in FIT_JOINTS:
        terms = fit_result[f"joint{joint_id}"]["terms"]
        coeff = fit_result[f"joint{joint_id}"]["coeff_full"]
        required_joint_ids = extract_required_joint_ids(terms)

        # 自动根据基函数项中实际出现的变量，拼出函数签名。
        signature_args = ", ".join([f"float q{joint_idx}" for joint_idx in required_joint_ids])
        if not signature_args:
            signature_args = "void"

        lines.append(f"#define MEC_ARM_GRAVITY_J{joint_id}_TERM_NUM {len(terms)}")
        lines.append(f"static const float kMecArmGravityJ{joint_id}Coeff[{len(terms)}] = {{")
        for value in coeff:
            lines.append(f"    {value:.9e}f,")
        lines.append("};")
        lines.append("")
        lines.append(f"/* Joint{joint_id} term order:")
        for idx, term in enumerate(terms):
            lines.append(f" * {idx}: {term}")
        lines.append(" */")
        lines.append("")
        lines.append(f"static inline float MecArm_Gravity_J{joint_id}({signature_args})")
        lines.append("{")
        lines.append("    return")
        for idx, term in enumerate(terms):
            expr = c_expr_for_term(term)
            suffix = ";" if idx == len(terms) - 1 else ""
            lines.append(f"        kMecArmGravityJ{joint_id}Coeff[{idx}] * ({expr}){suffix}")
            if idx != len(terms) - 1:
                lines[-1] += " +"
        lines.append("}")
        lines.append("")

    export_path.write_text("\n".join(lines), encoding="utf-8")


def write_predictions_csv(csv_path: Path, theta: np.ndarray, tau: np.ndarray, fit_result: dict):
    """
    导出逐样本预测结果，便于后续分析误差分布。

    这份 CSV 可用于：
        - 找出误差特别大的姿态点
        - 分析某个关节在哪些工作区拟合较差
        - 检查是否存在局部尖峰、局部符号异常
    """
    with csv_path.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.writer(csv_file)

        header = ["sample_id"]
        for i in range(1, 7):
            header.append(f"theta{i}")
        for joint_id in FIT_JOINTS:
            header += [
                f"tau_meas{joint_id}",
                f"tau_pred{joint_id}",
                f"err{joint_id}",
            ]
        writer.writerow(header)

        for idx in range(theta.shape[0]):
            row = [idx, *theta[idx].tolist()]
            for joint_id in FIT_JOINTS:
                pred = fit_result[f"joint{joint_id}"]["y_pred_full"][idx]
                meas = tau[idx, joint_id - 1]
                err = pred - meas
                row += [meas, pred, err]
            writer.writerow(row)


def write_report(report_path: Path, csv_path: Path, num_samples: int, fit_result: dict):
    """
    导出简要 Markdown 报告。

    报告用于人工快速阅读，重点展示：
        - 输入数据来源
        - 样本数
        - Ridge 系数
        - 每个关节使用的基函数
        - 每个关节的训练/测试误差
    """
    lines = []
    lines.append("# 三角基函数重力拟合报告")
    lines.append("")
    lines.append(f"- 输入 CSV: `{csv_path}`")
    lines.append(f"- 样本数量: `{num_samples}`")
    lines.append(f"- 测试集抽样步长: `{TEST_STRIDE}`")
    lines.append(f"- Ridge 正则系数: `{RIDGE_LAMBDA}`")
    lines.append("")

    for joint_id in FIT_JOINTS:
        joint_key = f"joint{joint_id}"
        joint_result = fit_result[joint_key]
        train = joint_result["train_metrics"]
        test = joint_result["test_metrics"]

        lines.append(f"## Joint{joint_id}")
        lines.append("")
        lines.append("### Basis Terms")
        for term in joint_result["terms"]:
            lines.append(f"- `{term}`")
        lines.append("")
        lines.append("### Train Metrics")
        lines.append(f"- MAE: `{train['mae']:.6f}`")
        lines.append(f"- RMSE: `{train['rmse']:.6f}`")
        lines.append(f"- MaxAbs: `{train['max_abs']:.6f}`")
        lines.append(f"- Corr: `{train['corr']}`")
        lines.append("")
        lines.append("### Test Metrics")
        lines.append(f"- MAE: `{test['mae']:.6f}`")
        lines.append(f"- RMSE: `{test['rmse']:.6f}`")
        lines.append(f"- MaxAbs: `{test['max_abs']:.6f}`")
        lines.append(f"- MAE Ratio = MAE / mean(|tau|): `{test['mae_ratio']}`")
        lines.append(f"- Corr: `{test['corr']}`")
        lines.append("")

    report_path.write_text("\n".join(lines), encoding="utf-8")


def main():
    # 1. 读取清洗后的准静态样本。
    #    theta 的形状为 (N, 6)
    #    tau   的形状为 (N, 6)
    theta, tau = load_clean_csv(DATA_CSV)

    # 2. 按固定步长划分训练集和测试集。
    #    训练集用于拟合参数，测试集用于评估泛化效果。
    train_mask, test_mask = split_train_test(theta.shape[0], TEST_STRIDE)

    # 3. 为本次运行创建独立的结果目录，便于保存历史结果。
    result_dir = create_result_dir(RESULTS_ROOT, DATA_CSV.stem)

    fit_result = {}

    for joint_id in FIT_JOINTS:
        # 4. 针对每个关节单独拟合：
        #    - terms 是该关节专属的三角基函数集合
        #    - phi   是设计矩阵
        #    - y     是该关节的真实力矩标签
        terms = JOINT_TERMS[joint_id]       # 基函数
        phi = build_design_matrix(theta, terms) # 设计矩阵
        y = tau[:, joint_id - 1]    # 从theta中选择对应关节力矩向量

        # 5. 划分训练/测试子集。
        phi_train = phi[train_mask]
        phi_test = phi[test_mask]
        y_train = y[train_mask]
        y_test = y[test_mask]

        # 6. 先只在训练集上拟合参数。
        #    这个 coeff_train 只用于评估测试集，不用于最终部署导出。
        coeff_train = fit_linear_model(phi_train, y_train, terms, RIDGE_LAMBDA)
        train_metrics = evaluate_model(phi_train, y_train, coeff_train)
        test_metrics = evaluate_model(phi_test, y_test, coeff_train)

        # 7. 为 MCU 部署，再使用全量样本重新拟合一遍。
        #    这样最终导出的系数利用了全部数据，通常更稳定。
        coeff_full = fit_linear_model(phi, y, terms, RIDGE_LAMBDA)
        full_metrics = evaluate_model(phi, y, coeff_full)

        fit_result[f"joint{joint_id}"] = {
            "terms": terms,
            "coeff_train": coeff_train.tolist(),
            "coeff_full": coeff_full.tolist(),
            "train_metrics": {
                "mae": train_metrics["mae"],
                "rmse": train_metrics["rmse"],
                "max_abs": train_metrics["max_abs"],
                "mae_ratio": train_metrics["mae_ratio"],
                "corr": train_metrics["corr"],
            },
            "test_metrics": {
                "mae": test_metrics["mae"],
                "rmse": test_metrics["rmse"],
                "max_abs": test_metrics["max_abs"],
                "mae_ratio": test_metrics["mae_ratio"],
                "corr": test_metrics["corr"],
            },
            "full_metrics": {
                "mae": full_metrics["mae"],
                "rmse": full_metrics["rmse"],
                "max_abs": full_metrics["max_abs"],
                "mae_ratio": full_metrics["mae_ratio"],
                "corr": full_metrics["corr"],
            },
            "y_pred_full": full_metrics["y_pred"],
        }

    # 8. 导出 JSON 摘要，便于程序读取和人工查看。
    summary = {
        "csv_path": str(DATA_CSV),
        "num_samples": int(theta.shape[0]),
        "test_stride": TEST_STRIDE,
        "ridge_lambda": RIDGE_LAMBDA,
        "fit_joints": FIT_JOINTS,
        "joint_results": {
            f"joint{joint_id}": {
                "terms": fit_result[f"joint{joint_id}"]["terms"],
                "coeff_full": fit_result[f"joint{joint_id}"]["coeff_full"],
                "train_metrics": fit_result[f"joint{joint_id}"]["train_metrics"],
                "test_metrics": fit_result[f"joint{joint_id}"]["test_metrics"],
                "full_metrics": fit_result[f"joint{joint_id}"]["full_metrics"],
            }
            for joint_id in FIT_JOINTS
        },
    }

    summary_path = result_dir / "data" / "fit_summary.json"
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding="utf-8")

    # 9. 导出逐样本预测结果，便于进一步做误差定位。
    predictions_path = result_dir / "data" / "fit_predictions.csv"
    write_predictions_csv(predictions_path, theta, tau, fit_result)

    # 10. 导出 MCU 头文件。这是最终真正用于下位机部署的结果。
    export_header_path = result_dir / "export" / "gravity_trig_model.h"
    write_export_header(export_header_path, fit_result)

    # 11. 导出 Markdown 报告，方便你直接查看每个关节的指标。
    report_path = result_dir / "report" / "report.md"
    write_report(report_path, DATA_CSV, theta.shape[0], fit_result)

    print("csv_path =", DATA_CSV)
    print("num_samples =", theta.shape[0])
    print("ridge_lambda =", RIDGE_LAMBDA)
    print("result_dir =", result_dir)
    print("summary_path =", summary_path)
    print("predictions_path =", predictions_path)
    print("export_header_path =", export_header_path)
    print("report_path =", report_path)
    for joint_id in FIT_JOINTS:
        metrics = fit_result[f"joint{joint_id}"]["test_metrics"]
        print(
            f"joint{joint_id} test: "
            f"mae={metrics['mae']:.6f}, "
            f"rmse={metrics['rmse']:.6f}, "
            f"mae_ratio={metrics['mae_ratio']}, "
            f"corr={metrics['corr']}"
        )


if __name__ == "__main__":
    main()
