'''

生成辨识结果报告report

'''

from pathlib import Path


# @brief    生成一次辨识实验的 Markdown 报告
#           报告内容包括：输入数据、矩阵规模、辨识秩、误差指标和图表引用
# @retval   返回生成后的 report.md 路径
def write_identification_report(
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
    result_dir = Path(result_dir)

    # 报告文件统一写入结果目录下的 report 子目录
    report_path = result_dir / "report" / "report.md"

    # 图像使用相对路径引用，便于整个结果目录被整体移动或归档
    text = f"""# 机械臂静态重力参数辨识报告

## 1. 项目目标

本次辨识的目标是针对机械臂静态工况建立重力补偿前馈模型。  
方法上采用 Pinocchio 从 URDF 构建刚体动力学模型，并基于静态样本构造重力回归矩阵，进一步通过 SVD 提取可辨识基参数，完成静态重力参数辨识。

## 2. 数据来源

- 输入 CSV: `{csv_path}`
- 样本数量: `{num_samples}`

## 3. 矩阵规模与辨识维度

- 原始回归矩阵维度: `{A_shape}`
- 观测向量维度: `{b_shape}`
- 基参数回归矩阵维度: `{A_base_shape}`
- 基参数映射矩阵维度: `{V_base_shape}`
- 原始问题有效秩: `{rank}`
- 基参数问题秩: `{rank_base}`

## 4. 误差指标

- 平均样本误差范数: `{mean_error:.6e}`
- 最大样本误差范数: `{max_error:.6e}`

## 5. 结果解读

1. 原始动力学参数在当前静态重力辨识任务下并非全部可辨识，因此需要通过 SVD 提取基参数空间。
2. 回归矩阵的有效秩反映了当前数据条件下真正能够被辨识出的独立参数组合数量。
3. 当前误差指标反映了模型预测力矩与样本观测力矩之间的吻合程度，误差越小说明重力补偿模型越准确。

## 6. 图表结果

### 奇异值谱
![奇异值谱](../figures/singular_values.svg)

### 样本误差范数
![样本误差范数](../figures/sample_error_norms.svg)

### 各关节平均绝对误差
![各关节平均绝对误差](../figures/joint_mean_abs_error.svg)

## 7. 结论

当前辨识流程已经完成了从 CSV 数据读取、URDF 建模、重力回归矩阵构建、基参数辨识到误差评估与结果可视化的完整闭环。  
后续若接入真实实验采样数据，可在此框架基础上进一步完成真实机械臂的静态重力补偿参数辨识。
"""

    # 统一以 UTF-8 写出，便于 Obsidian 和普通 Markdown 预览器直接打开
    report_path.write_text(text, encoding="utf-8")

    return report_path
