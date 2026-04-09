# 机械臂静态重力补偿参数辨识

## 1. 实现思路

项目目标：实现机械臂在静态工况下的重力参数辨识，并据此实现关节空间的重力补偿前馈。

机械臂的标准关节空间动力学可以写成：

$$
M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau + \tau_{ext}
$$

其中：

- $M(q)$ 是惯性矩阵
- $C(q,\dot{q})\dot{q}$ 是科氏力 / 离心力项
- $g(q)$ 是重力项
- $\tau$ 是关节驱动力矩
- $\tau_{ext}$ 是外部扰动或外力等效到关节空间后的力矩

本项目当前只聚焦于**静态重力辨识**。  
因此在静止工况下令速度和加速度为零，并忽略外部扰动，仅保留重力项：

$$
\dot{q} = 0,\qquad \ddot{q} = 0,\qquad \tau_{ext} = 0
$$

此时动力学方程退化为：

$$
\tau_g(q) = g(q)
$$

在 Pinocchio 中，重力项对动力学参数是线性的，因此可以写成：

$$
\tau_g(q) = Y_g(q)\,\pi
$$

其中：

- $\tau_g(q)$ 为当前姿态下的重力补偿力矩，也就是力矩观测向量
- $Y_g(q)$ 为重力回归矩阵
- $\pi$ 为动力学参数向量，也就是待辨识参数

整个静态重力辨识流程如下。

### 1.1 建立刚体动力学模型

利用 URDF 和 Pinocchio 建立机械臂刚体动力学模型。

### 1.2 构造 Pinocchio 状态量

将机械臂的物理关节角 `theta` 转换为 Pinocchio 需要的广义坐标 `q`。

### 1.3 构造静态样本

通过实验采集多组静态样本，每组样本包含：

- 关节角 `theta`
- 对应的关节力矩 `tau`

### 1.4 堆叠最小二乘问题

将所有样本堆叠为最小二乘问题：

$$
A\pi = b
$$

其中：

- $A$ 为所有静态样本对应的回归矩阵堆叠结果
- $b$ 为所有静态样本对应的力矩观测向量堆叠结果

### 1.5 提取可辨识基参数

由于静态重力辨识通常无法唯一确定全部原始动力学参数，因此对回归矩阵 $A$ 做 SVD 分解，提取可辨识基参数空间：

$$
\pi = V_{base}\alpha
$$

代入后可得到降维后的辨识问题：

$$
A_{base}\alpha = b
$$

### 1.6 最小二乘辨识

通过最小二乘求得基参数 $\hat{\alpha}$，再重构得到一组等价的参数向量 $\hat{\pi}$：

$$
\hat{\pi} = V_{base}\hat{\alpha}
$$

### 1.7 重力补偿预测

最终利用辨识结果预测任意姿态下的重力补偿力矩：

$$
\tau_{pred}(q) = Y_g(q)\hat{\pi}
$$

说明：

- 本项目当前只处理静态重力辨识
- 不涉及完整动态工况下的科里奥利项和惯量项辨识
- 第 1.5 到第 1.7 步的具体数学推导可参考 [docs/svd_base_parameter_identification.md](docs/svd_base_parameter_identification.md)

## 2. 库依赖

当前项目使用 Python 版本的 Pinocchio 完成建模、重力计算和回归矩阵构造。

核心依赖如下：

- Python 3.11
- Pinocchio 3.9.0
- NumPy
- SciPy

标准库依赖如下：

- `pathlib`
- `math`
- `csv`
- `json`
- `datetime`
- `dataclasses`

推荐环境：

- Windows + Conda

典型安装方式：

```powershell
conda create -n mec_arm python=3.11 -y
conda activate mec_arm
conda install pinocchio numpy scipy -c conda-forge --solver libmamba -y
```

说明：

- `pip list` 中包名通常显示为 `pin`
- 代码中实际导入方式为 `import pinocchio as pin`

## 3. 文件结构 文件职责

当前项目目录结构如下：

```text
gravity_identification/
  data/
    gravity_samples_demo.csv
  docs/
    svd_base_parameter_identification.md
  launch/
    demo_synthetic.py
    run_identification.py
  results/
    gravity_samples_demo/
      时间戳/
        data/
        figures/
        report/
  src/
    base_params.py
    csv_dataset.py
    dataset.py
    gravity.py
    identify.py
    report_plots.py
    report_writer.py
    results_io.py
    urdf_import.py
  README.md
```

各文件职责如下：

- [README.md](README.md)
  项目总说明，介绍辨识思路、依赖和目录结构。

- [data/gravity_samples_demo.csv](data/gravity_samples_demo.csv)
  示例静态样本数据。每一行包含一组关节角 `theta1~theta6` 和关节力矩 `tau1~tau6`。

- [docs/svd_base_parameter_identification.md](docs/svd_base_parameter_identification.md)
  记录基参数辨识的数学推导，重点说明为什么要对原始回归矩阵做 SVD 降维。

- [launch/demo_synthetic.py](launch/demo_synthetic.py)
  使用 URDF 模型制作数据集，验证“建模 -> 回归矩阵 -> 基参数辨识 -> 预测”的整条链路是否跑通。

- [launch/run_identification.py](launch/run_identification.py)
  项目主入口。负责读取 CSV、执行辨识、保存结果、生成图表和报告。

- [src/urdf_import.py](src/urdf_import.py)
  负责加载 URDF，构造 Pinocchio 模型与数据对象，并实现 `theta -> q` 的状态转换。

- [src/gravity.py](src/gravity.py)
  负责计算当前姿态下的重力补偿力矩 `tau_g`，以及静态辨识所需的重力回归矩阵 `Y_g(q)`。

- [src/dataset.py](src/dataset.py)
  定义单条静态辨识样本 `GravitySample` 的统一数据结构。

- [src/csv_dataset.py](src/csv_dataset.py)
  负责从 CSV 中读取样本，并转换为 `GravitySample` 列表。

- [src/identify.py](src/identify.py)
  负责构造总回归矩阵 `A` 和观测向量 `b`，并提供全参数辨识与重力预测函数。

- [src/base_params.py](src/base_params.py)
  负责对回归矩阵做 SVD 分析，提取基参数空间，并完成基参数辨识与参数重构。

- [src/results_io.py](src/results_io.py)
  负责创建结果目录，并保存 `summary.json`、`identification_result.npz`、`sample_predictions.csv` 等数据文件。

- [src/report_plots.py](src/report_plots.py)
  负责生成奇异值谱、样本误差、关节平均绝对误差等 SVG 图表。

- [src/report_writer.py](src/report_writer.py)
  负责自动生成辨识报告 `report.md`，用于实验记录和展示。

- [results/](results)
  存放每次辨识运行的结果，按“数据集名称/时间戳”分类管理。

结果目录说明：

- `data/`
  保存本次辨识的核心数值结果与逐样本预测结果。

- `figures/`
  保存本次辨识自动生成的图表。

- `report/`
  保存本次辨识自动生成的 Markdown 报告。

当前推荐运行方式：

```powershell
conda run -n 虚拟环境名称 python launch/run_identification.py
```

运行完成后，会在 [results](results) 下自动生成一份完整结果报告，用于后续分析。
