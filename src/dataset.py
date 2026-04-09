from dataclasses import dataclass
import numpy as np

@dataclass
class GravitySample:
    theta: np.ndarray
    tau: np.ndarray

    # @brief    初始化数据格式
    def __post_init__(self):
        # 将力矩数据和关节角度数据转换为numpy数组，强制为float并转为向量
        self.theta = np.asarray(self.theta, dtype=float).reshape(-1)
        self.tau = np.asarray(self.tau, dtype=float).reshape(-1)

        # 检查长度
        if self.theta.shape != (6,):
            raise ValueError("theta must contain 6 joint angles")
        
        if self.tau.shape != (6,):
            raise ValueError("tau must contain 6 joint torques")