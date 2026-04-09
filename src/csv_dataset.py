from pathlib import Path
import csv

from dataset import GravitySample

# 定义表头
REQUIRED_COLUMNS = [
    "theta1", "theta2", "theta3", "theta4", "theta5", "theta6",
    "tau1", "tau2", "tau3", "tau4", "tau5", "tau6",
]

#检查表头是否正确
def validate_csv_header(fieldnames):
    if fieldnames is None:
        raise ValueError("CSV header is missing")
    
    missing = [name for name in REQUIRED_COLUMNS if name not in fieldnames]

    if len(missing) > 0:
        raise ValueError(f"Missing columns in CSV header: {missing}")
    
# 将字典转为角度列表和力矩列表并构造GravitySample样本
def parse_gravity_row(row):
    theta = [
        float(row["theta1"]),
        float(row["theta2"]),
        float(row["theta3"]),
        float(row["theta4"]),
        float(row["theta5"]),
        float(row["theta6"]),
    ]

    tau = [
        float(row["tau1"]),
        float(row["tau2"]),
        float(row["tau3"]),
        float(row["tau4"]),
        float(row["tau5"]),
        float(row["tau6"]),
    ]

    sample = GravitySample(theta=theta, tau=tau)
    return sample

# 打开csv文件并解析返回samples
def load_gravity_samples_from_csv(csv_path):
    csv_path = Path(csv_path)

    # 路径检查
    if not csv_path.exists():
        raise FileNotFoundError(f"File not found: {csv_path}")

    samples = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)

        validate_csv_header(reader.fieldnames)

        for row_index, row in enumerate(reader, start=2):
            try:
                sample = parse_gravity_row(row)
            except Exception as e:
                raise ValueError(f"Failed to parse row {row_index}: {e}") from e
            
            samples.append(sample)

        if len(samples) == 0:
            raise ValueError("CSV file contains no data rows")

    return samples