"""
collect_serial_raw.py

串口采集机械臂辨识数据，实现以下功能：

1. 实时读取下位机通过串口发送的二进制数据
2. 导出原始 CSV 文件 mc02_capture_raw.csv
3. 采集结束后，根据 qdot 阈值清洗数据 筛出准静态数据 
4. 生成可直接用于重力辨识的 CSV 文件 mc02_capture_cleaned.csv

"""

from pathlib import Path
import csv
import struct
import time
import serial


# =========================
# 串口与协议配置
# =========================
SOF1 = 0xA5
SOF2 = 0x5A
FRAME_LEN = 58
BAUDRATE = 115200
PORT = "COM7"   # 按实际串口号修改


# =========================
# 输出路径配置
# =========================
PROJECT_ROOT = Path(__file__).resolve().parents[1]
RAW_OUTPUT_CSV = PROJECT_ROOT / "csv" / "mc02_capture_raw.csv"
CLEAN_OUTPUT_CSV = PROJECT_ROOT / "data" / "mc02_capture_cleaned.csv"


# =========================
# 清洗参数
# =========================
QDOT_THRESHOLD_RAD_S = 0.01  # 准静态阈值，可按实际情况调整
MIN_VALID_DT_S = 1.0e-4      # 避免 dt 过小导致差分发散


# 下位机 CRC16 查表，需与 MWL_CRC16 保持一致
CRC16_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]


# =========================
# 计算与下位机一致的 CRC16
# =========================
def crc16_dji(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


# =========================
# 创建原始csv数据文件
# =========================
def make_raw_csv_writer(csv_path: Path):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    csv_file = csv_path.open("w", encoding="utf-8-sig", newline="")
    writer = csv.writer(csv_file)
    writer.writerow([
        "seq", "tick_ms",
        "theta1", "theta2", "theta3", "theta4", "theta5", "theta6",
        "tau1", "tau2", "tau3", "tau4", "tau5", "tau6",
    ])
    return csv_file, writer


# =========================
# 读取原始csv文件 转换为便于处理的字典列表
# =========================
def load_raw_rows(csv_path: Path):
    rows = []

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)

        for row in reader:
            rows.append({
                "seq": int(row["seq"]),
                "tick_ms": int(row["tick_ms"]),
                "theta": [float(row[f"theta{i}"]) for i in range(1, 7)],
                "tau": [float(row[f"tau{i}"]) for i in range(1, 7)],
            })

    return rows


# =========================
# 使用中心差分（边界使用前向/后向差分）计算qdot 用于阈值判断清洗数据
# =========================
def compute_qdot_at_row(rows, index: int):
    
    n = len(rows)

    if n < 2:
        return None

    if index == 0:
        row_a = rows[0]
        row_b = rows[1]
    elif index == n - 1:
        row_a = rows[n - 2]
        row_b = rows[n - 1]
    else:
        row_a = rows[index - 1]
        row_b = rows[index + 1]

    dt_s = (row_b["tick_ms"] - row_a["tick_ms"]) * 1.0e-3
    if dt_s <= MIN_VALID_DT_S:
        return None

    qdot = []
    for theta_a, theta_b in zip(row_a["theta"], row_b["theta"]):
        qdot.append((theta_b - theta_a) / dt_s)

    return qdot


# =========================
# 根据 qdot 阈值清洗数据，整理出可以直接用于辨识（run_identification.py）的csv文件
# =========================
def write_clean_csv(raw_csv_path: Path, clean_csv_path: Path, qdot_threshold: float):
    rows = load_raw_rows(raw_csv_path)
    clean_csv_path.parent.mkdir(parents=True, exist_ok=True)

    if len(rows) < 2:
        raise ValueError("原始数据行数不足，无法计算 qdot 并清洗")

    kept_count = 0
    skipped_dt_count = 0

    with clean_csv_path.open("w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            "seq", "tick_ms", "qdot_max_abs",
            "theta1", "theta2", "theta3", "theta4", "theta5", "theta6",
            "tau1", "tau2", "tau3", "tau4", "tau5", "tau6",
        ])

        for index, row in enumerate(rows):
            qdot = compute_qdot_at_row(rows, index)
            if qdot is None:
                skipped_dt_count += 1
                continue

            qdot_max_abs = max(abs(v) for v in qdot)
            if qdot_max_abs > qdot_threshold:
                continue

            writer.writerow([
                row["seq"],
                row["tick_ms"],
                qdot_max_abs,
                *row["theta"],
                *row["tau"],
            ])
            kept_count += 1

    print("cleaning finished")
    print(f"raw rows           = {len(rows)}")
    print(f"skipped invalid dt = {skipped_dt_count}")
    print(f"kept rows          = {kept_count}")
    print(f"qdot threshold     = {qdot_threshold:.6f} rad/s")
    print(f"clean csv          = {clean_csv_path}")

    if kept_count == 0:
        print("warning: 清洗后没有保留任何样本，请放宽 qdot 阈值或检查采样流程。")


# =========================
# 实时采集串口数据 写进csv中
# =========================
def capture_raw_csv():
    ser = serial.Serial(PORT, BAUDRATE, timeout=0.05)
    print(f"open serial: {PORT} @ {BAUDRATE}")

    csv_file, writer = make_raw_csv_writer(RAW_OUTPUT_CSV)
    print(f"raw csv: {RAW_OUTPUT_CSV}")

    buf = bytearray()
    frame_count = 0
    bad_crc_count = 0
    last_print_t = time.time()

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)

            while True:
                sof_idx = buf.find(bytes([SOF1, SOF2]))
                if sof_idx < 0:
                    if len(buf) > 1:
                        del buf[:-1]
                    break

                if sof_idx > 0:
                    del buf[:sof_idx]

                if len(buf) < FRAME_LEN:
                    break

                frame = bytes(buf[:FRAME_LEN])

                recv_crc = int.from_bytes(frame[-2:], byteorder="little", signed=False)
                calc_crc = crc16_dji(frame[:-2])

                if recv_crc != calc_crc:
                    bad_crc_count += 1
                    del buf[0]
                    continue

                unpacked = struct.unpack("<BBHI12fH", frame)
                _, _, seq, tick_ms, *floats, _ = unpacked

                q = floats[:6]
                tau = floats[6:]

                writer.writerow([seq, tick_ms, *q, *tau])
                frame_count += 1

                del buf[:FRAME_LEN]

            now = time.time()
            if now - last_print_t > 1.0:
                print(f"frames={frame_count}, bad_crc={bad_crc_count}, buffer={len(buf)}")
                last_print_t = now

    except KeyboardInterrupt:
        print("stop capture")

    finally:
        csv_file.close()
        ser.close()
        print(f"saved raw csv: {RAW_OUTPUT_CSV}")
        print(f"valid frames  : {frame_count}")
        print(f"bad crc frames: {bad_crc_count}")


def main():
    capture_raw_csv()
    write_clean_csv(RAW_OUTPUT_CSV, CLEAN_OUTPUT_CSV, QDOT_THRESHOLD_RAD_S)


if __name__ == "__main__":
    main()
