'''

生成辨识结果图片figure

'''

from pathlib import Path
import math
import numpy as np


# SVG 画布和边距的全局配置，后续所有图都复用这一套布局参数
SVG_WIDTH = 960
SVG_HEIGHT = 540
MARGIN_LEFT = 90
MARGIN_RIGHT = 30
MARGIN_TOP = 60
MARGIN_BOTTOM = 90


# @brief    对 SVG 中的特殊字符做转义，避免标题或标签文本破坏 XML 结构
# @retval   返回可安全写入 SVG 的字符串
def _svg_escape(text):
    text = str(text)
    text = text.replace("&", "&amp;")
    text = text.replace("<", "&lt;")
    text = text.replace(">", "&gt;")
    text = text.replace('"', "&quot;")
    return text


# @brief    格式化坐标轴刻度文本
#           对跨度很大的数值自动切换为科学计数法
# @retval   返回刻度字符串
def _format_tick(value):
    value = float(value)

    if abs(value) >= 1e3 or (abs(value) > 0 and abs(value) < 1e-3):
        return f"{value:.1e}"

    return f"{value:.4f}".rstrip("0").rstrip(".")


# @brief    组合完整的 SVG 文档
#           统一添加头部、样式、背景和绘图元素
# @retval   返回完整 SVG 字符串
def _build_svg_document(elements):
    header = (
        f'<svg xmlns="http://www.w3.org/2000/svg" '
        f'width="{SVG_WIDTH}" height="{SVG_HEIGHT}" '
        f'viewBox="0 0 {SVG_WIDTH} {SVG_HEIGHT}">'
    )

    style = """
    <style>
      .title { font: 700 24px "Segoe UI", "Microsoft YaHei", sans-serif; fill: #102A43; }
      .subtitle { font: 400 14px "Segoe UI", "Microsoft YaHei", sans-serif; fill: #486581; }
      .axis { stroke: #243B53; stroke-width: 2; }
      .grid { stroke: #D9E2EC; stroke-width: 1; }
      .tick { font: 400 13px "Segoe UI", "Microsoft YaHei", sans-serif; fill: #334E68; }
      .label { font: 600 14px "Segoe UI", "Microsoft YaHei", sans-serif; fill: #243B53; }
    </style>
    """

    background = f'<rect x="0" y="0" width="{SVG_WIDTH}" height="{SVG_HEIGHT}" fill="#F8FBFF"/>'

    footer = "</svg>"

    return "\n".join([header, style, background] + elements + [footer])


# @brief    生成通用柱状图
#           当前用于样本误差图和关节平均绝对误差图
# @retval   返回 SVG 文本
def _make_bar_chart_svg(values, labels, title, subtitle, y_label, color):
    values = np.asarray(values, dtype=float).reshape(-1)

    if len(values) == 0:
        raise ValueError("values must not be empty")

    if len(labels) != len(values):
        raise ValueError("labels length must match values length")

    plot_left = MARGIN_LEFT
    plot_right = SVG_WIDTH - MARGIN_RIGHT
    plot_top = MARGIN_TOP
    plot_bottom = SVG_HEIGHT - MARGIN_BOTTOM
    plot_width = plot_right - plot_left
    plot_height = plot_bottom - plot_top

    # 柱状图默认从 0 开始绘制，因此这里取最大值作为上界
    y_max = float(np.max(values))
    if y_max <= 0:
        y_max = 1.0

    elements = []

    elements.append(f'<text x="{plot_left}" y="32" class="title">{_svg_escape(title)}</text>')
    elements.append(f'<text x="{plot_left}" y="52" class="subtitle">{_svg_escape(subtitle)}</text>')

    num_ticks = 5
    for i in range(num_ticks + 1):
        tick_value = y_max * i / num_ticks
        y = plot_bottom - (tick_value / y_max) * plot_height

        elements.append(f'<line x1="{plot_left}" y1="{y}" x2="{plot_right}" y2="{y}" class="grid"/>')
        elements.append(
            f'<text x="{plot_left - 12}" y="{y + 5}" text-anchor="end" class="tick">'
            f'{_svg_escape(_format_tick(tick_value))}</text>'
        )

    elements.append(f'<line x1="{plot_left}" y1="{plot_bottom}" x2="{plot_right}" y2="{plot_bottom}" class="axis"/>')
    elements.append(f'<line x1="{plot_left}" y1="{plot_top}" x2="{plot_left}" y2="{plot_bottom}" class="axis"/>')

    elements.append(
        f'<text x="26" y="{(plot_top + plot_bottom) / 2}" transform="rotate(-90 26 {(plot_top + plot_bottom) / 2})" class="label">'
        f'{_svg_escape(y_label)}</text>'
    )

    # 为每个样本或关节分配等宽槽位，再在槽位中绘制实际柱子
    slot_width = plot_width / len(values)
    bar_width = slot_width * 0.65
    label_step = max(1, math.ceil(len(values) / 12))

    for i, value in enumerate(values):
        x = plot_left + i * slot_width + (slot_width - bar_width) / 2
        bar_height = (value / y_max) * plot_height
        y = plot_bottom - bar_height

        elements.append(
            f'<rect x="{x}" y="{y}" width="{bar_width}" height="{bar_height}" '
            f'fill="{color}" rx="4" ry="4"/>'
        )

        if i % label_step == 0 or i == len(values) - 1:
            elements.append(
                f'<text x="{x + bar_width / 2}" y="{plot_bottom + 22}" text-anchor="middle" class="tick">'
                f'{_svg_escape(labels[i])}</text>'
            )

    return _build_svg_document(elements)


# @brief    生成通用折线图
#           当前主要用于绘制奇异值谱
# @retval   返回 SVG 文本
def _make_line_chart_svg(values, title, subtitle, y_label, color):
    values = np.asarray(values, dtype=float).reshape(-1)

    if len(values) == 0:
        raise ValueError("values must not be empty")

    plot_left = MARGIN_LEFT
    plot_right = SVG_WIDTH - MARGIN_RIGHT
    plot_top = MARGIN_TOP
    plot_bottom = SVG_HEIGHT - MARGIN_BOTTOM
    plot_width = plot_right - plot_left
    plot_height = plot_bottom - plot_top

    y_min = float(np.min(values))
    y_max = float(np.max(values))

    # 当所有值完全相同时，手动补一个小范围，避免除零和折线退化
    if y_min == y_max:
        pad = 1.0 if y_min == 0 else abs(y_min) * 0.1
        y_min -= pad
        y_max += pad

    elements = []

    elements.append(f'<text x="{plot_left}" y="32" class="title">{_svg_escape(title)}</text>')
    elements.append(f'<text x="{plot_left}" y="52" class="subtitle">{_svg_escape(subtitle)}</text>')

    num_ticks = 5
    for i in range(num_ticks + 1):
        tick_value = y_min + (y_max - y_min) * i / num_ticks
        y = plot_bottom - ((tick_value - y_min) / (y_max - y_min)) * plot_height

        elements.append(f'<line x1="{plot_left}" y1="{y}" x2="{plot_right}" y2="{y}" class="grid"/>')
        elements.append(
            f'<text x="{plot_left - 12}" y="{y + 5}" text-anchor="end" class="tick">'
            f'{_svg_escape(_format_tick(tick_value))}</text>'
        )

    elements.append(f'<line x1="{plot_left}" y1="{plot_bottom}" x2="{plot_right}" y2="{plot_bottom}" class="axis"/>')
    elements.append(f'<line x1="{plot_left}" y1="{plot_top}" x2="{plot_left}" y2="{plot_bottom}" class="axis"/>')

    elements.append(
        f'<text x="26" y="{(plot_top + plot_bottom) / 2}" transform="rotate(-90 26 {(plot_top + plot_bottom) / 2})" class="label">'
        f'{_svg_escape(y_label)}</text>'
    )

    # 只有一个点时把它放在画布中间，否则按横向均匀分布
    if len(values) == 1:
        xs = [plot_left + plot_width / 2]
    else:
        xs = [plot_left + i * plot_width / (len(values) - 1) for i in range(len(values))]

    ys = [
        plot_bottom - ((value - y_min) / (y_max - y_min)) * plot_height
        for value in values
    ]

    points = " ".join(f"{x},{y}" for x, y in zip(xs, ys))
    elements.append(
        f'<polyline fill="none" stroke="{color}" stroke-width="3" points="{points}"/>'
    )

    if len(values) <= 80:
        for x, y in zip(xs, ys):
            elements.append(f'<circle cx="{x}" cy="{y}" r="3.5" fill="{color}"/>')

    label_step = max(1, math.ceil(len(values) / 12))
    for i, x in enumerate(xs):
        if i % label_step == 0 or i == len(xs) - 1:
            elements.append(
                f'<text x="{x}" y="{plot_bottom + 22}" text-anchor="middle" class="tick">{i + 1}</text>'
            )

    return _build_svg_document(elements)


# @brief    保存奇异值谱图
#           先对奇异值做 log10 变换，再输出为 SVG
# @retval   返回图像路径
def save_singular_values_svg(result_dir, singular_values):
    singular_values = np.asarray(singular_values, dtype=float).reshape(-1)

    if len(singular_values) == 0:
        raise ValueError("singular_values must not be empty")

    # 奇异值跨度通常很大，转为对数尺度后更容易观察有效秩变化
    log_values = np.log10(np.maximum(singular_values, 1e-16))

    svg_text = _make_line_chart_svg(
        values=log_values,
        title="Singular Value Spectrum",
        subtitle="log10 scale, used to reveal effective rank and parameter observability",
        y_label="log10(sigma)",
        color="#C84C09",
    )

    output_path = Path(result_dir) / "figures" / "singular_values.svg"
    output_path.write_text(svg_text, encoding="utf-8")
    return output_path


# @brief    保存逐样本误差范数图
# @retval   返回图像路径
def save_sample_error_norms_svg(result_dir, error_norms):
    error_norms = np.asarray(error_norms, dtype=float).reshape(-1)

    labels = [str(i + 1) for i in range(len(error_norms))]

    svg_text = _make_bar_chart_svg(
        values=error_norms,
        labels=labels,
        title="Sample Error Norms",
        subtitle="L2 norm of prediction error for each static sample",
        y_label="||tau_meas - tau_pred||_2",
        color="#1D7874",
    )

    output_path = Path(result_dir) / "figures" / "sample_error_norms.svg"
    output_path.write_text(svg_text, encoding="utf-8")
    return output_path


# @brief    保存各关节平均绝对误差图
#           先对每个关节在所有样本上的误差取绝对值平均，再画柱状图
# @retval   返回图像路径
def save_joint_mean_abs_error_svg(result_dir, errors):
    errors = np.asarray(errors, dtype=float)

    if errors.ndim != 2:
        raise ValueError("errors must be a 2D array")

    joint_mae = np.mean(np.abs(errors), axis=0)
    labels = [f"J{i + 1}" for i in range(errors.shape[1])]

    svg_text = _make_bar_chart_svg(
        values=joint_mae,
        labels=labels,
        title="Joint Mean Absolute Error",
        subtitle="Average absolute torque prediction error for each joint",
        y_label="mean absolute error (N*m)",
        color="#2F6FED",
    )

    output_path = Path(result_dir) / "figures" / "joint_mean_abs_error.svg"
    output_path.write_text(svg_text, encoding="utf-8")
    return output_path
