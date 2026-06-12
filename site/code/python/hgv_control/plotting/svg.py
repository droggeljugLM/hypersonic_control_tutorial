"""Small SVG plotting utilities using only the Python standard library."""

from __future__ import annotations

import csv
from pathlib import Path
from xml.sax.saxutils import escape


Color = str
Series = tuple[str, list[float], list[float], Color]


def read_numeric_csv(path: Path) -> list[dict[str, float | str]]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        rows: list[dict[str, float | str]] = []
        for row in csv.DictReader(handle):
            parsed: dict[str, float | str] = {}
            for key, value in row.items():
                if value in {"True", "False"}:
                    parsed[key] = value
                    continue
                try:
                    parsed[key] = float(value)
                except (TypeError, ValueError):
                    parsed[key] = value
            rows.append(parsed)
        return rows


def _span(values: list[float]) -> tuple[float, float]:
    lo = min(values)
    hi = max(values)
    if lo == hi:
        pad = 1.0 if lo == 0.0 else abs(lo) * 0.05
        return lo - pad, hi + pad
    pad = 0.06 * (hi - lo)
    return lo - pad, hi + pad


def _points(xs: list[float], ys: list[float], x_min: float, x_max: float, y_min: float, y_max: float, box: tuple[int, int, int, int]) -> str:
    left, top, width, height = box
    parts: list[str] = []
    for x, y in zip(xs, ys):
        px = left + (x - x_min) / (x_max - x_min) * width
        py = top + height - (y - y_min) / (y_max - y_min) * height
        parts.append(f"{px:.2f},{py:.2f}")
    return " ".join(parts)


def line_plot(
    output: Path,
    title: str,
    x_label: str,
    y_label: str,
    series: list[Series],
    hlines: list[tuple[str, float, Color]] | None = None,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    width, height = 920, 520
    left, top, plot_width, plot_height = 78, 54, 720, 380
    all_x = [x for _, xs, _, _ in series for x in xs]
    all_y = [y for _, _, ys, _ in series for y in ys]
    if hlines:
        all_y.extend(value for _, value, _ in hlines)
    x_min, x_max = _span(all_x)
    y_min, y_max = _span(all_y)
    box = (left, top, plot_width, plot_height)

    body: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.0f}" y="28" text-anchor="middle" font-family="Arial" font-size="20">{escape(title)}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" fill="#fafafa" stroke="#222"/>',
        f'<line x1="{left}" y1="{top + plot_height}" x2="{left + plot_width}" y2="{top + plot_height}" stroke="#222"/>',
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{top + plot_height}" stroke="#222"/>',
        f'<text x="{left + plot_width/2:.0f}" y="{height - 28}" text-anchor="middle" font-family="Arial" font-size="14">{escape(x_label)}</text>',
        f'<text transform="translate(22,{top + plot_height/2:.0f}) rotate(-90)" text-anchor="middle" font-family="Arial" font-size="14">{escape(y_label)}</text>',
        f'<text x="{left}" y="{top + plot_height + 22}" text-anchor="middle" font-family="Arial" font-size="11">{x_min:.2f}</text>',
        f'<text x="{left + plot_width}" y="{top + plot_height + 22}" text-anchor="middle" font-family="Arial" font-size="11">{x_max:.2f}</text>',
        f'<text x="{left - 10}" y="{top + plot_height}" text-anchor="end" font-family="Arial" font-size="11">{y_min:.2f}</text>',
        f'<text x="{left - 10}" y="{top + 5}" text-anchor="end" font-family="Arial" font-size="11">{y_max:.2f}</text>',
    ]

    if hlines:
        for label, value, color in hlines:
            y = top + plot_height - (value - y_min) / (y_max - y_min) * plot_height
            body.append(f'<line x1="{left}" y1="{y:.2f}" x2="{left + plot_width}" y2="{y:.2f}" stroke="{color}" stroke-dasharray="8 6"/>')
            body.append(f'<text x="{left + plot_width + 8}" y="{y + 4:.2f}" font-family="Arial" font-size="12" fill="{color}">{escape(label)}</text>')

    legend_x, legend_y = left + plot_width + 30, top + 24
    for index, (label, xs, ys, color) in enumerate(series):
        body.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2.2" points="{_points(xs, ys, x_min, x_max, y_min, y_max, box)}"/>'
        )
        y = legend_y + index * 24
        body.append(f'<line x1="{legend_x}" y1="{y}" x2="{legend_x + 28}" y2="{y}" stroke="{color}" stroke-width="3"/>')
        body.append(f'<text x="{legend_x + 36}" y="{y + 4}" font-family="Arial" font-size="13">{escape(label)}</text>')

    body.append("</svg>")
    output.write_text("\n".join(body), encoding="utf-8")


def bar_chart(output: Path, title: str, y_label: str, labels: list[str], values: list[float], color: str = "#2f6fbb") -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    width, height = 860, 500
    left, top, plot_width, plot_height = 82, 54, 690, 360
    y_min, y_max = _span(values + [0.0])
    zero_y = top + plot_height - (0.0 - y_min) / (y_max - y_min) * plot_height
    bar_width = plot_width / max(1, len(values)) * 0.62
    body = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2:.0f}" y="28" text-anchor="middle" font-family="Arial" font-size="20">{escape(title)}</text>',
        f'<rect x="{left}" y="{top}" width="{plot_width}" height="{plot_height}" fill="#fafafa" stroke="#222"/>',
        f'<line x1="{left}" y1="{zero_y:.2f}" x2="{left + plot_width}" y2="{zero_y:.2f}" stroke="#333"/>',
        f'<text transform="translate(24,{top + plot_height/2:.0f}) rotate(-90)" text-anchor="middle" font-family="Arial" font-size="14">{escape(y_label)}</text>',
    ]
    for index, (label, value) in enumerate(zip(labels, values)):
        cx = left + (index + 0.5) * plot_width / len(values)
        y = top + plot_height - (value - y_min) / (y_max - y_min) * plot_height
        rect_y = min(y, zero_y)
        rect_h = abs(zero_y - y)
        fill = color if value >= 0 else "#c23b3b"
        body.append(f'<rect x="{cx - bar_width/2:.2f}" y="{rect_y:.2f}" width="{bar_width:.2f}" height="{rect_h:.2f}" fill="{fill}"/>')
        body.append(f'<text x="{cx:.2f}" y="{top + plot_height + 24}" text-anchor="middle" font-family="Arial" font-size="11">{escape(label)}</text>')
        body.append(f'<text x="{cx:.2f}" y="{rect_y - 6:.2f}" text-anchor="middle" font-family="Arial" font-size="11">{value:.1f}</text>')
    body.append("</svg>")
    output.write_text("\n".join(body), encoding="utf-8")

