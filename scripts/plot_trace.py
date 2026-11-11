"""Render C-generated SIMULATION CSV as SVG. Python standard library only."""
import csv
from pathlib import Path
import sys


def main():
    if len(sys.argv) != 3:
        raise SystemExit("Usage: python scripts/plot_trace.py assets/simulation_trace.csv assets/simulation_trace.svg")
    with open(sys.argv[1], newline="", encoding="ascii") as source:
        rows = list(csv.DictReader(source))
    left, right, top, bottom = 80, 920, 90, 410
    max_time = int(rows[-1]["time_ms"]) / 1000
    def x(row):
        return left + (int(row["time_ms"]) / 1000) / max_time * (right - left)
    def y(value, maximum):
        return bottom - value / maximum * (bottom - top)
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="520" viewBox="0 0 1000 520" role="img" aria-labelledby="title desc">',
             '<title id="title">Motor response: software simulation only</title>',
             '<desc id="desc">Requested speed changes at four seconds; shaft jam at nine seconds; stall fault inhibits PWM at 9.8 seconds.</desc>',
             '<rect width="100%" height="100%" fill="#fff"/>',
             '<g font-family="Arial,sans-serif" font-size="14" fill="#172b4d">',
             '<text x="80" y="32" font-size="23">Motor response and latched shutdown</text>',
             '<text x="80" y="58">Deterministic C model - SIMULATION ONLY; no physical measurements</text>']
    for rpm in range(0, 3001, 500):
        yp = y(rpm, 3000)
        parts.extend([f'<line x1="{left}" y1="{yp}" x2="{right}" y2="{yp}" stroke="#e5eaf0"/>',
                      f'<text x="65" y="{yp+5}" text-anchor="end">{rpm}</text>'])
    for sec in range(0, 13, 2):
        xp = left + sec / max_time * (right - left)
        parts.append(f'<text x="{xp}" y="435" text-anchor="middle">{sec}</text>')
    parts.extend(['<text x="80" y="80">RPM</text>', '<text x="920" y="80" text-anchor="end">PWM duty 0..1 (right axis)</text>',
                  '<text x="500" y="462" text-anchor="middle">Simulated time (seconds)</text>'])
    for duty in (0, 0.25, 0.5, 0.75, 1):
        parts.append(f'<text x="935" y="{y(duty,1)+5}">{duty:g}</text>')
    series = [("requested_rpm", 3000, "#62748e", 'stroke-dasharray="7 5"'),
              ("measured_rpm", 3000, "#1265b0", ""), ("pwm", 1, "#ba5b10", "")]
    for name, maximum, color, style in series:
        points = " ".join(f'{x(row):.2f},{y(float(row[name]),maximum):.2f}' for row in rows)
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2" {style}/>')
    first_fault = next(row for row in rows if row["state"] == "FAULT")
    for sec, label, yp in [(4, "Target change", 115), (9, "Jam injected", 140),
                            (int(first_fault["time_ms"])/1000, "Fault; PWM off", 165)]:
        xp = left + sec/max_time*(right-left)
        parts.append(f'<line x1="{xp}" y1="{top}" x2="{xp}" y2="{bottom}" stroke="#8c2741" stroke-dasharray="3 4"/>')
        parts.append(f'<text x="{xp-6}" y="{yp}" text-anchor="end">{label} ({sec:g}s)</text>')
    for xp, color, label in [(80,"#62748e","Requested RPM"),(320,"#1265b0","Simulated sensor RPM"),(650,"#ba5b10","PWM duty")]:
        parts.append(f'<line x1="{xp}" y1="495" x2="{xp+30}" y2="495" stroke="{color}" stroke-width="3"/>')
        parts.append(f'<text x="{xp+40}" y="500">{label}</text>')
    parts.append('</g></svg>')
    Path(sys.argv[2]).write_text("\n".join(parts), encoding="utf-8")
    print(f"Created {sys.argv[2]} from {len(rows)} simulated samples")

if __name__ == "__main__":
    main()
