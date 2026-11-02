"""Generates docs/pin_diagram.png and docs/architecture.png (needs matplotlib + graphviz 'dot')."""
import os, subprocess
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

HERE = os.path.dirname(os.path.abspath(__file__))

COL = {"sense": "#2e7d32", "i2c": "#1565c0", "spi": "#6a1b9a", "motor": "#e65100",
       "out": "#c62828", "nav": "#00838f", "pwr": "#424242", "free": "#9e9e9e"}

# ESP32 DevKit V1 (30-pin) - USB at the bottom
LEFT = [("EN", "", "free"), ("VP (36)", "TCRT5000 left DO", "nav"), ("VN (39)", "TCRT5000 right DO", "nav"),
        ("D34", "MQ2 AO via 10k/10k divider", "sense"), ("D35", "MQ135 AO via 10k/10k divider", "sense"),
        ("D32", "TB6612 PWMA (left motor)", "motor"), ("D33", "TB6612 AIN1", "motor"),
        ("D25", "RGB LED  R (220 ohm)", "out"), ("D26", "RGB LED  G (220 ohm)", "out"),
        ("D27", "RGB LED  B (220 ohm)", "out"), ("D14", "HC-SR04 ECHO via 4.7k/6.8k", "nav"),
        ("D12", "TB6612 AIN2", "motor"), ("D13", "HC-SR04 TRIG", "nav"),
        ("GND", "Common ground", "pwr"), ("VIN", "5 V from LM2596 (robot)", "pwr")]
RIGHT = [("D23", "MOSI  -> RC522 + SD card", "spi"), ("D22", "SCL -> OLED, SHT31, MLX90614", "i2c"),
         ("TX0", "", "free"), ("RX0", "", "free"), ("D21", "SDA -> OLED, SHT31, MLX90614", "i2c"),
         ("D19", "MISO  -> RC522 + SD card", "spi"), ("D18", "SCK   -> RC522 + SD card", "spi"),
         ("D5", "RC522 SDA (SS)", "spi"), ("TX2 (17)", "TB6612 BIN1", "motor"),
         ("RX2 (16)", "TB6612 PWMB (right motor)", "motor"), ("D4", "Passive buzzer I/O", "out"),
         ("D2", "TB6612 BIN2", "motor"), ("D15", "SD card CS", "spi"), ("GND", "Common ground", "pwr"),
         ("3V3", "3.3 V rail: OLED, SHT31, MLX, RC522,\nTCRT x2, TB6612 VCC+STBY, buzzer", "pwr")]

fig, ax = plt.subplots(figsize=(15, 9.5))
ax.set_xlim(0, 15); ax.set_ylim(-2.4, 9.2); ax.axis("off")
bx, by, bw, bh = 6.0, 0.2, 3.0, 8.0
ax.add_patch(FancyBboxPatch((bx, by), bw, bh, boxstyle="round,pad=0.05", fc="#263238", ec="black"))
ax.text(bx + bw / 2, by + bh - 0.45, "ESP32\nDevKit V1\n(30-pin)", ha="center", va="top",
        color="white", fontsize=13, fontweight="bold")
ax.add_patch(FancyBboxPatch((bx + 1.1, by - 0.15), 0.8, 0.45, boxstyle="round,pad=0.02",
                            fc="#b0bec5", ec="black"))
ax.text(bx + 1.5, by + 0.07, "USB", ha="center", va="center", fontsize=8)
step = (bh - 1.6) / (len(LEFT) - 1)
for i, ((pin, desc, c), (rpin, rdesc, rc)) in enumerate(zip(LEFT, RIGHT)):
    y = by + bh - 1.0 - i * step
    ax.plot([bx - 0.05], [y], "o", color="gold", ms=7, mec="black")
    ax.text(bx + 0.15, y, pin, color="white", va="center", fontsize=8.5)
    if desc:
        ax.plot([bx - 0.1, bx - 1.0], [y, y], color=COL[c], lw=2.2)
        ax.text(bx - 1.1, y, desc, ha="right", va="center", fontsize=9, color=COL[c], fontweight="bold")
    ax.plot([bx + bw + 0.05], [y], "o", color="gold", ms=7, mec="black")
    ax.text(bx + bw - 0.15, y, rpin, color="white", va="center", ha="right", fontsize=8.5)
    if rdesc:
        ax.plot([bx + bw + 0.1, bx + bw + 1.0], [y, y], color=COL[rc], lw=2.2)
        ax.text(bx + bw + 1.1, y, rdesc, ha="left", va="center", fontsize=9, color=COL[rc],
                fontweight="bold")

legend = [("Environmental sensors", "sense"), ("I2C bus", "i2c"), ("SPI bus", "spi"),
          ("Motor driver", "motor"), ("Navigation", "nav"), ("Alerts", "out"), ("Power", "pwr")]
for k, (name, c) in enumerate(legend):
    ax.plot([0.3 + k * 2.1, 0.8 + k * 2.1], [-0.55, -0.55], color=COL[c], lw=4)
    ax.text(0.9 + k * 2.1, -0.55, name, va="center", fontsize=9)
ax.text(0.3, -1.25,
        "Power: 2x18650 (7.4 V) -> 2S BMS -> switch -> TB6612 VM (motors)  and  -> LM2596 set to 5.0 V -> ESP32 VIN, "
        "MQ2/MQ135 heaters, HC-SR04, SD module.", fontsize=9.5)
ax.text(0.3, -1.75,
        "Rules: ESP32 pins are 3.3 V max (dividers on MQ AO and HC-SR04 ECHO).  RC522 is 3.3 V only.  "
        "All grounds joined.  Pins 34/35/36/39 are input-only (ADC1).", fontsize=9.5)
ax.set_title("Pin diagram - Multi-Hazard Environmental Patrol Robot", fontsize=15, fontweight="bold")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "pin_diagram.png"), dpi=170)
plt.close(fig)

DOT = r"""
digraph G {
  rankdir=TB; splines=spline; nodesep=0.3; ranksep=0.55; newrank=true;
  node [shape=box, style="rounded,filled", fontname="Helvetica", fontsize=11, fillcolor="#ffffff"];
  edge [fontname="Helvetica", fontsize=9, color="#455a64"];

  subgraph cluster_sense { label="Sensing layer"; style="rounded,filled"; fillcolor="#e8f5e9"; color="#2e7d32"; fontname="Helvetica-Bold";
    sht [label="SHT31\nair temp + RH"]; mlx [label="MLX90614\nwall / socket temp"];
    mq2 [label="MQ2\ngas / smoke"]; mq135 [label="MQ135\nair quality"]; }
  subgraph cluster_nav { label="Navigation layer"; style="rounded,filled"; fillcolor="#e0f7fa"; color="#00838f"; fontname="Helvetica-Bold";
    ir [label="2x TCRT5000\nline following"]; us [label="HC-SR04\nobstacle stop"]; rfid [label="RC522 RFID\ncheckpoint / zone ID"]; }
  subgraph cluster_proc { label="Processing - ESP32 firmware"; style="rounded,filled"; fillcolor="#e3f2fd"; color="#1565c0"; fontname="Helvetica-Bold";
    pre [label="Pre-processing\nADC averaging, median filter,\ndivider correction, validation"];
    feat [label="Feature extraction\nDew point, wall-surface RH (Magnus),\ndT surface, Rs/R0"];
    base [label="Per-zone adaptive baseline\nEWMA mean/var -> z-score\n(frozen during alerts)"];
    cls [label="Hazard classifier\nGas > Hot spot > Mold > Bad air"];
    fsm [label="Patrol state machine\nFOLLOW_LINE / AT_CHECKPOINT /\nWAIT_OBSTACLE / GAS_ALERT"];
    pre -> feat -> base -> cls; {rank=same; pre feat base cls fsm} }
  subgraph cluster_out { label="Output & communication"; style="rounded,filled"; fillcolor="#ffebee"; color="#c62828"; fontname="Helvetica-Bold";
    oled [label="OLED display"]; led [label="RGB LED + buzzer\n(colour / tone per hazard)"];
    sd [label="microSD\nCSV log per zone"]; tg [label="Telegram bot\nvia WiFi"]; }
  subgraph cluster_act { label="Actuation"; style="rounded,filled"; fillcolor="#fff3e0"; color="#e65100"; fontname="Helvetica-Bold";
    drv [label="TB6612FNG"]; mot [label="2x DC motors"]; drv -> mot; }
  subgraph cluster_pwr { label="Power"; style="rounded,filled"; fillcolor="#f5f5f5"; color="#424242"; fontname="Helvetica-Bold";
    bat [label="2x 18650 (7.4 V)\n+ 2S BMS + switch"]; buck [label="LM2596\n7.4 V -> 5 V"]; bat -> buck; }

  {sht mlx mq2 mq135} -> pre;
  {ir us rfid} -> fsm;
  cls -> fsm [xlabel="alert"];
  fsm -> drv;
  cls -> {oled led sd tg};
  bat -> drv [style=dashed, xlabel="7.4 V", constraint=false];
  buck -> sht [style=dashed, xlabel="5 V / 3.3 V to all modules"];
}
"""
p = os.path.join(HERE, "architecture.dot")
open(p, "w").write(DOT)
subprocess.run(["dot", "-Tpng", "-Gdpi=160", p, "-o", os.path.join(HERE, "architecture.png")], check=True)
print("diagrams written")
