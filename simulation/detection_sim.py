"""
Offline simulation of the patrol robot's hazard-detection pipeline.

The hardware is not wired yet, so this script tests the SAME detection logic as
firmware/patrol_robot/detection.h on a SYNTHETIC dataset:
  * 5 hostel zones with different "normal" conditions
  * 21 days x 6 patrols/day (one checkpoint sweep per zone per patrol)
  * sensor noise taken from datasheet accuracy (SHT31 +-0.3 C / +-2 %RH, MLX90614 +-0.5 C)
  * injected, labelled hazard events (mold, bad air, gas, hot spot)

It compares a single GLOBAL fixed threshold against the PER-ZONE ADAPTIVE baseline
(EWMA mean/variance + z-score, frozen during alerts), computes precision / recall /
F1 / false-alarm rate, a confusion matrix, a feature-correlation matrix and a
Monte-Carlo error-propagation estimate for wall-surface humidity.

Run:  python3 detection_sim.py      (needs numpy, pandas, matplotlib)
Outputs go to ./results/
"""
import os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RNG = np.random.default_rng(42)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
os.makedirs(OUT, exist_ok=True)

# ---------------- physics (identical to detection.h) ----------------
A, B = 17.62, 243.12

def es(t):            return 6.112 * np.exp(A * t / (B + t))
def vp(t, rh):        return rh / 100.0 * es(t)
def dew(t, rh):
    g = np.log(vp(t, rh) / 6.112)
    return B * g / (A - g)
def surface_rh(t, rh, ts): return 100.0 * vp(t, rh) / es(ts)

# ---------------- thresholds (identical to detection.h) ----------------
MOLD_RH, HOT_DELTA, GAS_ABS, GAS_RATIO, Z_LIM = 80.0, 10.0, 2.5, 1.5, 3.0
AIR_RATIO_GLOBAL = 1.3       # the "naive" global rule for bad air
ALPHA, WARMUP, MIN_SD = 0.05, 10, 0.05
LABELS = ["OK", "BAD AIR", "MOLD RISK", "HOT SPOT", "GAS/SMOKE"]

# ---------------- synthetic hostel zones ----------------
ZONES = {
    #                       T_air  RH   wall cooler by  MQ135 V  MQ2 V
    "Z1 Corridor":         (29.0, 62.0, 1.0,            0.80,    0.60),
    "Z2 Near bathroom":    (28.0, 74.0, 1.5,            1.15,    0.62),
    "Z3 Room corner":      (27.5, 66.0, 2.0,            0.85,    0.58),
    "Z4 Pantry / LPG":     (30.0, 60.0, 1.0,            1.00,    0.65),
    "Z5 Electrical panel": (31.0, 55.0, 0.5,            0.78,    0.60),
}
DAYS, PATROLS = 21, 6
EVENT_P = 0.08     # chance a visit contains an injected hazard (after the first 2 days)

rows = []
for day in range(DAYS):
    for p in range(PATROLS):
        hour = 6 + p * 3                                  # 06:00 ... 21:00
        diurnal = 1.5 * np.sin((hour - 9) / 24 * 2 * np.pi)
        for zone, (t0, rh0, wall, aq0, g0) in ZONES.items():
            t_true = t0 + diurnal + RNG.normal(0, 0.4)
            rh_true = np.clip(rh0 - 2 * diurnal + RNG.normal(0, 2.0), 20, 98)
            ts_true = t_true - wall + RNG.normal(0, 0.3)
            aq_true = aq0 * (1 + 0.01 * day / DAYS) + RNG.normal(0, 0.03)   # slow drift
            g_true = g0 + RNG.normal(0, 0.02)
            event = "OK"
            if day >= 2 and RNG.random() < EVENT_P:
                event = RNG.choice(["BAD AIR", "MOLD RISK", "HOT SPOT", "GAS/SMOKE"])
                if event == "BAD AIR":   aq_true *= RNG.uniform(1.4, 1.9)
                if event == "MOLD RISK": ts_true = t_true - RNG.uniform(4.5, 7.0); rh_true = max(rh_true, 70)
                if event == "HOT SPOT":  ts_true = t_true + RNG.uniform(12, 25)
                if event == "GAS/SMOKE": g_true = RNG.uniform(1.0, 3.0)
            # ground truth for mold is physical: true wall RH >= 80 %
            if event == "OK" and surface_rh(t_true, rh_true, ts_true) >= MOLD_RH:
                event = "MOLD RISK"
            # measured values = truth + datasheet-level sensor noise
            rows.append(dict(
                day=day, hour=hour, zone=zone, truth=event,
                t_air=t_true + RNG.normal(0, 0.3 / 2),     # +-0.3 C treated as ~2 sigma
                rh=np.clip(rh_true + RNG.normal(0, 2.0 / 2), 1, 100),
                t_surface=ts_true + RNG.normal(0, 0.5 / 2),
                mq2_v=max(0.05, g_true + RNG.normal(0, 0.01)),
                mq135_v=max(0.05, aq_true + RNG.normal(0, 0.01)),
                rh_surface_true=surface_rh(t_true, rh_true, ts_true),
            ))

df = pd.DataFrame(rows)
df["t_dew"] = dew(df.t_air, df.rh)
df["rh_surface"] = surface_rh(df.t_air, df.rh, df.t_surface)
df["dT_surface"] = df.t_surface - df.t_air

# ---------------- detectors ----------------
mq2_clean = df[df.day == 0].mq2_v.mean()
aq_global_clean = df[df.day == 0].mq135_v.mean()

def classify(r, z_aq, z_g, adaptive):
    if r.mq2_v > GAS_ABS or r.mq2_v > mq2_clean * GAS_RATIO or (adaptive and z_g > Z_LIM):
        return "GAS/SMOKE"
    if r.t_surface - r.t_air > HOT_DELTA:
        return "HOT SPOT"
    if r.rh_surface >= MOLD_RH:
        return "MOLD RISK"
    if adaptive:
        if z_aq > Z_LIM: return "BAD AIR"
    elif r.mq135_v > aq_global_clean * AIR_RATIO_GLOBAL:
        return "BAD AIR"
    return "OK"

class Baseline:
    def __init__(self): self.mean, self.var, self.n = 0.0, 0.0, 0
    def score(self, x, freeze):
        if self.n < WARMUP:
            self.n += 1
            d = x - self.mean
            self.mean += d / self.n
            self.var += (d * (x - self.mean) - self.var) / self.n
            return 0.0
        sd = max(np.sqrt(self.var), MIN_SD)
        z = (x - self.mean) / sd
        if not freeze:
            d = x - self.mean
            self.mean += ALPHA * d
            self.var = (1 - ALPHA) * (self.var + ALPHA * d * d)
        return z

base_aq = {z: Baseline() for z in ZONES}
base_g = {z: Baseline() for z in ZONES}
active = {z: False for z in ZONES}
pred_g, pred_a, zs = [], [], []
for r in df.itertuples():
    za = base_aq[r.zone].score(r.mq135_v, active[r.zone])
    zg = base_g[r.zone].score(r.mq2_v, active[r.zone])
    pa = classify(r, za, zg, adaptive=True)
    active[r.zone] = pa != "OK"
    pred_a.append(pa)
    pred_g.append(classify(r, 0, 0, adaptive=False))
    zs.append(za)
df["pred_global"], df["pred_adaptive"], df["z_mq135"] = pred_g, pred_a, zs

# evaluate after the 2-day learning period
ev = df[df.day >= 2].copy()

def metrics(truth, pred):
    out = {}
    for lab in LABELS[1:]:
        tp = int(((pred == lab) & (truth == lab)).sum())
        fp = int(((pred == lab) & (truth != lab)).sum())
        fn = int(((pred != lab) & (truth == lab)).sum())
        p = tp / (tp + fp) if tp + fp else float("nan")
        rc = tp / (tp + fn) if tp + fn else float("nan")
        f1 = 2 * p * rc / (p + rc) if p == p and rc == rc and p + rc else float("nan")
        out[lab] = dict(TP=tp, FP=fp, FN=fn, precision=p, recall=rc, f1=f1)
    hazard_t, hazard_p = truth != "OK", pred != "OK"
    tn = int((~hazard_t & ~hazard_p).sum()); fp = int((~hazard_t & hazard_p).sum())
    tp = int((hazard_t & hazard_p).sum()); fn = int((hazard_t & ~hazard_p).sum())
    out["ANY HAZARD"] = dict(TP=tp, FP=fp, FN=fn, precision=tp / (tp + fp), recall=tp / (tp + fn),
                             f1=2 * tp / (2 * tp + fp + fn))
    out["_false_alarm_rate"] = fp / (fp + tn)
    out["_accuracy"] = float((truth == pred).mean())
    return out

mg, ma = metrics(ev.truth, ev.pred_global), metrics(ev.truth, ev.pred_adaptive)

tbl = []
for lab in LABELS[1:] + ["ANY HAZARD"]:
    for name, m in (("Global fixed", mg), ("Per-zone adaptive", ma)):
        d = m[lab]
        tbl.append(dict(category=lab, method=name, TP=d["TP"], FP=d["FP"], FN=d["FN"],
                        precision=round(d["precision"], 3), recall=round(d["recall"], 3),
                        f1=round(d["f1"], 3)))
res = pd.DataFrame(tbl)
res.to_csv(os.path.join(OUT, "metrics.csv"), index=False)

# ---------------- feature correlation (feature selection) ----------------
feat = ["t_air", "rh", "t_surface", "t_dew", "rh_surface", "dT_surface", "mq2_v", "mq135_v"]
corr = df[feat].corr()
corr.round(3).to_csv(os.path.join(OUT, "feature_correlation.csv"))

# ---------------- Monte-Carlo error propagation for RH_surface ----------------
N = 20000
t, rh, ts = 28.0, 75.0, 24.0
mc = surface_rh(t + RNG.normal(0, 0.3, N), rh + RNG.normal(0, 2.0, N), ts + RNG.normal(0, 0.5, N))
nominal = surface_rh(t, rh, ts)
# measurement error of the pipeline on the simulated dataset
err = ev.rh_surface - ev.rh_surface_true
mae, rmse, bias = float(err.abs().mean()), float(np.sqrt((err ** 2).mean())), float(err.mean())

# ---------------- plots ----------------
plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})

# 1) MQ135 per zone: global threshold vs zone baselines
fig, ax = plt.subplots(figsize=(10, 4.2))
for zone in ZONES:
    s = df[df.zone == zone]
    ax.plot(s.day + s.hour / 24, s.mq135_v, lw=1, label=zone)
ax.axhline(aq_global_clean * AIR_RATIO_GLOBAL, color="k", ls="--", lw=1.2,
           label="Global fixed limit")
bad = df[df.truth == "BAD AIR"]
ax.scatter(bad.day + bad.hour / 24, bad.mq135_v, marker="x", color="red", s=40, zorder=5,
           label="Injected bad-air event")
ax.set_xlabel("Day"); ax.set_ylabel("MQ135 output (V)")
ax.set_title("MQ135 readings per zone: the bathroom zone sits on a single global limit")
ax.legend(fontsize=8, ncol=3, loc="upper left")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "mq135_zones.png"), dpi=160); plt.close(fig)

# 2) F1 comparison
fig, ax = plt.subplots(figsize=(8, 3.8))
cats = LABELS[1:] + ["ANY HAZARD"]
x = np.arange(len(cats)); w = 0.38
g = [mg[c]["f1"] for c in cats]; a = [ma[c]["f1"] for c in cats]
ax.bar(x - w / 2, g, w, label="Global fixed threshold", color="#9aa5b1")
ax.bar(x + w / 2, a, w, label="Per-zone adaptive (ours)", color="#1f4e79")
for i, (gv, av) in enumerate(zip(g, a)):
    ax.text(i - w / 2, gv + 0.02, f"{gv:.2f}", ha="center", fontsize=8)
    ax.text(i + w / 2, av + 0.02, f"{av:.2f}", ha="center", fontsize=8)
ax.set_xticks(x); ax.set_xticklabels(cats); ax.set_ylim(0, 1.12); ax.set_ylabel("F1-score")
ax.set_title("F1-score by hazard category (synthetic data, days 3-21)")
ax.legend(fontsize=8, loc="lower right")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "f1_comparison.png"), dpi=160); plt.close(fig)

# 3) confusion matrix (adaptive)
cm = pd.crosstab(pd.Categorical(ev.truth, LABELS), pd.Categorical(ev.pred_adaptive, LABELS),
                 dropna=False)
cm.to_csv(os.path.join(OUT, "confusion_matrix_adaptive.csv"))
fig, ax = plt.subplots(figsize=(5.4, 4.6))
ax.imshow(cm.values, cmap="Blues")
for i in range(len(LABELS)):
    for j in range(len(LABELS)):
        v = cm.values[i, j]
        ax.text(j, i, v, ha="center", va="center",
                color="white" if v > cm.values.max() / 2 else "black", fontsize=9)
ax.set_xticks(range(len(LABELS))); ax.set_xticklabels(LABELS, rotation=30, ha="right")
ax.set_yticks(range(len(LABELS))); ax.set_yticklabels(LABELS)
ax.set_xlabel("Predicted"); ax.set_ylabel("True")
ax.set_title("Confusion matrix - per-zone adaptive")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "confusion_matrix.png"), dpi=160); plt.close(fig)

# 4) error propagation histogram
fig, ax = plt.subplots(figsize=(7, 3.6))
ax.hist(mc, bins=60, color="#1f4e79", alpha=0.85)
ax.axvline(MOLD_RH, color="red", ls="--", label="80 % mold threshold")
ax.axvline(nominal, color="k", lw=1, label=f"nominal {nominal:.1f} %")
ax.set_xlabel("Computed wall-surface RH (%)"); ax.set_ylabel("count")
ax.set_title(f"Monte-Carlo error propagation: {nominal:.1f} % +- {mc.std():.1f} % (1 sigma)")
ax.legend(fontsize=8)
fig.tight_layout(); fig.savefig(os.path.join(OUT, "error_propagation.png"), dpi=160); plt.close(fig)

# 5) correlation heatmap
fig, ax = plt.subplots(figsize=(6.2, 5.2))
im = ax.imshow(corr.values, cmap="RdBu_r", vmin=-1, vmax=1)
for i in range(len(feat)):
    for j in range(len(feat)):
        ax.text(j, i, f"{corr.values[i, j]:.2f}", ha="center", va="center", fontsize=7)
ax.set_xticks(range(len(feat))); ax.set_xticklabels(feat, rotation=45, ha="right")
ax.set_yticks(range(len(feat))); ax.set_yticklabels(feat)
fig.colorbar(im, ax=ax, shrink=0.8); ax.set_title("Feature correlation (feature selection)")
fig.tight_layout(); fig.savefig(os.path.join(OUT, "feature_correlation.png"), dpi=160); plt.close(fig)

df.to_csv(os.path.join(OUT, "synthetic_dataset.csv"), index=False)

# ---------------- summary ----------------
lines = [
    "PATROL ROBOT - DETECTION SIMULATION (synthetic data, seed 42)",
    f"Records: {len(df)}  (5 zones x {DAYS} days x {PATROLS} patrols); evaluated: {len(ev)} (days 3-{DAYS})",
    f"Injected / physical hazard visits in evaluation set: {(ev.truth != 'OK').sum()}",
    "",
    "Overall (any hazard vs OK):",
    f"  Global fixed : precision {mg['ANY HAZARD']['precision']:.3f}  recall {mg['ANY HAZARD']['recall']:.3f}"
    f"  F1 {mg['ANY HAZARD']['f1']:.3f}  false-alarm rate {mg['_false_alarm_rate']*100:.1f} %  accuracy {mg['_accuracy']*100:.1f} %",
    f"  Per-zone EWMA: precision {ma['ANY HAZARD']['precision']:.3f}  recall {ma['ANY HAZARD']['recall']:.3f}"
    f"  F1 {ma['ANY HAZARD']['f1']:.3f}  false-alarm rate {ma['_false_alarm_rate']*100:.1f} %  accuracy {ma['_accuracy']*100:.1f} %",
    "",
    "Per category:",
    res.to_string(index=False),
    "",
    "Wall-surface RH measurement error vs ground truth (evaluation set):",
    f"  MAE {mae:.2f} %RH   RMSE {rmse:.2f} %RH   bias {bias:+.2f} %RH",
    f"Monte-Carlo propagation at 28 C / 75 % / wall 24 C: {nominal:.1f} % +- {mc.std():.1f} % (1 sigma), "
    f"P(below 80 %) = {(mc < 80).mean()*100:.2f} %",
    "",
    "Highly correlated feature pairs (|r| > 0.9):",
]
for i in range(len(feat)):
    for j in range(i + 1, len(feat)):
        if abs(corr.values[i, j]) > 0.9:
            lines.append(f"  {feat[i]} ~ {feat[j]}: r = {corr.values[i, j]:.2f}")
summary = "\n".join(lines)
open(os.path.join(OUT, "summary.txt"), "w").write(summary + "\n")
print(summary)
