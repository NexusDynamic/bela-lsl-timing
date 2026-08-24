#!/usr/bin/env python3
"""Analyse one Bela timing session directory.

    python3 analysis/analyse_session.py logs/20260824_143000_1234

Computes:
  T1  motor -> photon     = t_photodiode - t_fsr          (Bela frames only)
  T2  touch -> OS report  = (touch_clock + theta) - t_fsr
  T3  OS report -> photon = t_photodiode - (touch_clock + theta)
  T4  LSL one-way         = arrival - (sender_ts + correction)

plus an LSL-independent bracket on the clock offset theta, from the physical
constraints T2 >= 0 and T3 >= D_MIN, as a cross-check on time_correction.
"""

import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Minimum plausible OS-report -> photon latency, seconds. Used only for the
# upper end of the theta bracket; set from your own display measurements.
D_MIN = 0.004

# Event codes, must match docs/flutter_outlet_spec.md
TOUCH_REGISTERED = 2
FLASH_PRESENTED = 4

# Max gap between an FSR assert and the photodiode assert of the same trial.
# A motor->photon latency beyond this would be pathological; keeping it tight is
# what stops a false FSR trigger from being matched to the next trial's flash.
T1_WINDOW_S = 0.30

# Max gap when matching an LSL touch event to its FSR edge, after the touch has
# been mapped into Bela time.
PAIR_WINDOW_S = 0.30


def load(session: Path):
    meta = json.loads((session / f"{session.name}_meta.json").read_text())
    out = {"meta": meta}
    for key, name in meta["files"].items():
        p = session / name
        out[key] = pd.read_csv(p) if p.exists() else None
    return out


def fit_frame_to_clock(sync: pd.DataFrame):
    """Fit lsl_clock ~ frame using only tightly-bracketed sync pairs.

    bracket_frames upper-bounds how stale the frame counter was when the clock
    was read, so filtering on it removes the scheduling-latency bias rather
    than assuming it away.
    """
    if sync is None or len(sync) < 2:
        raise SystemExit("not enough sync rows to fit the frame->clock map")

    cutoff = max(sync.bracket_frames.quantile(0.25), sync.bracket_frames.min())
    tight = sync[sync.bracket_frames <= cutoff]
    if len(tight) < 2:
        tight = sync

    x = tight.frame_req.to_numpy(dtype=float)
    y = tight.lsl_clock.to_numpy(dtype=float)
    slope, intercept = np.polyfit(x, y, 1)
    resid = y - (slope * x + intercept)

    nominal = 1.0 / 44100.0
    print("Frame -> LSL clock map")
    print(f"  sync rows            : {len(sync)} ({len(tight)} used, "
          f"bracket <= {cutoff:.0f} frames)")
    print(f"  fitted frame period  : {slope*1e9:.4f} ns "
          f"({(slope/nominal - 1)*1e6:+.1f} ppm vs 44100 Hz)")
    print(f"  residual std / max   : {resid.std()*1e6:.1f} us / "
          f"{np.abs(resid).max()*1e6:.1f} us")
    if resid.std() > 1e-3:
        print("  WARNING: residuals > 1 ms; the sync task may be starved.")
    return slope, intercept


def frames_to_clock(frames, slope, intercept):
    return slope * np.asarray(frames, dtype=float) + intercept


def sensor_events(edges: pd.DataFrame, role: str):
    """Accepted asserting edges for one sensor role."""
    e = edges[(edges.role == role) & (edges.accepted == 1) & (edges.active == 1)]
    return e.frame.to_numpy(dtype=np.int64)


def report_stat(label, values, unit="ms", scale=1e3):
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if v.size == 0:
        print(f"  {label:<26} n=0")
        return
    v = v * scale
    print(f"  {label:<26} n={v.size:<5d} "
          f"median={np.median(v):8.2f} {unit}  "
          f"mean={v.mean():8.2f}  sd={v.std():7.2f}  "
          f"[{v.min():.2f}, {v.max():.2f}]")


def main(session: Path):
    d = load(session)
    meta = d["meta"]
    edges, sync = d.get("edges"), d.get("sync")
    lsl, corr, status = d.get("lsl"), d.get("timecorr"), d.get("status")

    print(f"Session {meta['session']}  (LSL {'on' if meta['lsl_enabled'] else 'off'})")
    print(f"  started {meta['start_wall_iso_utc']}\n")

    # ---- data integrity first: nothing below is trustworthy without this ----
    print("Integrity")
    if status is not None and len(status):
        counts = status.event.value_counts().to_dict()
        for k in ("XRUN", "BLOCK_GAP", "QUEUE_FULL", "STREAM_LOST", "CLOCK_RESET"):
            if counts.get(k):
                print(f"  !! {k}: {counts[k]}  -- data has gaps, treat with care")
        end = status[status.event == "SESSION_END"]
        print(f"  clean shutdown       : {'yes' if len(end) else 'NO'}")
        if len(end):
            print(f"  dropped events       : {int(end.detail_num.iloc[-1])}")
    print()

    slope, intercept = fit_frame_to_clock(sync)
    print()

    # ---- T1: the gold standard, no cross-device clock involved -------------
    fsr = sensor_events(edges, "fsr")
    photo = sensor_events(edges, "photodiode")
    fs = float(meta.get("digital_sample_rate") or 44100.0)

    t1, t1_fsr, t1_photo = [], [], []
    window = T1_WINDOW_S * fs
    used = set()
    for p_ in photo:
        prev = fsr[(fsr < p_) & (p_ - fsr <= window)]
        if prev.size:
            f = prev[-1]              # nearest preceding FSR edge
            t1.append((p_ - f) / fs)
            t1_fsr.append(f)
            t1_photo.append(p_)
            used.add(int(f))

    print("T1  motor -> photon   (Bela hardware clock only)")
    report_stat("t_photo - t_fsr", t1)
    print(f"  FSR edges={fsr.size}  photodiode edges={photo.size}  paired={len(t1)}")
    if fsr.size:
        unpaired = fsr.size - len(used)
        print(f"  unpaired FSR edges   : {unpaired} "
              f"({100*unpaired/fsr.size:.0f}% -- expected, the FSR false-triggers)")
    if photo.size and len(t1) < photo.size:
        print(f"  flashes with no FSR  : {photo.size - len(t1)} "
              f"(touch registered without tripping the FSR)")
    print()

    if not meta["lsl_enabled"] or lsl is None or not len(lsl):
        print("No LSL data in this session; T2/T3/T4 unavailable.")
        return

    # ---- T4: LSL one-way, confounded by path asymmetry --------------------
    corr_ok = corr[corr.ok == 1] if corr is not None else None
    print("LSL transport")
    if corr_ok is None or not len(corr_ok):
        print("  no successful time_correction measurements")
        return

    theta_lsl = np.interp(lsl.arrival_lsl_clock.to_numpy(),
                          corr_ok.lsl_clock.to_numpy(),
                          corr_ok.correction.to_numpy())
    clean = lsl.samples_available_before == 0
    t4 = lsl.arrival_lsl_clock.to_numpy() - (lsl.lsl_timestamp.to_numpy() + theta_lsl)

    report_stat("apparent one-way (all)", t4)
    report_stat("apparent one-way (clean)", t4[clean.to_numpy()])
    print(f"  clean arrivals       : {int(clean.sum())}/{len(lsl)} "
          f"(samples_available_before == 0)")
    if clean.sum() < 0.5 * len(lsl):
        print("  WARNING: most arrivals were backlogged; T4 is not usable.")

    unc = corr_ok.uncertainty.to_numpy()
    print(f"  correction           : median {np.median(corr_ok.correction)*1e3:+.3f} ms, "
          f"drift {np.polyfit(corr_ok.lsl_clock, corr_ok.correction, 1)[0]*1e6:+.2f} ppm")
    print(f"  uncertainty (~RTT/2) : median {np.median(unc)*1e3:.3f} ms, "
          f"max {unc.max()*1e3:.3f} ms")
    print("  ^ this bounds the one-way error: time_correction assumes path")
    print("    symmetry, so T4 = true_one_way - asymmetry/2. Report the bound.")
    print()

    # ---- T2 / T3 and the LSL-independent theta bracket --------------------
    touch = lsl[lsl.ch0 == TOUCH_REGISTERED]
    if not len(touch):
        print("No TOUCH_REGISTERED events; T2/T3 unavailable.")
        return

    touch_clock = touch.ch1.to_numpy()          # iPad clock
    # Interpolate the correction at the row's own arrival time, which is already
    # on the Bela clock -- interpolating at an iPad-clock value would index the
    # series by the wrong axis.
    touch_theta = np.interp(touch.arrival_lsl_clock.to_numpy(),
                            corr_ok.lsl_clock.to_numpy(),
                            corr_ok.correction.to_numpy())
    touch_bela = touch_clock + touch_theta      # mapped into Bela time

    fsr_clock = frames_to_clock(t1_fsr, slope, intercept)
    photo_clock = frames_to_clock(t1_photo, slope, intercept)

    pair_f, pair_p, pair_t, pair_th = [], [], [], []
    for tb, tc, th in zip(touch_bela, touch_clock, touch_theta):
        if not len(fsr_clock):
            break
        j = int(np.argmin(np.abs(fsr_clock - tb)))
        if abs(fsr_clock[j] - tb) <= PAIR_WINDOW_S:
            pair_f.append(fsr_clock[j])
            pair_p.append(photo_clock[j])
            pair_t.append(tc)
            pair_th.append(th)
    pair_f = np.array(pair_f); pair_p = np.array(pair_p)
    pair_t = np.array(pair_t); pair_th = np.array(pair_th)

    print(f"T2 / T3   (paired trials: {len(pair_t)} of {len(touch)} touch events)")
    if not len(pair_t):
        print("  nothing paired")
        return

    theta = pair_th
    report_stat("T2 touch -> OS report", (pair_t + theta) - pair_f)
    report_stat("T3 OS report -> photon", pair_p - (pair_t + theta))
    print()

    lo = np.max(pair_f - pair_t)
    hi = np.min(pair_p - pair_t) - D_MIN
    med = float(np.median(theta))
    print("theta bracket from physics alone (independent of LSL)")
    print(f"  T2 >= 0        => theta >= {lo:+.6f} s")
    print(f"  T3 >= {D_MIN*1e3:.0f} ms   => theta <= {hi:+.6f} s")
    print(f"  width                : {(hi-lo)*1e3:.2f} ms")
    print(f"  LSL time_correction  : {med:+.6f} s  -> "
          f"{'INSIDE bracket' if lo <= med <= hi else 'OUTSIDE BRACKET (!)'}")
    if not (lo <= med <= hi):
        print("  The network path is badly asymmetric, or an assumption above is")
        print("  wrong. Do not report T2/T3 without investigating.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    main(Path(sys.argv[1]))
