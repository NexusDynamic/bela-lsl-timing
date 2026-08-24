#!/usr/bin/env python3
"""Generate a synthetic session with known ground truth, to exercise the file
format and analyse_session.py without hardware.

    python3 analysis/make_synthetic_session.py logs/SYNTH_test
    python3 analysis/analyse_session.py       logs/SYNTH_test

Recovered values should match the GROUND TRUTH line this prints.
"""
import json, os, sys
import numpy as np

FS = 44100.0
SLOPE = (1.0 / FS) * (1 + 23e-6)   # 23 ppm audio-clock error
INTERCEPT = 5000.0                 # Bela local_clock at frame 0
THETA = 12.3456                    # true (Bela - iPad) offset, s
T1, T2 = 0.060, 0.020              # true motor->photon, touch->OS
NET = 0.0030                       # true one-way network latency
CORR_DRIFT_PPM = 8.0
N = 60


def main(d: str):
    rng = np.random.default_rng(7)
    os.makedirs(d, exist_ok=True)
    stem = os.path.basename(d.rstrip("/"))
    bela = lambda fr: SLOPE * np.asarray(fr, dtype=float) + INTERCEPT

    fr = np.arange(0, int(120 * FS), int(0.2 * FS))
    lat = rng.exponential(80e-6, fr.size)          # positive-biased sched jitter
    np.savetxt(f"{d}/{stem}_sync.csv",
               np.column_stack([fr, fr + np.round(lat * FS) + 16,
                                np.round(lat * FS) + 16,
                                bela(fr) + lat, bela(fr) + lat - 3000.0]),
               delimiter=",", fmt="%.9f", comments="",
               header="frame_req,frame_latest,bracket_frames,lsl_clock,monotonic_clock")

    edges, lslrows, rx = [], [], 0
    ei = {"fsr": 0, "photodiode": 0}

    def edge(f_, role, state, active):
        ei[role] += 1
        edges.append((int(f_), 0 if role == "fsr" else 1, role, state, active,
                      0, 1, 0, ei[role]))

    start = int(5 * FS)
    tj, dj = rng.normal(0, 0.004, N), rng.normal(0, 0.003, N)
    for i in range(N):
        f0 = start + int(i * 1.5 * FS)
        t1_i = T1 + tj[i] + dj[i]
        edge(f0, "fsr", 1, 1)
        edge(f0 + int(t1_i * FS), "photodiode", 0, 1)   # active-LOW comparator
        tc_ipad = bela(f0) + T2 + tj[i] - THETA
        for code, clk in ((2, tc_ipad), (4, tc_ipad + t1_i - T2 - tj[i])):
            arr = clk + THETA + NET + rng.exponential(3e-4)
            lslrows.append((rx, "ipad-A", "LSLTest-iPad", clk, arr,
                            int((arr - INTERCEPT) / SLOPE), 0, 6,
                            code, clk, i, rx))
            rx += 1
        if i % 5 == 0:                                   # FSR false trigger
            edge(f0 + int(0.7 * FS), "fsr", 1, 1)

    edges.sort(key=lambda r: r[0])
    with open(f"{d}/{stem}_edges.csv", "w") as f:
        f.write("frame,pin,role,state,active,dt_frames_prev,accepted,"
                "suppressed_count,edge_index\n")
        for r in edges:
            f.write(",".join(str(x) for x in r) + "\n")

    with open(f"{d}/{stem}_lsl.csv", "w") as f:
        f.write("rx_index,source_id,stream_name,lsl_timestamp,arrival_lsl_clock,"
                "arrival_frame,samples_available_before,n_channels,"
                + ",".join(f"ch{c}" for c in range(8)) + "\n")
        for r in lslrows:
            f.write(f"{r[0]},{r[1]},{r[2]},{r[3]:.9f},{r[4]:.9f},{r[5]},{r[6]},"
                    f"{r[7]},{r[8]:.9f},{r[9]:.9f},{r[10]:.9f},{r[11]:.9f},0,0,,\n")

    tcl = np.arange(bela(start), bela(start) + 95, 1.0)
    corrs = THETA + CORR_DRIFT_PPM * 1e-6 * (tcl - tcl[0]) + rng.normal(0, 5e-5, tcl.size)
    with open(f"{d}/{stem}_timecorr.csv", "w") as f:
        f.write("lsl_clock,source_id,correction,remote_time,uncertainty,clock_reset,ok\n")
        for c, v in zip(tcl, corrs):
            f.write(f"{c:.9f},ipad-A,{v:.9f},{c-v:.9f},"
                    f"{abs(rng.normal(3e-4, 5e-5)):.9f},0,1\n")

    with open(f"{d}/{stem}_status.csv", "w") as f:
        f.write("frame,lsl_clock,event,detail_num,detail\n")
        f.write(f"0,{INTERCEPT:.9f},SESSION_START,0,{stem}\n")
        f.write(f"{int(120*FS)},{bela(120*FS):.9f},SESSION_END,0,{stem}\n")

    json.dump({"session": stem, "schema_version": 1, "lsl_enabled": True,
               "start_wall_iso_utc": "2026-08-24T14:30:00Z",
               "digital_sample_rate": 44100.0,
               "files": {"edges": f"{stem}_edges.csv", "status": f"{stem}_status.csv",
                         "sync": f"{stem}_sync.csv", "lsl": f"{stem}_lsl.csv",
                         "timecorr": f"{stem}_timecorr.csv"}},
              open(f"{d}/{stem}_meta.json", "w"), indent=2)

    print(f"GROUND TRUTH  T1={T1*1e3:.1f}ms  T2={T2*1e3:.1f}ms  "
          f"T3={(T1-T2)*1e3:.1f}ms  net={NET*1e3:.1f}ms  theta={THETA:+.4f}s  "
          f"clock_err=+23.0ppm  corr_drift=+{CORR_DRIFT_PPM:.1f}ppm")
    print(f"  note: one-way reads ~{(NET+3e-4-CORR_DRIFT_PPM*1e-6*47.5)*1e3:.2f} ms, "
          f"not {NET*1e3:.1f} -- the correction series drifts while the true offset "
          f"here does not, so the drift is correctly subtracted out.")
    print(f"wrote {d}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "logs/SYNTH_test")
