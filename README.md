# Bela iPad timing characterisation rig

Companion Bela app for measuring the input→display→network latency chain of an
iPad running a Flutter [liblsl.dart](https://github.com/zeyus/liblsl.dart) app.

The Bela is the measurement instrument. It watches two digital sensor inputs —
an FSR sitting over the on-screen button, and a photodiode over a square that
flashes white as soon as the touch is registered — and simultaneously consumes
the iPad's LSL stream. It writes a typed set of CSVs for offline analysis. It
has no outlet.

## What it measures

| | quantity | needs a cross-device clock? |
|---|---|---|
| **T1** | motor → photon (`t_photodiode − t_fsr`) | **no** — Bela frames only, 22.7 µs resolution |
| **T2** | touch → OS report | yes |
| **T3** | OS report → photon | yes |
| **T4** | LSL one-way transport | yes, and confounded — see below |

**T1 is the gold standard**: one hardware clock, nothing about LSL can corrupt
it.

**T4 is fundamentally confounded.** `time_correction` is a round-trip estimate
that assumes path symmetry, so what is actually measured is
`true_one_way − asymmetry/2`. Software alone cannot separate the two. Every
correction is therefore logged with its `uncertainty` (≈ RTT/2), which is a hard
bound on the error — report the bound alongside the number. On wired Ethernet
that bound is ~0.1–0.3 ms; on Wi-Fi it can be 5–50 ms, and characterising that
latency itself is useful.

There is also an **LSL-independent bracket on the clock offset θ**, from the
physical constraints `T2 ≥ 0` and `T3 ≥ D_min` across many trials. It is coarse
(order ±10 ms) but it is a genuine cross-check on `time_correction`, and the
analysis script reports whether the two agree.

## Design notes

Raw frame↔clock sync pairs
and the full `time_correction` series are logged so the mapping can be fitted,
audited and re-fitted offline.

- `render()` (Xenomai RT) only reads pins, detects edges and detects block gaps.
  No clock calls, no allocation, no I/O.
- The **LSL thread blocks** on `pull_sample` and timestamps arrival on the very
  next line, so the arrival stamp is accurate to microseconds rather than to a
  polling interval.
- `samples_available()` is recorded *before* each pull. A non-zero value means
  the sample was already buffered, so that row measures consumer lag rather than
  transport — filter on it before computing T4.
- **Block gaps and underruns are logged.** A dropped audio block means digital
  frames were never read, so an edge could be missing. Without this you cannot
  tell "the iPad never flashed" from "the Bela missed it".
- Every raw edge is logged, tagged with an `accepted` flag against a per-pin
  refractory window. Debouncing is a decision, so it stays reversible offline.

## Layout

```
render.cpp                        the application (ENABLE_LSL=0 for pins-only)
settings.json                     Bela CLArgs; period 16, digital on, analog off
lsl_api.cfg                       liblsl config; set KnownPeers before a session
docs/flutter_outlet_spec.md       the iPad-side contract
analysis/analyse_session.py       computes T1-T4 and the theta bracket
analysis/make_synthetic_session.py  known-ground-truth fixture for the above
render_lsl.cpp.orig               previous prototypes, kept for reference;
render_no_lsl.cpp.orig              the .orig suffix keeps them out of the build
```

Bela compiles every `*.cpp` in the project directory, so there must only ever be
one.

## Build

```sh
./build.sh          # cross-compiles and deploys via ../Bela/scripts
```

Requires the cross-toolchain and sysroot from `SyncBelaSysroot.sh`.

## Output

One directory per session, `logs/<YYYYMMDD_HHMMSS>_<rand>/`:

| file | one row per |
|---|---|
| `*_edges.csv` | digital edge (frame, pin, role, state, active, refractory flags) |
| `*_lsl.csv` | LSL sample (sender ts, arrival clock, arrival frame, backlog, ch0..ch7) |
| `*_sync.csv` | frame↔clock pair with a `bracket_frames` staleness bound |
| `*_timecorr.csv` | `time_correction` measurement with `remote_time` and `uncertainty` |
| `*_status.csv` | session start/end, XRUN, BLOCK_GAP, STREAM_LOST, CLOCK_RESET |
| `*_meta.json` | pin map, rates, liblsl versions, file index |
| `*_stream.xml` | the iPad stream's full LSL header, verbatim |

All schemas are fixed-width — no ragged rows.

```sh
python3 analysis/analyse_session.py logs/20260824_143000_1234
```

## Before a session

- **Wire the Bela to the same switch as the iPads' AP** if at all possible. Over
  Wi-Fi, T4 measures two wireless hops and the RTT/2 bound dominates.
- Set `KnownPeers` in `lsl_api.cfg` to the iPad IPs.
- **Pin the iPad's display refresh rate.** A ProMotion iPad ramping 60→120 Hz on
  touch injects variable latency into every trial.
- **Fix the photodiode patch position** and record it — scanout is row-by-row,
  so vertical position costs up to a full refresh period.
- `ntpdate` the Bela (no battery-backed RTC) so session directories sort. All
  data timestamps are frames or `local_clock()`, so this is cosmetic only.
- Check `*_status.csv` shows zero XRUN/BLOCK_GAP before trusting a session.

## Known confounds

- **The FSR edge is not the moment of contact.** Capacitive touch fires on
  contact; the FSR needs measurable force, which arrives later and depends on tap
  velocity. This biases T1 low and T2 high. Use a consistent mechanical tapper,
  not a finger.
- **The FSR perturbs what it measures** — it desensitises the touchscreen, so
  besides false-triggering it may also delay genuine touch registration. Run a
  bare-finger control block (T3 only) to check.
- **Comparator threshold and pixel rise time** give `t_photodiode` a fixed bias
  (a few ms on LCD, ~1 ms on OLED). It is a bias, not jitter, so it is tolerable
  if measured once and documented.
- Mirroring the sensors to analog inputs would let the force ramp and the
  photodiode rise be seen directly, removing the threshold guesswork. Currently
  digital-only by choice.

# Acknowledgements

- [Christian A. Kothe: liblsl](https://github.com/sccn/liblsl) for the LSL library
- [armlabs: OLED SSD1306 Linux driver](https://github.com/armlabs/ssd1306_linux) for the OLED driver
- [Liam Donovan <liam@bela.io>: Bela](https://bela.io) for the Bela platform
