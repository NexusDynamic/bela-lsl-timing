# Flutter (iPad) LSL outlet spec

Contract between the iPad app and `render.cpp` on the Bela. The Bela side reads
`channel_count` from the stream info and logs every channel generically, so it
will not break if this changes — but the offline analysis assumes this layout.

## Stream

| property | value |
|---|---|
| `name` | must contain `LSLTest` (the Bela's `kStreamPrefixFilter`) |
| `type` | `Timing` |
| `channel_format` | `cf_double64` |
| `channel_count` | 6 |
| `nominal_srate` | `IRREGULAR_RATE` |
| `source_id` | **stable per-device string** (e.g. `ipad-<vendorid>`) |

`source_id` must be stable and non-empty: it is what makes the inlet
recoverable across a Wi-Fi dropout, and it is the join key in the logs.

## Channels

| ch | name | contents |
|---|---|---|
| 0 | `event_code` | 1=`SESSION_START`, 2=`TOUCH_REGISTERED`, 3=`FLASH_REQUESTED`, 4=`FLASH_PRESENTED`, 5=`SESSION_END` |
| 1 | `event_clock` | `lsl_clock_ex()` at the instant the event was **observed** |
| 2 | `trial` | trial counter, shared by every event of one trial |
| 3 | `seq` | session-wide sample counter, +1 per pushed sample |
| 4 | `aux_a` | `TOUCH_REGISTERED`: `PointerEvent.timeStamp` in the LSL epoch. `FLASH_PRESENTED`: `FrameTiming.vsyncStart` |
| 5 | `aux_b` | `FLASH_PRESENTED`: `FrameTiming.rasterFinishWallTime`. Otherwise 0 |

A `double` represents integers exactly up to 2^53, so codes and counters are
safe. Unused fields are 0, never NaN (NaN would round-trip through CSV badly).

## Rules that carry most of the value

**1. Push with an explicit timestamp.**

```dart
outlet.pushSample(data, timestamp: eventClock);   // NOT push-time
```

Channel 1 then deliberately duplicates the timestamp. That redundancy is the
point: any divergence between the LSL timestamp and `event_clock` exposes a
timestamping problem in the wrapper rather than hiding it in the data.

**2. `PointerEvent.timeStamp` (ch4) is the highest-value extra field.**

It is the OS's hardware event timestamp, so it splits "touch reached iOS" from
"Flutter handled it in Dart" — turning T2 from one number into two. Document
the epoch conversion used (on iOS it derives from `CACurrentMediaTime`), and
capture the conversion constant once per session rather than per event.

**3. `trial` and `seq` (ch2/ch3) remove all guesswork from offline pairing.**

The FSR sits over the button and can both false-trigger and desensitise the
touchscreen, so not every FSR edge has a matching touch. With `trial`, an FSR
edge with no matching trial is *unambiguously* a false trigger rather than a
judgement call about time windows. `seq` gaps reveal dropped samples.

## Also required

- **Push immediately.** No coalescing, no batching. Keep the outlet's
  `max_buffered` small (a chunk size of 1).
- **`FLASH_PRESENTED` is necessarily reported late** — `FrameTiming` only
  arrives via `addTimingsCallback` after the frame. That is fine: its
  `event_clock` is what matters, not when it was pushed.
- **Put session context in the `desc` XML.** The Bela writes the full stream
  header verbatim to `<session>_stream.xml`, so anything here lands in the
  record:

```dart
final desc = info.desc();
desc.appendChildValue('device_model',      'iPad Pro 11 M4');
desc.appendChildValue('os_version',        '18.5');
desc.appendChildValue('app_version',       '...');
desc.appendChildValue('display_refresh_hz','120');   // pin it, see below
desc.appendChildValue('patch_position',    'top-left, 0-120px from top');
desc.appendChildValue('patch_size_px',     '200x200');
```

## Experiment-side requirements

These bound what the numbers mean and cannot be fixed in software afterwards:

- **Pin the display refresh rate.** A ProMotion iPad idling at 60 Hz and
  ramping to 120 Hz on touch injects variable latency into every trial.
- **Fix the photodiode patch position** and record it. Panel scanout is
  row-by-row, so vertical position adds a fixed offset of up to one refresh
  period. Top of screen is cleanest.
- **Separate trials by > 1 s** so pairing is never ambiguous even if `trial` is
  ever missing.
- **Keep the white patch on for a fixed duration** (~100 ms). The falling edge
  gives a second, independent check on the same trial.
