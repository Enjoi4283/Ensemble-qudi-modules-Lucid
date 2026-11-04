# Tabor Lucid + qudi — bench-verified running notes

Notes from integrating a Tabor Lucid signal generator with qudi via the
Ensemble-qudi-modules fork. Everything here was verified on hardware; where
something was not verified it is marked **unverified**.

Test unit: Lucid Portable (`PID 0x6081`), firmware `07CCE000`, identified as
`Tabor Electronics,LS6081P,<serial>,07CCE000`.

RF verification used a scope with limited bandwidth, so tests were kept to
100–200 MHz. Amplitude comparisons near the top of the scope's range are
affected by front-end rolloff and should not be read as source flatness.

---

## 1. Why the custom driver stack

The Lucid cannot be driven over VISA/SCPI-over-TCP. It speaks **raw USB HID**:
output reports of 256 bytes, ASCII command starting at byte offset 3, and
responses arriving as input reports with NUL-byte padding.

Vendor ID `0x168C`. Product IDs: Desktop `0x6002`, Portable `0x6081`,
Benchtop `0x3002`.

The upstream Ensemble driver (`mw_source_tabor.py`) is the practical starting
point, but it ships with defects that prevent it running at all on a Portable
unit. Thirteen changes were applied here; the substantive nine are listed in
section 3 with the evidence that justified each, and the remainder are noted
in section 3.10.

---

## 2. Environment

qudi-core and qudi-iqo-modules do not support Python 3.11+ for this fork, and
the fork pins `python_requires='>=3.8, <3.11'`. Python **3.10** is required.

```
conda create -n qudi310 python=3.10 -y
```

```
conda activate qudi310
```

```
pip install qudi-core qudi-iqo-modules pywinusb numpy scipy
```

Clone the fork and install editable (the `--no-deps` matters — the fork's
`requirements.txt` pins `PySide2==5.15.2.1`, which will not resolve on a
current environment):

```
git clone https://github.com/zhangjixing123/ensemble-qudi-modules %USERPROFILE%\ensemble-qudi-modules
```

```
pip install -e %USERPROFILE%\ensemble-qudi-modules --no-deps
```

Register the Jupyter kernel so the qudi console uses the same interpreter
(without this, the console can attach from a different environment and module
handles resolve to `None`):

```
qudi-install-kernel
```

Launch:

```
qudi -c %USERPROFILE%\ensemble-qudi-modules\LaborConfiguration.cfg
```

Always compile the driver before launching — a syntax error costs a full
startup cycle to discover otherwise:

```
python -m py_compile %USERPROFILE%\ensemble-qudi-modules\src\qudi\hardware\microwave\mw_source_tabor.py
```

---

## 3. The driver patches

All in `src/qudi/hardware/microwave/mw_source_tabor.py`.

### 3.1 Device-ID filter

`on_activate` filtered HID devices on `teLucidBenchtopId` only, so a Portable
or Desktop unit was never found and `on_activate` raised
`IndexError: list index out of range`.

**Evidence:** module activation failed immediately on a Portable unit.

Patch: filter on `teLucidPortableId`.

### 3.2 Read timing in `_command_wait`

The original closed the HID device immediately after registering the
raw-data handler, so a query frequently returned the *previous* command's
response, or nothing. `cw_on()` polls `:OUTP?` in a loop, so it spun forever
holding the module thread lock.

**Evidence:** `cw_on()` hung indefinitely while the RF output was demonstrably
on at the scope.

Patch: clear the response buffer, register the handler **before** sending, wait
up to 1.5 s for a response, then close.

### 3.3 Inline SCPI error detection

The Lucid does **not** populate the SCPI error queue usefully over HID.
Every malformed command tested left `:SYSTem:ERRor?` reading `0`:

| Command sent | `:SYST:ERR?` |
|---|---|
| `:NOTACOMMAND 5` | `0` |
| `*XYZ?` | `0` |
| `:POWer 999` | `0` |
| `:POWer ABC` | `0` |
| `:FOOBAR:BAZ?` | `0` |

Errors instead come back **inline in the next response**, as
`SCPI ERROR. WRONG COMMAND!`. Patch: raise `RuntimeError` when any response
contains `SCPI ERROR`.

**Evidence:** the queue probe above showed all five cases silent; a genuinely
invalid command later returned the inline error string.

### 3.4 Bounded output-state waits

`off()`, `cw_on()` and both `start_scan()` branches polled
`while self._command_wait(':OUTP?') == 'ON'` with no exit path. Replaced with
`_wait_output_state()`, which makes 10 bounded reads then logs a warning and
proceeds.

**Evidence:** the unbounded loops were the deadlock in 3.2.

### 3.5 `_write_list` — command syntax

The upstream implementation sent commands this firmware does not accept:
`LIST:DELete:ALL` (no such command) and `:LIST:DEFine` with **spaces after
commas** (rejected as `Invalid point`).

Working form, verified on hardware:

```
:LIST ON
:LIST:DEF 2,100000000,5,0,0,0.2
:LIST:DEF 3,150000000,5,0,0,0.2
:LIST:DEF 4,200000000,5,1,0,0.2
```

Fields: `<step>,<frequency>,<power>,<last_entry>,<advance>,<dwell>`.
`<dwell>` is in **seconds**. `<advance>` 0 = auto, 1 = trigger.

**Steps start at 2.** Step 1 returns `Invalid point` — it is implicit.

### 3.6 `_write_list` — continuous initiation

The original ended with `:INIT:CONT OFF` and `:TRIG:ADV STEP`, which put the
unit in triggered run mode. With auto-advance entries and no trigger source
connected, the list sat on the first frequency forever.

**Evidence:** a list appeared to output a single steady tone at its first
frequency until `:INIT:CONT ON` was sent, at which point it began stepping.

Patch: `:INIT:CONT ON` for auto-advance lists.

### 3.7 `off()` — unconditional stop

`off()` began with `if self.module_state() != 'idle'`. When module state and
hardware disagreed (inevitable once anything is driven outside the driver),
`off()` returned without sending `:OUTP OFF` and the RF stayed on.

**Evidence:** `mw.off()` left output running; only a raw `:OUTP OFF` stopped it.

Patch: always send `:OUTP OFF`, then unlock only if the module thought it was
busy.

### 3.8 `start_scan` — list routing

The JUMP_LIST branch called `_write_sweep()` — the `_write_list()` call was
commented out immediately above it. List scans therefore drove the sweep
engine.

**Evidence:** list mode set up through the normal qudi API produced sweep-like
behaviour rather than stepping through the configured frequencies.

Patch: call `_write_list()`.

### 3.9 `advance_mode` ConfigOption

Added `advance_mode` (`'AUTO'` default, `'TRIGGER'` alternative). In
`TRIGGER` mode each list entry gets `advance=1`, and the driver sets
`:TRIG:SOUR EXTernal`, `:INIT:CONT OFF`, `:TRIG:ADV STEP` and the configured
edge polarity.

**Unverified** — no trigger source was available at the bench. The code path
matches the manual's semantics but the behaviour has not been observed.

### 3.10 Four smaller changes

- `_write_list` now sends `:LIST ON` to enable list mode, replacing the
  `LIST:DELete:ALL` call that does not exist on this firmware
- `configure_scan` accepts the scan mode as a **string** (`'JUMP_LIST'`,
  `'EQUIDISTANT_SWEEP'`) as well as an enum, because a remote console cannot
  pass the driver's own enum object
- `sample_rate_limits` raised from `(0.1, 100)` to `(0.1, 10000)`. The
  hardware dwell floor is 100 µs, so the original cap throttled the
  instrument by two orders of magnitude
- `freq_limits` changed from the hardcoded `(1e8, 3e9)` to `(9e3, 12e9)`, and
  the `frequency_min` / `frequency_max` options now set the range directly
  rather than only being able to narrow it (`max()`/`min()` previously made a
  configured minimum below 100 MHz impossible)

---

## 4. Firmware behaviour that contradicts the manual

Confirmed on hardware; the Lucid Programming Manual Rev. 1.1 is wrong in each
case.

### 4.1 `:LIST:DEF` rejects spaces

The manual's example is `:LIST:DEF 1, 2e9, 0, 0, 1e-3` — with spaces. The
firmware returns `Invalid point`. Use the compact form with no whitespace.

The manual's example also has only **five** values for a six-parameter
command.

### 4.2 `:LIST:DEF` starts at step 2

Documented range is `2 to 4096`, but the manual's own example uses step 1.
The firmware enforces the range: step 1 → `Invalid point`.

### 4.3 `:LIST?` always returns `OFF`

Documented as returning 1 when generating, 0 when off. It returns `OFF`
regardless. The upstream fork's author documented this; it is correct. List
state must be tracked in software.

### 4.4 Numeric arguments are not always accepted

`:INIT:CONT 1` was accepted silently and changed nothing; `:INIT:CONT ON`
works. The manual lists `{OFF/ON/0/1}` for this command. Prefer mnemonics.

### 4.5 Trigger source has no "internal"

Documented as `{EXTernal/BUS/TIMer/SPI}`. Values `INT`, `INTernal` and
`IMMediate` are all accepted silently and leave the source on External
(default `0`). There is no internal trigger source; `TIMer` (2) is the
software-timed alternative.

`:TRIG:SOUR?` and `:INIT:CONT?` do read back truthfully. `:FRSW?` does too.

---

## 5. Frequency sweep mode — requires triggered run mode

Sweep mode **does work**, but only in triggered run mode driven by the timer
trigger source. In continuous run mode it accepts every command, reports
`:FRSW?` as `ON`, and produces no sweep at all.

### Why continuous mode fails

The manual (6.3) states that the trigger source only affects the generator
*after it has been programmed to operate in Trigger Run mode*. Sweep
apparently requires triggered mode even though list mode does not — the
opposite requirement.

Evidence: with `:INIT:CONT ON` and `:TRIG:SOUR 0` the output stayed at the
sweep start frequency indefinitely, through a clean power cycle and a fresh
module activation, with every parameter reading back correctly. Switching to
`:TRIG:SOUR 2` + `:INIT:CONT OFF` produced movement immediately.

### Working configuration, stepped

```
:TRIG:SOUR 2
:INIT:CONT OFF
:FRSW:STAR 100000000
:FRSW:STOP 200000000
:FRSW:STEP 4
:FRSW:TIME 2
:FRSW:DIR NORM
:FRSW ON
:OUTP ON
```

This produces discrete steps between the configured points, not a ramp. The
sweep engine interpolates into discrete frequencies and steps between them.
There is no analog ramp mode in the manual (sections 10.1–10.6 define only
state, start, steps, stop, time and direction).

### Continuous-looking sweep

Make the steps fine enough that the output is effectively continuous:

```
:TRIG:SOUR 2
:INIT:CONT OFF
:FRSW:STEP 1000
:FRSW:TIME 1e-3
:FRSW ON
:OUTP ON
```

1000 steps across 100–200 MHz is 100 kHz per point at 1 ms each: a full sweep
per second with steps far below the scope's resolution.

Limits: `:FRSW:TIME` floor is 100 µs, `:FRSW:STEP` ceiling is 65535. At
65535 steps and 100 µs dwell, a full-range sweep takes ~6.5 s at ~1.5 kHz
resolution.

### Trigger advance: ONC, not ONCE

`:TRIG:ADV` controls whether one trigger runs the whole sweep or advances a
single step:

- `ONC` — one trigger generates the entire sweep or list
- `STEP` — one trigger generates the current step and advances

`:TRIG:ADV ONCE` and `:TRIG:ADV 1` are both accepted silently and leave the
setting at `STEP`. Only `:TRIG:ADV ONC` takes effect and reads back as `ONC`.

### Readback caveat

`:FRSW?` returns `ON`/`OFF` (configured state), not `1`/`0` (generating state)
as documented in 10.1. It reports `ON` whether or not a sweep is actually
running, so it cannot be used to confirm sweep activity. Verify on the scope.

---

## 6. Working console recipes

The qudi console namespace is cleared on every qudi restart. Rebuild the
handle each session:

```python
mm = qudi.module_manager
mw = mm['tabor_12G'].instance
```

The module name is the key shown in the hardware list, exact case. All work
recorded here used `tabor_12G`; `tabor_3G` was never activated and remains
untested. `.instance` is required — indexing alone returns the manager
wrapper, which has no driver methods.

**Never mix raw `_command_wait(':OUTP ...')` with driver methods.** The module
state and the hardware diverge, after which `set_cw` refuses and `off()`
misbehaves. If that happens, call `mw.off()` (now unconditional), or
deactivate and reactivate the module.

### CW

```python
mw.set_cw(150e6, -20)
mw.cw_on()
```

```python
mw.off()
```

### List scan via the qudi API

```python
mw.configure_scan(5, [100e6, 150e6, 200e6], 'JUMP_LIST', 0.5)
mw.start_scan()
```

Pass the mode as a **string** — `'JUMP_LIST'` or `'EQUIDISTANT_SWEEP'` — not
`mw.scan_mode`, which returns an enum object that fails the remote-console
check (see 6.1). The sample rate is converted to a per-step dwell time in
seconds as `1/rate`.

### List mode, raw

```python
mw._command_wait(':LIST:DEF 2,100000000,5,0,0,0.5')
mw._command_wait(':LIST:DEF 3,150000000,5,0,0,0.5')
mw._command_wait(':LIST:DEF 4,200000000,5,1,0,0.5')
mw._command_wait(':INIT:CONT ON')
mw._command_wait(':OUTP ON')
```

---

### 6.1 Enum arguments cannot cross the rpyc console boundary

Calling `configure_scan` from the console with an explicitly imported
`SamplingOutputMode` enum raises `AssertionError`, because the console's enum
object is not the one the driver holds. Pass the mode as a **string** instead
(`'JUMP_LIST'` or `'EQUIDISTANT_SWEEP'`); the driver coerces it. This is
verified working and is the recommended console form.

---

## 7. Verified vs unverified

| Capability | Status |
|---|---|
| HID enumeration and identity | verified |
| CW: set, on, off, readback | verified on scope |
| List mode: stepping through frequencies | verified on scope |
| List mode via `configure_scan`/`start_scan` | verified |
| Sweep via `_write_sweep` (driver method) | verified on scope |
| Sweep via `configure_scan` with string mode | verified on scope |
| Mode switching list -> sweep -> list | verified |
| Low frequency (1 MHz) after floor change | verified |
| Full regression pass (10 steps) | all passed |
| `off()` stops RF unconditionally | verified |
| Inline SCPI error detection | verified |
| Frequency sweep mode | verified — requires triggered run mode + timer source |
| External TTL triggering | **unverified** — no trigger source at bench |
| Desktop model (`0x6002`) | **unverified** — config change only, no unit tested |
| Sustained operation under load | **unverified** — bench tests were seconds, not hours |

---

## 8. Notes for deployment

- The Desktop unit is a different product class: OEM module, SPI and micro-USB,
  no LAN, no front panel. The driver's HID path should work provided the
  product ID is set, but nothing on this bench confirms it. Do not commit to it
  without testing a unit or a written answer from the vendor.
- `freq_limits` was hardcoded to `(1e8, 3e9)` upstream, regardless of model.
  It is now `(9e3, 12e9)`, and `frequency_min` / `frequency_max` set the range
  directly. Always set them per model: the Portable tops out at 6 GHz, the
  Benchtop at 12 GHz. Do not assume the driver default matches the hardware.
- `teLucidBenchtopId` in the upstream config stanzas is `0x1202`, which matches
  no Lucid product. The correct Benchtop ID is `0x3002`. The driver filters on
  `teLucidPortableId`, so this is inert, but it is misleading.
- `FM_MODE` appears in the upstream config stanzas but the driver never reads
  it.
- The list and sweep state queries are unreliable. Any logic that infers
  instrument state from `:LIST?` will be wrong.
- Error handling must be host-side. Range checks in `MicrowaveConstraints` are
  the only protection against out-of-range set-points, because the instrument
  reports nothing.
