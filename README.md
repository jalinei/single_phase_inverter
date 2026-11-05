# single_phase_inverter

Two independent controllers use SOGI quadrature signals and dq PI loops:

- `singlePhaseInverterFollowing` tracks grid phase with a PLL and controls current through synchronization and connection modes.
- `singlePhaseInverterForming` generates phase from a frequency reference and uses cascaded voltage/current loops.

Each class lives in its own header and source file. Include the specific header,
or `singlePhaseInverter.h` for both. The controller type is chosen when declaring
the object; there is no FORMING/FOLLOWING selector or runtime `setMode()`.

## Initialization and control

```cpp
#include "singlePhaseInverterFollowing.h"
#include "singlePhaseInverterForming.h"

singlePhaseInverterFollowing following;
if (following.init(100.0F, 20.0F, 2.0F * PI * 50.0F, 0.0001F) == 0) {
    following.requestSync();
}

singlePhaseInverterForming forming;
if (forming.init(100.0F, 2.0F * PI * 50.0F, 0.0001F) == 0) {
    forming.setVdqRef({20.0F, 0.0F, 0.0F});
}
```

Following initialization takes bus voltage (V), nominal grid peak voltage (V),
reference angular frequency (rad/s), and sampling period (s). Forming takes bus
voltage, reference angular frequency, and sampling period. Both return `0` on
success, `-EINVAL` for invalid parameters, or the failing PI initialization error.
Parameters must be finite and positive, with representable reciprocals and a
frequency below Nyquist (`w_ref * Ts < pi`). Invalid initialization latches
`VSI_FAULT_INVALID_CONFIGURATION`; control stays disabled until `init()` succeeds. Call `init()` with hardware stopped to reinitialize
filters, PI loops, phase, references, measurements, and faults.

Call `calculateDuty(voltage, current)` once per sampling period with instantaneous
measurements in volts and amperes. It returns the unclamped leg duty
`0.5 + alpha_output / (2 * V_bus)`. The application selects modulation, limits duty,
applies paired bridge PWM updates, and handles PWM enable and relay hardware.

Both controllers provide `getVdq()`, `getIdq()`, `getVdqOut()`, `getVab()`,
`getIab()`, `getVabOutput()`, `getTheta()`, and `getw()`. Frequency is the PLL
estimate for following and the oscillator reference for forming. `getPower()`
returns active power (d, W) and reactive power (q, var). `setVBus()`, `setIdqRef()`,
and `setWRef()` update bus voltage, dq current reference, and angular frequency
reference without clamping. Nonfinite d/q references latch
`VSI_FAULT_INVALID_REFERENCE` immediately and leave the previous reference intact.
Frequency updates must satisfy the same positive/finite/timing rules as initialization;
invalid updates fault without replacing the frequency reference. Following frequency
updates do not retune PLL gains. Both controllers check computed commands, duty,
and power for nonfinite results and latch `VSI_FAULT_INVALID_CONTROL_STATE` on
arithmetic overflow, returning neutral duty (0.5) with PWM requests off.

Forming additionally provides `setVdqRef()` and `getIdqRefDelta()`. The voltage
loop correction is added to its current reference. A nonpositive d-axis voltage
reference resets both loops and commands zero voltage on the next step.
`isPwmRequested()` is true when that reference is positive and no fault is latched.
Both controllers expose `tripFault()`, `getFaultReason()`, and `clearFault()`.
Forming retains its voltage reference when a fault is cleared; the caller must
ensure hardware is ready before clearing it.

## Following operating modes

`vsi_mode_t` names the following controller's operating modes. Read the current
mode with `getMode()` and request transitions using the lifecycle methods.
The previous `vsi_status_t` and `getStatus()` names have been replaced.

| Mode | Behavior |
| --- | --- |
| `VSI_OFF` | No PWM or relay request; current PI states reset. PLL still runs. |
| `VSI_VOLTAGE_SYNC` | Qualify grid voltage, PLL alignment, and frequency continuously. |
| `VSI_GRID_CONNECT` | Match measured grid voltage, request PWM/relay connection, wait for feedback. |
| `VSI_GRID_CONNECTED` | Regulate both current axes to zero, regardless of stored current reference. |
| `VSI_RUN` | Apply the supplied dq current reference directly, without ramping or reference clamping. |
| `VSI_STOPPING` | Keep PWM and relay enabled; regulate both current axes to zero. |
| `VSI_DISCONNECT` | PWM off; request relay opening and wait for open feedback. |
| `VSI_FAULT` | Drop PWM/relay requests immediately; latch until explicitly cleared. |

Call `requestSync()` to start and hold `VSI_VOLTAGE_SYNC` with PWM and relay
requests off, even after synchronization qualifies. It cancels a pending RUN
request and disconnects first if already connecting or connected.
Call `requestConnect()` to allow connection after synchronization. After successful connection,
`requestRun()` requests RUN and `requestStop()` selects connected
zero-current control. A RUN request can be supplied before connecting, but
connection always passes through one zero-current GRID_CONNECTED control step.
`requestDisconnect()` requests disconnection. Any reference ramp, including a
planned power reduction before disconnecting, belongs to the caller.

Synchronization requires positive d voltage within 10% of the configured grid
peak, frequency within 5% of the frequency reference, and `abs(Vq) < 0.1 V` for
1000 consecutive control calls (100 ms at 10 kHz). A failed sample resets the
counter. `getSync()` reports true once this dwell completes, even while
`requestSync()` holds VOLTAGE_SYNC with PWM and relay requests off. A failed
sample or bad external verdict clears qualification; a full dwell is required again.
Changing the frequency reference in VOLTAGE_SYNC also restarts the dwell.
In GRID_CONNECT, alignment is checked on every step, before accepting relay
feedback. Loss of alignment latches `VSI_FAULT_SYNC_LOST` and drops both hardware
requests, including when contact feedback arrives on that same step.
These are control defaults, not a grid protection implementation.
Loss of established connection feedback latches FAULT. Grid quality assessment
belongs to an external observer. Feed its verdict to `setGridQuality(bool)`
from the control task; no grid-quality observer is implemented in this class.
A bad verdict blocks connection while synchronizing. In GRID_CONNECT,
GRID_CONNECTED, RUN, STOPPING, or DISCONNECT, it immediately latches
`VSI_FAULT_BAD_GRID_QUALITY` and drops PWM/relay requests without waiting for
current to decrease. A good verdict alone does not clear FAULT or restart RUN.
The verdict defaults to true after `init()` for operation without an observer;
when monitoring is used, supply the observer's initial verdict before connecting
and refresh it as needed. Unknown/unready observer status can be supplied as false.

`getFaultReason()` reports the first fault cause until a successful `clearFault()`
or `init()`. Causes distinguish bad grid quality, connection/disconnection timeout,
lost connection feedback, invalid measurements, invalid bus voltage, invalid
control state, invalid configuration/references, lost connection-time alignment,
failed PWM updates, and externally reported faults. Applications
can call `tripFault(reason)`; the no-argument call records `VSI_FAULT_EXTERNAL`.
A later fault does not overwrite the first cause. `VSI_FAULT_NONE` means no
latched fault.

Connection feedback must arrive within one second of GRID_CONNECT. During
STOPPING, PWM and relay remain enabled while both current axes target zero.
Once both measured dq axes are below 0.1 A, the mode becomes DISCONNECT:
PWM and relay requests are false while awaiting open contact feedback. Current
rising again does not re-enable PWM or request relay closure in DISCONNECT.
The combined STOPPING/DISCONNECT sequence has a one-second timeout. Timeout latches FAULT.
This describes the existing orderly `requestDisconnect()` path. Emergency
`tripFault(reason)` immediately drops requests and does not follow that current
settling sequence. Actual PWM stopping and relay actuation remain application
hardware actions.

### Application hardware hooks

- `isGridConnectionRequested()` supplies the relay command. In FOLLOWING, this
  is true exactly in GRID_CONNECT, GRID_CONNECTED, RUN, and STOPPING.
- `isPwmRequested()` supplies the PWM enable command. The application applies
  the duty through its paired hardware API and stops PWM when this is false.
  In FOLLOWING, it is derived from the same four states as the relay request;
  DISCONNECT always requests both outputs off.
- `setGridConnected(bool)` supplies confirmed relay contact feedback, independently
  of the requested state. Do not echo the request as physical feedback.
- `clearFault()` succeeds only with open connection feedback and, in FOLLOWING,
  an acceptable external grid-quality verdict. It clears the fault reason and
  leaves the controller OFF; connection and RUN requests must be issued again.
  Invalid initialization requires a successful `init()`; invalid stored bus
  voltage must be corrected before clearing a fault.

The example in `src/main.cpp` supports synchronization only. Serial command `c`
starts and holds `VSI_VOLTAGE_SYNC`, independently of the boost mode. `i` cancels
it and selects idle. Selecting `p`, `b`, or `m` also cancels the synchronization
hold and selects the corresponding boost mode. No command requests grid
connection or RUN. The inverter bridge stays disabled, and the application
has no relay command or contact-feedback integration.

## Validation

Run `bash tests/inverter/run.sh` from the project root. Host tests use the real
SOGI, PLL, transforms, and PI implementation with CMSIS math/logging shims.
They cover both controller types, following sequencing, direct references, and
fault handling, invalid setup/references, arithmetic overflow, sync-only
qualification, and alignment loss before relay closure. Tests enable undefined
behavior and float-to-integer overflow sanitizers. Power-stage behavior and relay timing require hardware validation.

## Dependencies

Requires `transform`, `pid`, `trigo`, and math/type headers from the OwnTech
control library configured in `platformio.ini`.
