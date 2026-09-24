# Experimental Flex Gait and Front/Rear Coordination Design

## Goal

Add an Experimental Flex gait, an independent physical Front/Rear direction
selector, and neutral-hold turning across Firmware, the Pi application and
gateway, and the Qt console. Preserve the existing safety, authority,
ownership, and transition paths.

## Frozen gait semantics

The backend wire values are `SimpleGait = 0`, `CPG = 1`, and
`ExperimentalFlex = 2`. Each generator emits raw logical joint targets. The
MotionManager applies the same common policy after every successful sample:

1. `SameDirection` leaves the generator's Front/Rear relation unchanged.
   `OppositeDirection` negates only RearRight and RearLeft.
2. `TurnLeft` sets FrontLeft and RearLeft to logical zero;
   `TurnRight` sets FrontRight and RearRight to logical zero.
3. Existing mode/start transition blending runs over these policy-adjusted
   targets, followed by the existing envelope sanitizer and ServoService write.

Turned-off paddles remain in the active Motion write mask and under Motion
ownership. Selector changes are admitted only while STOPPED and do not write
servos, acquire or release ownership, or start Motion.

SimpleGait keeps its 0.5 Hz, ±1000 cdeg sinusoid with `Front = q`,
`Rear = -q`; turn modes no longer scale either side. CPG keeps its existing
core equations, graph, phase/integration ordering, and Front/Rear source
polarity. Its adapter no longer changes oscillator amplitude for turn modes.

Experimental Flex has a 2000 ms canonical cycle: a 1300 ms power half-cosine
from `-1000` to `+1000` cdeg, then a 700 ms recovery half-cosine from `+1000`
to `-1000` cdeg. Reset phase is the power-stroke midpoint (`q = 0`). Logical
mapping is FrontRight/FrontLeft `= q` and RearRight/RearLeft `= -q` before
common coordination. STOP, FORWARD, TURN_LEFT, TURN_RIGHT, ASCEND, and DESCEND
are supported; BACKWARD remains invalid. FrontAxis ASCEND/DESCEND bias remains
±1000 cdeg.

## Wire and controller contract

Protocol V2 `SetGaitBackend (0x16)` remains a one-byte payload and accepts
backend value 2. `SetFrontRearCoordination (0x17)` is a new one-byte command:
0 is SameDirection and 1 is OppositeDirection. No Protocol V2 version or ACK
format changes.

RBRP adds command kind `SetFrontRearCoordination = 0x09`, encoded as the
existing command-kind byte followed by the coordination byte. Existing
command-kind values and the Backward qualification gate remain unchanged.

Direct and remote Qt controllers expose requested, confirmed, and pending
coordination state using the existing gait-selector ACK lifecycle. Confirmed
selector values remain unknown until locally ACK-confirmed; no selector query
or telemetry is introduced. Both selectors reject local requests during
active Motion or while either selector is pending. The UI enables them only
with active control, inactive Motion, and no pending selector. Selection never
starts Motion.

## Safety and exclusions

Keep the 750 ms Start/Mode/Stop transitions, Stop Motion, Disable All,
heartbeat fail-safe, Control Authority, Motion servo ownership, operational
envelope, and fail-closed hardware-failure behavior. Do not change CPG core
math, Depth control, Backward/Reverse, Brake, Emergency Stop, or add hardware
DOFs.

## Verification

Firmware host tests cover the Flex waveform/reset/modes, generator-independent
coordination and turning, ownership/mask preservation, transition behavior,
selector lifecycle, and Protocol V2 parsing. Pi CMake/CTest covers codecs,
onboard submission, RBRP validation, gateway mapping, and Linux port forwarding.
Qt tests cover Protocol V2, direct/remote ACK lifecycle and gating, and the
Gait panel. Run the full Firmware host gate, Pi CTest, Qt configure/build/CTest,
and ARM firmware build if the toolchain is available. Report each evidence
surface separately; host tests do not imply target or physical verification.
