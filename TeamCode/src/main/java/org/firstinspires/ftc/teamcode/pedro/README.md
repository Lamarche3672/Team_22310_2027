# Pedro: 2 Pods + Pinpoint

**Team 22310 · Pedro 3**

> The build passes. **The robot still needs tuning before autonomous.**

## What's fixed

| File | What changed |
| --- | --- |
| `Constants.java` | Waits up to 3 extra seconds for Pinpoint calibration. Stops and brakes the motors if initialization fails. |
| `procedures/PinpointTuner.java` | Custom-pod tuning now uses real encoder counts, not calculated position. |
| Direction tests | Use raw encoder travel and remember the largest push, even if you roll back to the starting point. Show X/Y ticks and Pinpoint status. |

## Wiring

| Part | Connect to |
| --- | --- |
| Forward pod | Pinpoint **X** input |
| Sideways pod | Pinpoint **Y** input |
| Pinpoint | REV Hub **I2C** port; name it `pinpoint` |

- Pods plug into the **Pinpoint**, not motor encoder ports.
- Pinpoint uses its own IMU. Mount it sticker-side up, following its manual.
- Motor names: `fl_motor`, `fr_motor`, `bl_motor`, `br_motor`.

## Before tuning

- [ ] Build and install the Robot Controller app.
- [ ] Connect to the robot's Wi-Fi.
- [ ] Open **http://192.168.43.1:10158**.

## Tune in this order

**After each change to `Constants.java`, rebuild and install before the next step.**

### 1. Motors — Mecanum Tuner

- Raise the robot so the drive wheels are clear of the floor.
- Check motor names and directions.
- If changed, paste `drivetrainConfig` into `Constants.java`.
- Keep `c.manualBrakeMode.set(true);`.

### 2. Pods — Pinpoint Tuner

**Use Pinpoint Tuner, NOT Two Wheel Tuner or Three Wheel Tuner.**

- Select your actual pod type: 4-bar, swingarm, or custom. The default is not confirmed.
- Keep the robot **still during initialization/calibration**.
- After confirming a stage, it starts automatically. Keep still through initialization, then do the requested hand movement. Press **Stop** to save each measurement; don't wait for a separate Start button.
- Direction tests: push **6-12 inches forward**, then **6-12 inches left** in the next stage. Do not turn.
- Watch **X/Y encoder travel (ticks)** on the Driver Station. X should change for forward; Y for left. A negative count is OK; the tuner will set the reversal.
- For offsets: turn exactly **180° counterclockwise**, keeping the robot's center fixed.
- Paste the Java `localizerConfig` into `Constants.java`.
- Set `LOCALIZER_TUNED = true` only after entering those measured values.

*This tuner does not power the drive motors.*

### 3. Check position — Tests → Pose Test

Move the robot by hand:

| Movement | Expected reading |
| --- | --- |
| Forward | X increases |
| Left | Y increases |
| Turn counterclockwise | Heading increases; X/Y barely change |
| Push a measured distance | The same distance in inches |

**Fix incorrect readings before continuing.**

### 4. Driving — Foresight Tuner

> **This tuner drives the robot, including at full power.** Clear the floor and be ready to Stop.

- Paste its Java `foresightConfig` into `Constants.java`.
- Set `FORESIGHT_TUNED = true` after entering the measured values.
- Try short Line/Curve tests before the sample autonomous in `PedroAuto.java`.

## If setup stops

| Message | What to do |
| --- | --- |
| `FAULT_X_POD_NOT_DETECTED` | The Pinpoint reports that the forward pod is not detected. Power off the robot, then check the forward pod and its cable at the Pinpoint **X** port. Retry; if it persists, diagnose the pod, cable and port. |
| `FAULT_Y_POD_NOT_DETECTED` / `FAULT_NO_PODS_DETECTED` | Power off and check the affected pod cables and Pinpoint ports. |
| `CALIBRATING` / `NOT_READY` | Keep still during calibration and retry. If it persists, record the exact status and check connections. |
| `FAULT_BAD_READ` | Check the Pinpoint I2C connection with robot power off; retry. |
| Travel less than one inch | After initialization, push in the requested direction. If ticks stay at 0, check pod wheel contact and the cable. |
| Other encoder moved | Check for swapped X/Y pod cables and push in the requested direction. |
| Movement on both sides | Repeat one straight push without crossing behind the starting point. |
| Pinpoint reported a fault | Fix the displayed pod/I2C fault and retry. The tuner will not save a direction from bad readings. |
| Run Pinpoint Tuner | Finish step 2. Do not guess offsets or just switch the flag to `true`. |
| Run Foresight Tuner | Finish step 4 before autonomous. |

**Units:** distances = inches · headings = radians · custom resolution = ticks per inch.

**A successful build does not mean the robot is calibrated.**
