# Pedro setup: Pinpoint + 2 pods

**Not tuned yet.** Do not run autonomous until the steps below are complete. A successful build alone does not mean the robot is ready.

## 1. Before you start

1. In Android Studio, build and **install the latest TeamCode app on the Control Hub**. A GitHub push alone does not update the robot. Installing restarts Robot Controller.
2. Connect the computer to the robot's Wi-Fi. Open **[192.168.43.1:10158](http://192.168.43.1:10158)**.
3. Check Robot Configuration: the Pinpoint is an I2C device named `pinpoint`. Its **X** port connects to the forward pod; **Y** connects to the sideways pod. Both pods connect to the Pinpoint, not motor ports.
4. Keep the Driver Station nearby so you can stop the robot. Keep hands clear of wheels.

Mount the Pinpoint sticker-side up. Pedro recommends a Control Hub I2C port **other than port 0** (used by the built-in IMU).

## 2. Check the motors

Run **Mecanum Tuner** with the robot safely raised and drive wheels off the ground. It spins each motor. Follow the screen and stop if the wrong wheel spins. If you change `drivetrainConfig` in `Constants.java`, keep `c.manualBrakeMode.set(true);`, then rebuild and install.

## 3. Tune the Pinpoint

Put the robot on the floor so both pod wheels touch it. Select **Pinpoint Tuner** (not Two Wheel or Three Wheel). Enter `pinpoint` and select your **actual** pod type; the default is not confirmed. For custom pods, enter the measured push distance and follow that step first.

For each stage, confirm the prompt, then **keep the robot still while it initializes**. The stage starts automatically; there is no separate Start button to wait for. Then:

1. Push **forward** 6–12 inches in a straight line. Press **Stop** in the tuner.
2. At the next stage, push **left** 6–12 inches without turning. Press **Stop**.
3. At the offset stage, turn exactly **180° counterclockwise** around the robot's fixed center. Press **Stop**.

These movements are by hand; Pinpoint Tuner does not drive the motors. Watch the Driver Station telemetry: **X ticks** should change during the forward push, **Y ticks** during the left push. Negative ticks are OK.

When finished, copy the Java `localizerConfig` result into `Constants.java`, replacing the existing block. Set `LOCALIZER_TUNED = true` **only after** entering the measured results. Rebuild and install.

## 4. Check position, then tune driving

Run **Tests → Pose Test**. Move the robot by hand: forward should increase X, left should increase Y, and a counterclockwise turn should increase heading. Check a measured distance too. Fix wrong readings before continuing.

Then run **Foresight Tuner** on a clear floor. **It drives the robot, including at full power, and can coast past the requested distance.** Allow plenty of space and be ready to stop it from the Driver Station. Follow the prompts, paste its Java `foresightConfig` into `Constants.java`, set `FORESIGHT_TUNED = true`, then rebuild and install. Try short Line/Curve tests before autonomous.

**Do not confuse Pose Test with Odometry Test:** Odometry Test drives the robot automatically. Mecanum Tuner, Foresight Tuner, and the driving/path tests also power motors.

## If something goes wrong

Turn robot power **off before unplugging or reseating cables**. After checking, power on and retry. If a message persists, take a photo of the **full message** and Pinpoint LED.

| Message or symptom | What to do |
| --- | --- |
| `FAULT_X_POD_NOT_DETECTED` / blue LED | Pinpoint cannot detect the forward pod. Check the pod and cable at the **X** port. |
| `FAULT_Y_POD_NOT_DETECTED` / orange LED | Check the sideways pod and cable at the **Y** port. |
| `FAULT_NO_PODS_DETECTED` / purple LED | Check both pod cables and their Pinpoint ports. |
| `CALIBRATING` or `NOT_READY` | Keep the robot still during initialization and retry. If it persists, record the exact status and check connections. |
| `FAULT_BAD_READ` | Check the Pinpoint's I2C cable and connection. |
| `FAULT_IMU_RUNAWAY` | Keep still during calibration and retry. If it persists, consult the Pinpoint manual. |
| `pinpoint` not found | Check the I2C connection and the exact device name in Robot Configuration. |
| Less than one inch / ticks stay at zero | Wait until initialization finishes, then push 6–12 inches. Check the pod wheel touches the floor and its cable is connected. |
| Other encoder moved / swapped pods | Push straight without turning; check forward pod is X and sideways pod is Y. |
| Movement on both sides | Retry with one straight push; don't roll past the starting point in the opposite direction. |
| Invalid offset measurement | Retry the offset step: keep the center fixed and turn exactly 180° counterclockwise. |
| `Run Pinpoint Tuner` | Complete step 3, copy the measured config and rebuild. Do not guess offsets or just flip the flag. |
| `Run Foresight Tuner` / algorithm required | Complete step 4 and rebuild before follower or autonomous tests. |
| Web page won't open | Check robot power, Robot Controller app and computer Wi-Fi; use `http://192.168.43.1:10158`. |

If an X/Y pod fault remains after checking connections, the pod, cable, or port may need further testing. Code cannot confirm which part is faulty without testing the robot.
