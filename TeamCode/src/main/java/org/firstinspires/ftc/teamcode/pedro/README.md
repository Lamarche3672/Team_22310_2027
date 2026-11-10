# Pedro setup 🤖

> **Not ready for autonomous yet.** Finish the steps below first. A successful build does not mean the robot is tuned.

## At a glance

**Install app** → **Check motors** → **Tune Pinpoint** → **Check position** → **Tune Foresight**

## 1️⃣ Get ready

- [ ] In Android Studio, build and **install TeamCode on the Control Hub**. A GitHub push does not update the robot. Installing restarts Robot Controller.
- [ ] Connect your computer to the robot's Wi-Fi.
- [ ] Open **[192.168.43.1:10158](http://192.168.43.1:10158)** in a browser.
- [ ] Keep the Driver Station nearby so you can stop the robot.

### Check the wiring

```text
Forward pod  ───────▶  Pinpoint X
Sideways pod ───────▶  Pinpoint Y
Pinpoint     ── I2C ─▶  Control Hub
```

- In Robot Configuration, name the Pinpoint **`pinpoint`**.
- Plug pods into the **Pinpoint**, not motor ports.
- Mount the Pinpoint **sticker-side up**. Pedro recommends a Control Hub I2C port **other than port 0** (used by its built-in IMU).

## 2️⃣ Check the motors

1. Safely raise the robot so its **drive wheels are off the ground**.
2. Open **Mecanum Tuner**. It spins each motor—keep hands clear.
3. Follow the screen. If the wrong wheel spins, stop and check the wiring/configuration.
4. If you change `drivetrainConfig` in `Constants.java`, keep `c.manualBrakeMode.set(true);`, then rebuild and install.

## 3️⃣ Tune the Pinpoint

Put the robot on the floor so **both pod wheels touch the floor**.

1. Choose **Pinpoint Tuner**—not Two Wheel or Three Wheel.
2. Enter `pinpoint`. Select your **actual** pod type; the default isn't confirmed.
3. For custom pods, enter a measured forward push distance and complete that extra step first.

> **Every stage:** confirm the prompt → **keep still until Driver Station telemetry says READY** → move by hand → press **Stop** in the tuner. Stages start automatically; don't wait for a separate Start button.

| Stage | Move by hand | Check |
| --- | --- | --- |
| Forward | ⬆️ Push **forward** 6–12 inches, straight | X ticks change |
| Strafe | ⬅️ Push **left** 6–12 inches, straight | Y ticks change |
| Offset | ↺ Turn **180° counterclockwise** around a fixed center | Don't slide the center |

Watch X/Y ticks on the Driver Station. **Negative ticks are OK.** Pinpoint Tuner does not drive the motors.

### Save the result

1. Copy the **Java `localizerConfig`** result into `Constants.java`, replacing the old block.
2. Set `LOCALIZER_TUNED = true` **only after** entering those measured results.
3. **Rebuild and install** the app.

## 4️⃣ Check position

Open **Tests → Pose Test**. Move the robot **by hand**:

| Move | Should happen |
| --- | --- |
| ⬆️ Forward | X increases |
| ⬅️ Left | Y increases |
| ↺ Turn counterclockwise | Heading increases |
| 📏 Push a measured distance | Reading matches the distance |

Fix wrong readings before continuing. **Pose Test is not Odometry Test:** Odometry Test drives the robot automatically.

## 5️⃣ Tune driving

> ⚠️ **Foresight Tuner drives the robot, sometimes at full power.** Clear plenty of floor space; it may coast past the requested distance. Be ready to stop it from the Driver Station.

1. Run **Foresight Tuner** and follow the prompts.
2. Copy its Java `foresightConfig` into `Constants.java`.
3. Set `FORESIGHT_TUNED = true`, then **rebuild and install**.
4. Try short Line/Curve tests before autonomous. These tests also drive the robot.

## 🛠️ Something went wrong?

**Power the robot off before unplugging or reseating cables.** Then power on and retry. If it still fails, take a photo of the **full error and Pinpoint LED**.

| What you see | What to try |
| --- | --- |
| **X pod not detected** / blue LED | Check the forward pod and cable at Pinpoint **X**. |
| **Y pod not detected** / orange LED | Check the sideways pod and cable at Pinpoint **Y**. |
| **No pods detected** / purple LED | Check both pod cables and Pinpoint ports. |
| **CALIBRATING** or **NOT_READY** | Keep still during initialization; retry. Check connections if it persists. |
| **FAULT_BAD_READ** | Check the Pinpoint's I2C cable and connection. |
| **FAULT_IMU_RUNAWAY** | Keep still during calibration; retry. If it persists, consult the Pinpoint manual. |
| **`pinpoint` not found** | Check the I2C connection and exact name in Robot Configuration. |
| **Less than one inch** or ticks stay at 0 | Wait for initialization, then push 6–12 inches. Check pod wheel contact and cable. |
| **Other encoder moved** / swapped pods | Push straight without turning. Check forward pod = X, sideways pod = Y. |
| **Movement on both sides** | Retry one straight push. Don't roll past the starting point in the opposite direction. |
| **Invalid offset measurement** | Turn exactly 180° counterclockwise with the center fixed; retry. |
| **Run Pinpoint Tuner** | Finish step 3, copy the measured config and rebuild. Don't guess offsets or just flip the flag. |
| **Run Foresight Tuner** / algorithm required | Finish step 5 and rebuild before follower or autonomous tests. |
| **Web page won't open** | Check robot power, Robot Controller app, and computer Wi-Fi. Use `http://192.168.43.1:10158`. |

If a pod fault remains, the **pod, cable, or port** may need further testing. The message alone cannot tell which part is faulty.
