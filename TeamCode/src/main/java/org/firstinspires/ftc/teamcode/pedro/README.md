# Team 22310: two pods + goBILDA Pinpoint

This setup uses Pedro 3's **PinpointLocalizer**. The Pinpoint reads both
pod encoders and its own IMU. Do not use Two Wheel Tuner or Three Wheel Tuner
for this hardware. Those unused procedure files are left in place.

## Hardware

- Connect the forward/parallel pod to the Pinpoint X encoder input.
- Connect the sideways/perpendicular pod to the Pinpoint Y encoder input.
- Connect the Pinpoint to a REV Hub I2C port. In Robot Configuration, select
  the goBILDA Pinpoint device and name it `pinpoint`, or enter its actual name
  in the tuner. Pod encoders do **not** use the drive-motor encoder ports.
- Mount the Pinpoint according to its goBILDA instructions (sticker side up).
- Select the actual pod type: goBILDA 4-bar, swingarm, or custom. The 4-bar
  value in Constants.java is only an initial selection, not a confirmed fact.
- Existing drive motor names are `fl.motor`, `fr.motor`, `bl.motor`, `br.motor`.

## Tuning order

1. Build/install the Robot Controller code. Connect to the robot's Wi-Fi and
   open **http://192.168.43.1:10158** in a browser.
2. Verify motor names/directions with **Mecanum Tuner** with the robot safely
   raised. If needed, paste its drivetrainConfig into Constants.java, retaining
   `c.manualBrakeMode.set(true);`, then rebuild/install.
3. Run **Pinpoint Tuner**. Enter the Pinpoint name and actual pod type.
   Keep the robot completely still during initialization/IMU calibration.
   After each stage starts, perform the indicated hand push, then press Stop
   to save that measurement. For offsets, rotate exactly **180 degrees CCW**
   about a fixed robot center, then Stop. These stages do not power the motors.
4. Copy the tuner's **Java** localizerConfig into Constants.java, replacing
   only that configuration block. Set `LOCALIZER_TUNED = true`. Rebuild/install.
5. Run **Tests → Pose Test** first (hand movement, no drive power). Confirm:
   forward increases X, left increases Y, CCW increases heading, a measured
   push reports the correct inches, and an in-place turn barely changes X/Y.
   Correct any errors before proceeding. Localization Test adds manual driving.
6. Run **Foresight Tuner** with plenty of clear floor and a driver ready to Stop.
   It drives the robot automatically, including full-power measurements.
   Paste its Java foresightConfig into Constants.java, set
   `FORESIGHT_TUNED = true`, then rebuild/install.
7. Verify short Line/Curve tests before running **22310 Pedro Auto Example**.
   That autonomous is only a sample route, not a scoring routine.

All path distances are inches and headings are radians. Generated custom
encoder resolution is ticks per inch. Imports needed by the generated Java
configuration blocks are already included in Constants.java.

The measured offsets/motion constants cannot be determined from source code.
Until tuning is completed, the flags remain false and autonomous motion is
deliberately blocked. A successful build does not verify robot calibration.
