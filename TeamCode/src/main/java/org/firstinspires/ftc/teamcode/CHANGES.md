# Code changes, Nov 23

Quick notes on what changed in the robot code and what to check before we trust it.

**Not tested on the real robot yet.** It builds and passes the simulator, that's it.

## Autos

| Auto | What it does |
|---|---|
| Red Alliance Strong | Shoots 4 preloads, intakes the garden, drives across, shoots again, parks |
| Blue Alliance Strong | Same thing, mirrored for blue |
| RedAllianceWeak | Drives to the shooting spot, shoots 4 preloads, parks |
| BlueAllianceWeak | Same thing, mirrored for blue |

- Every auto gives up and parks at 24 s if something goes wrong.
- Every auto selects MAIN TELEOP afterwards.
- The indexer runs slowly while intaking to make room for more pollen.
- Flower pickup is in the code but turned off (`FLOWER_CYCLE` / `FLOWER_PICKUP`).
- Blue is just red rotated 180 degrees. Change red and blue changes too.

Files:
- `FinalAuto/AutoBase.java` is the shared part of all four autos
- `FinalAuto/FinalMechanismsAuto/` has the auto versions of drive, intake, flywheel
- `FinalMechanismsAuto/Volley.java` is one round of shooting

## Teleop

- Sticks are smoother (less twitchy near the middle, no dead spot).
- Robot speeds up gradually, stops right away when you let go.
- Low battery: power gets limited smoothly instead of the robot browning out.
- Gamepad 2 buzzes when field-centric turns off (long) or back on (2 short).

### Flywheel controls (gamepad 1)

| Button | Does |
|---|---|
| RIGHT BUMPER (hold) | Spin flywheels |
| Y | High / low speed |
| LEFT TRIGGER | Reverse (for jams) |
| BACK | Unlock tuning |
| D-PAD | Change P and F (only when unlocked) |
| X | Change step size |
| A | Save P and F (auto uses them too) |

The controller buzzes when the flywheels are at speed.

## Bugs fixed

- Battery dropping from 11 V to 4 V while driving (drive motors were in the wrong run mode)
- Flywheels staying on and draining the battery
- Pairing a controller mid match reset the heading
- A loose odometry pod cable turned off field-centric
- Indexer stuttering while shooting
- Autos kept pushing into a wall after getting stuck
- Colour sensor had red and blue swapped
- Auto align aimed at the wrong tags and had forward backwards
- Driver Station telemetry flickering
- Pedro "Pinpoint not ready" in the tuners

## Nov 24 fixes

- FTCLib removed from `TeamCode/build.gradle`. Nothing used it and it was the same library as SolversLib twice. SolversLib is still there. Do a Gradle sync after pulling.
- Auto: if the Pinpoint stops answering, the robot stops instead of driving blind for a second.
- Auto: only shoots if it is actually facing the target (within 10 degrees).
- Auto: park deadline moved to 24 s so the worst case still ends before 30 s.
- "22310 Pedro Auto Test" never got past the first leg. Fixed.
- Flywheels: pressing the bumper again while the wheels are still spinning no longer brakes them first.
- Heading reset now happens when you let go of OPTIONS, so pairing with START + A/B can't trigger it.
- Drive still ramps up when loops are slow.
- Auto align opmode renamed to "April Tag Auto Align" (it drives the robot, it is not just a camera test). Its tuning can't go negative anymore.
- Turret align ignores results from the wrong Limelight pipeline.
- Backup opmodes: "Mecanum Field Oriented" used the run mode that caused the brownouts, and "Index" / "Omnidirectional Intake" could never move their motor. Fixed.

## Check before a match

1. Flywheel direction. Teleop is `REVERSE`, auto is `FORWARD`. One is wrong. Run an auto with the wheels off the ground and look.
2. Flywheel speed in auto. `HIGH_VELOCITY = 3000` is probably more than the motors can do. The Driver Station warns if it is. Lower it to what they actually reach.
3. Start pose. After INIT the Driver Station shows where the robot should be. It has to match where the robot really is.
   - Red Strong: x 56, y 8, heading 270
   - Blue Strong: x 88, y 136, heading 90
   - Red Weak: x 56, y 8, heading 90
   - Blue Weak: x 88, y 136, heading 270
4. Blue autos have never been on a field. Test before using them.

## Where to change stuff

| What | Where |
|---|---|
| Auto speeds, timings, positions | top of `RedAllianceStrong.java` / `RedAllianceWeak.java` |
| Flywheel speeds for auto | top of `FlywheelAuto.java` |
| How the sticks feel | top of `MecanumDriveFinal.java` (`STICK_LINEAR_PART`, `MAX_POWER_RISE_PER_SECOND`) |
| Shooting time per volley | `Volley.java` (`FEED_MS`) |
| Pedro tuning | `FinalAuto/pedro/PedroTuning/Constants.java` |

If something feels off, change the number instead of reverting the whole file.
