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

### All controls

| Gamepad 1 | Does |
|---|---|
| RIGHT TRIGGER | Intake |
| LEFT TRIGGER | Outtake everything (wins over everything else) |
| LEFT BUMPER | Index inwards (feeds the shooter) |
| RIGHT BUMPER (hold) | Spin flywheels |
| RIGHT + LEFT BUMPER | Shoot |
| Y | Flywheel high / low speed |
| BACK | Unlock flywheel tuning |
| D-PAD | Change P and F (only when unlocked) |
| X | Change step size |
| A | Save P and F (auto uses them too) |

| Gamepad 2 | Does |
|---|---|
| LEFT STICK | Drive (field-centric) |
| RIGHT STICK | Turn |
| OPTIONS (let go) | Make the way the robot faces "forward" |

Gamepad 1 buzzes when the flywheels are at speed.

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

### More Nov 24 fixes (things that could go wrong at a comp)

- MAIN TELEOP still drives if the intake, flywheel or Pinpoint is missing from the config. It says what is off on the Driver Station.
- If the Pinpoint cable blips and the Pinpoint restarts, auto stops instead of driving from a wrong position. In teleop it goes robot-centric until you press OPTIONS.
- If the Pinpoint glitches while the robot sits in INIT before auto, it sets itself up again instead of crashing.
- Pressing STOP right after INIT on an auto no longer shows a fake error.
- Flywheels coast down before reversing (LEFT TRIGGER while still spinning used to slam them backwards). Same for the auto spin-up after a stop.
- Auto shooting waits for the wheels to recover between pollen even when they can't reach the target speed.
- Driver Station says so if a flywheel reads 0 (encoder cable or jam).
- LEFT TRIGGER (outtake) now always wins over RIGHT TRIGGER and LEFT BUMPER.
- Gamepad 2 buzzes at START if field-centric is not available.
- "Straight Line Test" and "Angled Line Test" moved to the "Test" group so nobody picks them in a match.

## Nov 24: "TargetPositionNotSetException" crash

**What it was:** our code never uses RUN_TO_POSITION. The SDK shows that message whenever a hub refuses to switch a motor on, and a hub does that right after it loses power for a moment. Driving and shooting at the same time pulls a lot of current, the battery voltage dips, a hub resets, and the next motor command crashed the whole OpMode.

**What changed:**
- Every motor (drive, flywheels, intake, index, in teleop and auto) now catches that refusal, sets the motor up again (mode, brake, flywheel PIDF) and keeps going. No more crash.
- The Driver Station shows "A HUB LOST POWER" when it happens. If you see it, the robot has a power problem: check the battery (charge, age), the power switch, the XT30 cable between the hubs, and that the battery connector is tight.
- The drive's low-battery limiter now reacts about twice as fast, so the voltage dips less.

**Flywheels (REV HD Hex 40:1, top speed 2800 ticks/s):**
- Starting PIDF is now based on the real motor: F = 32767 / 2800 = 11.7, P = 5.85, I = 0.59. Before, F was about 18% too high, so the wheels overshot and "at speed" never showed. If you saved your own P and F with A, those still win.
- Spin-up is faster (0.3 s instead of 0.4 s) on a good battery. Below 11 V it ramps up more slowly so it doesn't pull the battery down; it never stops spinning up.
- Auto `HIGH_VELOCITY` is now 2300 (was 3000, which the motor can't reach, so the wheels just ran flat out).

## Check before a match

1. Flywheel direction. Teleop is `REVERSE`, auto is `FORWARD`. One is wrong. Run an auto with the wheels off the ground and look.
2. Flywheel speed in auto. `HIGH_VELOCITY` is now 2300. If shots land short or long from the auto spots, change it in `FlywheelAuto.java` (2380 is about the most the motors can hold).
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
