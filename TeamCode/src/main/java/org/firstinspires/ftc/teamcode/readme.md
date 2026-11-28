# Team 22310 robot code

Everything our robot runs in a match is in this folder. This page says how to drive it, how the code is laid out, what to check before a comp, and what changed.

It builds and passes the desktop tests and the simulator. **The camera aim, the one-button shot speed and the new flywheel control have not been proven on the real robot yet.** Do the checklist in "Before a comp".

## Controls

![controls](controls.svg)

| Gamepad 1 (mechanisms) | Does |
|---|---|
| RIGHT TRIGGER | Intake |
| LEFT TRIGGER | Spit everything out (beats everything else). The index runs slowly so pollen don't get thrown |
| RIGHT BUMPER (hold) | Spin up the shooter to the set speed |
| LEFT BUMPER (hold) | Shoot: the index runs for as long as it is held |
| BACK | Open / close the shooter tuning menu. Closed, the D-PAD and X do nothing |
| D-PAD left / right | Menu open: pick a shooter number on the screen: Speed, P, I, F, Feed on, Feed off |
| D-PAD up / down | Menu open: change that number (hold to keep going). Saved by itself |
| X (hold 1 second) | Menu open: shooter numbers back to default |

| Gamepad 2 (driving) | Does |
|---|---|
| LEFT STICK | Drive (field-centric) |
| RIGHT STICK | Turn |
| LEFT TRIGGER (hold) | Auto aim at our hive |
| RIGHT TRIGGER (hold) | Slow mode |
| START (tap) | Make the way the robot faces "forward" |

Before START, on either gamepad: **X = blue alliance, B = red alliance**. After an auto it already knows.

**To shoot:** hold RIGHT BUMPER, wait for gamepad 1 to buzz (wheels ready), then hold LEFT BUMPER. LEFT BUMPER runs the index the moment you press it, so pressing it before the buzz sends pollen into slow wheels. Gamepad 2 buzzes when the aim is locked.

## How scoring works (and why the aim does what it does)

From the competition manual (sections 9.6, 9.9, 10.5):

- Each alliance's hive is a seesaw with a CELL at each end. **Only the cell that is tilted UP can be scored in.**
- Enough pollen in the up cell TIPS the hive (20 points). The other cell is then the up one.
- The up cell's opening faces outward, toward its end of the field. So **you shoot from the end the up cell faces**, and after a tip you shoot from the other end. Our Strong auto does exactly that: shoot, cross the field, shoot again.
- At the start of a match the up cells are red 34-37 and blue 42-45 (the tag numbers under them).
- Shooting at the other alliance's hive, or at the outside of a cell on purpose, is a foul (G417).

The camera works out which cell is up by itself: when it can see both of our stickers, the higher one is the up cell. It remembers that for 30 seconds; after that a single sticker in view is simply taken as the target.

## Shooter speed and PID

There is one flywheel speed. The camera does not change it. What the screen shows is what the wheels are asked for.

The screen always says whether the flywheels are at FULL POWER or held at a lower speed. To change anything, press BACK on gamepad 1 to open the tuning menu (press it again to close). Open, it shows six lines, with `>>` marking the one the D-PAD changes. Closed, the D-PAD and X do nothing, so nothing can be changed by accident in a match:

| Line | Default | One press | What it does |
|---|---|---|---|
| Speed | 3000 | 50 | How fast the wheels spin. 2800 or more means FULL POWER straight to both motors, with no speed control at all |
| P | 0.70 | 0.05 | How hard it pushes back when the speed is off. Wheels hunting up and down: lower it |
| I | 0.50 | 0.05 | Slowly removes a steady error. Speed sits a little under the target: raise it |
| F | 1.00 | 0.01 | The base power for a speed. Speed sits well under the target everywhere: raise it |
| Feed on (ms) | 250 | 25 | Only used when Feed off is above 0: how long the belt runs to hand over one pollen |
| Feed off (ms) | 0 | 50 | 0 = the belt runs for as long as LEFT BUMPER is held. Above 0 = one pollen at a time: the belt stops this long between pollen so the wheels get their speed back |

- D-PAD left / right picks the line, up / down changes it. Hold it down to keep stepping.
- The numbers are saved on the Control Hub when you let go, and are still there after a restart or a new install.
- Autonomous uses the same numbers. The auto INIT screen shows them.
- Hold X for a second to go back to the defaults.
- LEFT TRIGGER runs the wheels backwards at 1600.
- Below 2800 the two wheels are held within 50 of each other. The flywheel line shows how far apart they are and names the slow wheel if they are not. At full power they are not matched: the only way to match there is to slow the faster wheel, which throws speed away. A wheel that is slow at full power has a mechanical problem (drag, a slipping hub, a tired motor).
- **Hold LEFT BUMPER and the belt runs.** That is the default. If several pollen come out weakly together (they arrive nose to tail, and the second hits wheels that have not recovered from the first), try one at a time: raise Feed off to about 400. The belt then goes on, off, on, off while the bumper is held. For a single good shot you can also just tap the bumper.
- **Where the encoders are.** The flywheel encoder cables go in the Expansion Hub's encoder ports 2 and 3, right beside the flywheel motor wires. If the flywheel line reads 0 / 0 while the wheels spin, they are not being read: check both cables are pushed in.
- With encoders working, a Speed below 2800 is held properly (P, I, F and wheel matching all work). 3000 is still plain full power with no control; for the most even volleys try about 2500.
- At full power (anything near 2800 or above) the wheels have nothing in hand, so they slow a little while pollen go through. If the first pollen flies further than the rest, lower Speed until they match (around 2300 to 2400).

## Auto aim

Hold LEFT TRIGGER on gamepad 2. The robot turns to face our hive and stays locked on, and you can keep driving with the left stick.

How it works: the camera only says *where* the hive is. The turning is done on the Pinpoint gyro, which is read every loop with no delay. That is why it is quick and why it stays locked when the tag blinks out for a moment.

The turn itself has three parts:

- **Turn and brake.** Power = `KP` x how far off - `KD` x how fast it is turning, up to full power.
- **Friction push.** A mecanum drive needs a fair bit of power before it turns at all, so plain KP/KD gets most of the way and then crawls the last few degrees. Whenever the robot is closing in slower than it should for how far off it is, extra power builds until it is not. It finds the robot's friction by pushing, so there is nothing to measure.
- **Safety net.** If the robot swings through the target and back, KP and KD are turned down a step each swing until it settles, and the AIM line says "WOBBLING".

The AIM line shows how long the last lock took ("AIMED (locked in 0.45 s)"). Use that number when you change anything.

- Red aims at tags 30-37, blue at 38-45. Wrong alliance = it never finds the hive.
- It aims at the middle of the up cell's sticker, even when only some of its four tags are in view.
- If it can only see the down cell it still points at the hive and says "only the DOWN cell in view".
- The Driver Station line "AIM" says what it sees, including which tag numbers are in view and which pipeline the camera is on.
- It uses Limelight pipeline 0 and assumes the camera looks the way the shooter shoots.

### Limelight setup (do this once, in the Limelight web page)

The tags are 3 to 4 feet up and face the floor, so the camera has to be tilted up about 25 to 35 degrees to see them.

1. Robot config: the Limelight is an "Ethernet Device" named `limelight`.
2. Pipeline 0 is a Fiducial / AprilTag pipeline, family "AprilTag Classic 36h11".
3. ID filter empty (or 30-45). Minimum area 0.
4. Exposure as low as still reads the tags while the robot spins (high exposure blurs them), gain up to compensate.
5. "Ignore NetworkTables Index" is NOT ticked (or the code cannot pick the pipeline).
6. 640x480 is fast. Go to 1280x960 only if far tags drop out.

If it is wrong on the real robot, it is one number at the top of `Shared/HiveAim.java`:

| Problem | Change |
|---|---|
| Turns AWAY from the hive | `TX_SIGN` to -1 |
| Always a bit left or right | `AIM_OFFSET_DEGREES` (+ aims further right) |
| Wobbles, or the AIM line says WOBBLING | lower `KP` and `KD` together (try 2.8 and 0.35) |
| Sails past the hive, then comes back | raise `KD` only (try 0.50) |
| Picks the lower cell | `IMAGE_Y_GROWS_DOWN` to false |
| Whole turn is slow | raise `KP` and `KD` together (try 4.5 and 0.45) |
| Fast at first, slow over the last few degrees | raise `PUSH_PER_SECOND` (try 2.5) |
| Twitches when it is nearly there | lower `PUSH_PER_SECOND` (try 1.0), or raise `REST_DEGREES` to 1.0 |
| AprilTag pipeline is not slot 0 | `PIPELINE` |

## Autos

| Auto | What it does |
|---|---|
| Red Alliance Strong | Shoots 4 preloads, intakes the garden, drives back beside the start, shoots 4 more into the same cell, parks |
| Blue Alliance Strong | Same, mirrored |
| RedAllianceWeak | Starts in the corner beside our parking zone, drives one straight line into the zone and stops. No shooting, no turning |
| BlueAllianceWeak | Same, mirrored |

- **Every shot is aimed with the camera** if it can see the hive: the robot turns to the tags before feeding. If it can't see them it shoots on the heading from the route. A camera reading that asks for more than 25 degrees is ignored as a bad read.
- Shot speed in auto: the same set speed, P, I and F as teleop (the numbers changed with the D-PAD on gamepad 1). The INIT screen shows them as "Shooter".
- The strong auto shoots both volleys from the start side, at the cell that is up when the match starts, so it scores whether or not the hive tips. To go back to shooting the second volley from across the field, set `SECOND_SHOT_FROM_START_SIDE` to false in `RedAllianceStrong.java`.
- Each volley: the wheels spin for 2.5 s, then the index feeds one pollen at a time: belt on 0.5 s, then a wait until the wheels are back at speed (0.3 to 0.8 s), eight times, so the belt runs 4 s in all.
- Garden run: both legs bow out into the field (about 15 inches from the bottom wall) for the part where the robot turns, because turning right beside the wall puts a corner into it. The garden point itself has not moved. The three bend points are next to it in `RedAllianceStrong.java`.
- Every auto stops scoring at 24 s no matter what and parks, using whatever time is left up to 29 s. It selects MAIN TELEOP afterwards.
- A missing intake, flywheel or camera does not stop the auto. It still drives and parks.
- Blue is red turned half a turn. Change red and blue changes too. Blue has never been on a field.
- Flower pickup is written but switched off.

## Battery protection

The battery used to drop from 11 V to 4 V and once reset a hub mid-match. Now:

- `Shared/Battery.java` watches the voltage for the whole robot.
- Low voltage caps the drive power and slows how fast any motor may ramp up.
- Drive, intake and index all ramp up over a fraction of a second instead of jumping to full.
- The flywheels limit their own current while spinning up (about 10 A for both, not 17).
- If a hub does restart, every motor sets itself up again and carries on. The Driver Station shows "A HUB LOST POWER".

Why it happens: if the four drive motors, two flywheels, intake and index all start together they can ask for 55 to 75 A, from a battery behind a 20 A fuse. A worn battery loses several volts at that.

Code can only soften it. The rest is hardware:

- **Batteries.** A full pack rests at 13 V or more. "11 V at the start" means nearly empty. If you can get a goBILDA Battery Health Analyzer: match packs should read 130 milliohms or less, and anything over 200 should be retired.
- **XT30 plugs.** The Expansion Hub gets its power through the Control Hub and the XT30 cable between them. Loose plugs usually mean squashed pins; open the pins up a little rather than gluing.
- **The Driver Station shows each hub's voltage.** If they differ by more than about half a volt it says "HUBS DIFFER": that is the cable between them, not the battery.
- **Check the flywheel gearbox.** The code is told the flywheels are HD Hex motors with 40:1 gearboxes. A 40:1 output only turns 150 rpm, which is very slow for a shooter unless something gears it back up. REV's own launchers use the HD Hex with no gearbox. The code does not care (the encoder is on the motor either way), but the gearbox adds friction and current.

## How the code is laid out

```
teamcode/
  Shared/               used by BOTH teleop and auto, so they can never disagree
    Battery             voltage watch + power limits
    FlywheelControl     runs the shooter wheels at a speed, says when they are ready
    IntakeAndIndex      intake roller + index belt, directions and speeds in one place
    HiveAim             Limelight -> which way to face, and how far the hive is
    ShooterSettings     the set speed and P, I, F, saved on the Control Hub
    HubReset            recovering when a hub loses power
    AutoHandoff         tells teleop which way the robot faced when auto ended

  FinalTeleOp/
    MainTeleOp          the match teleop: wires gamepads to mechanisms
    FinalMechanisms/
      MecanumDriveFinal   field-centric drive
      Shooter             gamepad 1 shooter buttons + the D-PAD menu for speed and PID
      IndexIntake         gamepad 1 intake / index buttons
    RobotSensors/         test OpModes (camera, colour sensor, turret)

  FinalAuto/
    AutoBase            what all four autos share (park deadline, clean-up, blue mirroring)
    RedAllianceStrong, RedAllianceWeak      the routes (blue ones are one line each)
    FinalMechanismsAuto/
      MecanumDriveAuto    Pedro drive as commands that can't hang, plus camera aim
      Volley              one volley: aim, spin up, feed, stop
      FlywheelAuto, IndexIntakeAuto    the shared mechanisms as auto commands
    pedro/              Pedro Pathing config and tuners (see pedro/PedroTuning/README.md)

  SavedBackupCode/      old code, kept for reference
```

Rule of thumb: a motor direction, a power or a speed lives in `Shared/`. Buttons live in `FinalTeleOp/`. Routes live in `FinalAuto/`.

## Before a comp

1. **Flywheel direction.** Wheels off the ground, hold RIGHT BUMPER. If they spin the wrong way, flip `DIRECTION` in `Shared/FlywheelControl.java` (one line, fixes teleop and auto).
2. **Flywheels smooth?** Under a second to speed, then steady. If they hunt, lower `DEFAULT_P` in `FlywheelControl.java` (try 0.6).
3. **Auto aim.** LEFT TRIGGER with the hive in view. Table above if it is wrong.
4. **Shooter numbers.** Check Speed, P, I, F on the INIT screen are what you tuned.
5. **Outtake.** LEFT TRIGGER: intake and index both run backwards at the same power. The index speed is `INDEX_OUTTAKE_POWER` in `Shared/IntakeAndIndex.java`.
6. **Auto start pose.** After INIT the Driver Station shows where the robot should be. It has to match.
   - Red Strong: x 56, y 8, heading 270
   - Blue Strong: x 88, y 136, heading 90
   - Red Weak: x 8, y 136, heading 90 (in the corner beside our parking zone, touching both walls)
   - Blue Weak: x 136, y 8, heading 270
7. **Run each auto you will use.** Watch the "Last aim" line: it says whether the camera aimed the shot.
8. **Pinpoint cable test.** INIT an auto, pull the Pinpoint cable for half a second, plug it back. It should say "Pinpoint restarted".
9. **After auto.** Teleop now sets "forward" by itself from where auto ended (the Drive line says "forward set by auto"). Check it once: push the left stick away from you and the robot should drive away from you. If it is wrong, tap START the old way, and fix `RED_DRIVERS_FACE_DEGREES` in `Shared/AutoHandoff.java`. This only applies when teleop is initialised within 30 seconds of the auto ending (always true in a match). Otherwise forward is whichever way the robot faces at INIT, like before.
10. **Limelight.** Do the setup list above and check the AIM line shows our tag numbers.
11. Fresh battery every match (13 V or more resting).

## What the Driver Station is telling you

| Message | Means |
|---|---|
| A HUB LOST POWER | Battery or power cable problem. The code recovered; check battery, switch, XT30 cables |
| EXPANSION HUB NOT FOUND | The Control Hub could not find the Expansion Hub when the robot started, so the intake, index and flywheels do not exist (driving still works: those motors are on the Control Hub). The names are not the problem. Check the Expansion Hub's power and the cable between the hubs, then restart the robot |
| INTAKE OFF / FLYWHEEL OFF / AUTO AIM OFF | That device is missing from the robot config. Everything else still works |
| A flywheel encoder counts backwards (handled in code) | That motor's encoder and power wires are opposite ways round. The code flips the reading, so nothing to do before a match |
| NO FLYWHEEL ENCODERS | Neither flywheel encoder reads anything. The wheels still spin, on power alone, but the speed is not held and shots drift with the battery. Check both encoder cables |
| AIM: ... WOBBLING | The aim swung past the hive and calmed itself down. Lower `KP` and `KD` in `Shared/HiveAim.java` |
| ROBOT-CENTRIC | No Pinpoint heading. Driving works, auto aim does not |
| Pinpoint restarted - face forward, tap START | Pinpoint cable blipped. Point the robot away from you and tap START |
| Index is waiting: flywheels still turning backwards | Only just after an outtake. It goes as soon as the wheels have slowed |
| A FLYWHEEL READS 0 | Flywheel encoder cable or a jam |
| Drive power limit under 100% | Battery low; power eased off so the robot doesn't brown out |
| HUBS DIFFER | The two hubs see different voltages: check the XT30 cable between them |
| only the DOWN cell in view | The camera can't see the up cell's tags. You may be at the wrong end of the field for this cell |
| AIM: only sees tags ... | The camera sees tags, but not our hive's. Wrong alliance, or facing the wrong hive |
| camera not answering | Limelight unplugged or still starting (about 20 s after power on) |

## Where to change numbers

| What | Where |
|---|---|
| Auto routes, speeds, timings | top of `RedAllianceStrong.java` / `RedAllianceWeak.java` |
| Auto flywheel speed | top of `FlywheelAuto.java` |
| Teleop backup speeds (1500 / 900) | top of `Shooter.java` |
| Flywheel direction, smoothness, "ready" tolerance | top of `Shared/FlywheelControl.java` |
| Intake / index / outtake power | top of `Shared/IntakeAndIndex.java` |
| Auto aim | top of `Shared/HiveAim.java` |
| Battery limits | top of `Shared/Battery.java` |
| Stick feel, slow mode speed | top of `MecanumDriveFinal.java` |
| Auto volley timing (spin-up, pulse, pause, total feed) | top of `Volley.java` |
| Pedro tuning | `FinalAuto/pedro/PedroTuning/Constants.java` |

Change the number instead of reverting a whole file.

## How it was tested

No robot was available, so everything was checked on a laptop:

- desktop tests with fake motors, gamepads, Pinpoint and battery (drive, flywheels, feed timing, shot setup, hub restarts, the whole teleop with devices missing)
- a motor and battery model for the flywheels
- a turn simulation for the auto aim (about 550 turns: light and heavy robots, easy and sticky turning, slow loops, wrong camera timing)
- the four real auto routines run in a field simulator through ten scenarios (blocked by a wall, dead flywheel, stuck robot, ...). All end parked with everything off.

That finds logic bugs. It cannot find a backwards motor, a wrong camera angle or a number that needs tuning. That is what the checklist is for.

## What changed

**Nov 28**
- Weak autos (both colours) are now park only. Start the robot in the corner beside our parking zone, touching both walls. It drives one straight line (about 30 inches) into the zone and stops at the edge nearest the side wall, a couple of inches clear of it. How far from the wall is the x of `parkatSpawn` in `RedAllianceWeak.java`. The old route up the middle is in the git history.
- Strong autos no longer rub the bottom wall. Both garden legs were straight lines a couple of inches from it with the robot turning on the way, which put a corner into the wall for about 3 s a run. They now bow out into the field while the robot turns. Same garden point, same shots, about 0.2 s longer.
- Teleop: the flywheels hold a speed (default 900, raised from 850 which landed a little short; it is the average of the two wheels) instead of running flat out. Flat out, the first pollen met wheels 11 to 20% faster than the rest and flew long. The feedforward now knows how much voltage this robot's wheels need just to turn, and follows the battery voltage. If the screen says the wheels fell during a feed, lower Speed to the number it suggests.
- Strong autos: back to the earlier garden route (original end point, no stop on arrival, one turning path back), still shooting the second volley from the start side. The drive speed cap is gone again.
- Auto volleys feed one pollen at a time: belt on 0.5 s, then wait until the wheels are back at speed (0.3 to 0.8 s), eight times.

**Nov 27 (late)**
- The two gamepads no longer get in each other's way. The battery guard used to slow and limit the drive below 11 V, and on this robot the shooter alone holds the flywheel hub at 10 to 11 V, so driving went soft whenever the other driver ran the shooter. The guard now only steps in below about 9 V.
- Full-power flywheels are back to true full power after a half-second soft start. For one build they went through a voltage-based current cap, which starved them whenever the voltage sagged (strong one moment, weak the next).

**Nov 27 (night)**
- Brownout fix. The battery fell to 5.3 V in a run and both hubs restarted. Full-power flywheels were starting with no soft start (each motor pulling its full stall current at once) while the drive was moving. The spin-up current cap is back for full power too, the drive is told to go easy while the wheels spin up, the index no longer starts in one step, and the index no longer sits stalled during the garden run. In the autos the drive is capped at 75% of top speed, which also caps the power it takes off the line (Pedro otherwise starts every path at 100%).
- Auto volleys run on fixed timing: 2.5 s spin-up, then four 1 s pulses of the index with 0.5 s pauses. The autos no longer wait on a "ready" signal.
- Garden run no longer spins among the pollen. The end point was touching both walls, so the robot kept pushing and pivoting; it is now 1.5 inches further out, the robot releases its wheels on arrival, and it backs out straight before turning.

**Nov 27 (evening)**
- What "TEST: Robot check" found on the robot: each flywheel is slow even on its own and draws 5 to 6 A doing it (right 1100, left 1500; healthy is 2500+ at under 1 A), the index draws 6.4 A empty, and the hub with the flywheels gets about 0.9 V less than the other one. The wheels are not fighting each other and reversing one changes nothing. So the weak shots are friction and power wiring, not a setting.
- Autos: only the index runs while feeding (the intake ran too), to leave more for the flywheels.
- The robot check now shows both hubs' voltage for each flywheel run and flags a high index current.
- New test OpMode, "TEST: Robot check" (in the Test group). Press START and it tests the hubs, battery, Pinpoint, camera, flywheels (each alone and both together), intake and index by itself in about 45 seconds, and leaves a report on the screen to photograph. The drive wheels never turn in this test. For the flywheels it says in words what is slowing them. Photos of the Driver Hub showed the wheels reading 840 and 1400 at full power (healthy is 2500+) with the battery falling 2 V, so the shooter is being held back mechanically or the two wheels are fighting.
- READY now comes up at full power once the wheels have levelled off (it never did, because they were being compared with a target of 3000).
- The shooter tuning menu has to be opened with BACK. It used to listen to the D-PAD all the time, and shots went short without anybody meaning to lower the speed. The saved numbers from before are not read, so Speed is back to 3000 (full power).
- The screen says FULL POWER, or "held at N - NOT full power", so a low speed setting is obvious.
- The search for flywheel encoders on ports 0 and 1 is gone again: they are on ports 2 and 3, where the code always looked.
- Feeding one pollen at a time is on the D-PAD menu (Feed off above 0). It is off by default: holding LEFT BUMPER runs the belt the whole time. Autonomous feeds the same way teleop is set.

**Nov 27 (later)**
- Strong autos: the garden pollen are now shot from beside the start position, into the same cell as the preloads, instead of from across the field. The far shot only scored if the hive had tipped.
- Parking may use all the time left in autonomous (it used to give up after 4 s, and the park spot is now further away).
- Autos: the flywheels always get at least 2.5 s to spin up before the first pollen, and the index feeds for 4.5 s (was 2.5 s). The first auto shot was going nowhere because the wheels were not up and the belt stopped before the pollen reached them.
- Speed 2800 or more is now plain full power to both flywheels: no correction, no current cap, no wheel matching, nothing that depends on an encoder. If a shot is weak like that, the software is not what is holding it back.
- The index feeds at full power (was 0.95) and gets up to speed four times quicker.
- One set flywheel speed. The camera no longer changes it, and saved shots, shot setup and the Y presets are gone.
- Speed, P, I and F are changed with the D-PAD on gamepad 1 and saved by themselves. Default speed 3000.
- Autonomous uses the same speed and PID as teleop.
- The two flywheels are held within 50 of each other, and the screen shows how far apart they are.

**Nov 27**
- Shots were still weak with no tag in view. For one day, A in shot setup overwrote the HIGH preset whenever the camera had no distance, so a speed tuned for a close shot could become the speed for everything (the autos too). A now only saves against a distance; B is the button that changes a preset. The saved presets from that day are no longer read, so HIGH is back to 2300.
- The teleop INIT screen shows the HIGH and LOW speeds and the saved shots.
- Autonomous shoots at the teleop HIGH preset (was its own 2100). Change HIGH from the gamepad and the autos follow.
- Shots were far too weak with the Limelight plugged in and fine with it unplugged. It was not power: with the camera seeing the hive, the speed came from saved shots, and the saved speeds were too low. The old saved-shot file is no longer read (new file name), so the list starts empty and the preset is used until shots are saved again.
- A saved shot no longer applies to shots further away than it: past the furthest saved shot the speed climbs to the HIGH preset.
- The "Shot speed" line says "from SAVED SHOTS" when a saved shot is setting the speed.

**Nov 26**
- Auto aim is faster. The old one crawled the last few degrees on a robot that is hard to turn. In the simulation a sticky robot now locks in about 0.6 s instead of 1.1 s, and an easy-turning one in about 0.4 s (that one was already quick).
- The aim can use full power (was 70%). While the flywheels spin up or the battery is tired, its top power comes down so it cannot brown out a hub.
- The aim backs itself off if it ever swings through the target, and says WOBBLING.
- Pictures taken in the middle of a fast turn count for less, so a small error in the camera's timing no longer drags the aim past the hive.
- The AIM line shows how long the lock took.
- "EXPANSION HUB NOT FOUND" replaces the confusing "unable to find right_flywheel" when the Expansion Hub is not connected.
- Flywheels no longer run slow when an encoder is wrong. An encoder that counted backwards used to hold the wheels at about half the asked speed whatever the setting, and two dead encoders capped them at 56% power. Backwards readings are now flipped, a dead one is covered by the other wheel, and with no encoders at all the wheels run on power alone. The Driver Station says which.
- Quicker flywheel spin-up on a fresh battery (12.5 V and up).
- Shooter presets: HIGH is now 2300 (was 1500), LOW is 1500 (was 900), and backwards is 1600 (was 900). Change them at the top of `Shooter.java`.
- Outtake: the index runs backwards at the same power as the intake (0.95). At 0.25 it did not turn.
- LEFT BUMPER always runs the index at once in teleop. It no longer waits for the flywheels (that wait left the index standing still on the real robot). Wait for the buzz / READY, then press it.
- In autonomous the index is held only at the start of a volley until the wheels are ready (0.6 s at most), then runs without pausing.
- Even speed through a volley now comes from the flywheels: power kept in hand at HIGH, plus the feed boost.
- The Y presets can be set from the gamepad in shot setup and are remembered.
- The flywheels no longer slow down when the index starts: HIGH keeps a tenth of the power in hand, and the wheels get a little extra power the moment feeding begins instead of after they have already slowed.
- The flywheel line shows the power being sent, next to the two speeds.

**Nov 25 (revamp)**
- New `Shared/` folder: one copy of the battery guard, flywheel control, intake/index, camera aim and shot table, used by teleop and auto.
- Autonomous aims every shot with the Limelight and uses the saved shot speeds.
- The aim knows the hive tips: it picks the up cell, finds the middle of its sticker from whatever tags are in view, and remembers which cell is up for 30 seconds after it last saw both.
- Hive distance now comes from how far apart the tags look. No Limelight 3D setting needed.
- Auto aim is about 30% quicker to lock and no longer overshoots when there is delay in the loop.
- Teleop takes "forward" from where auto ended, so no START tap after auto.
- The battery guard reads every hub (the lowest one counts), and the drive and intake go easier while the flywheels spin up.
- Flywheel gain and wheel matching re-tuned against how the hub really measures speed (steps of 20 ticks/s, about 25 ms late).
- Auto flywheel speed 2100 (was 2300).
- The Pinpoint tuner rejects a nonsense custom pod scale.
- Consistent shots: the index only feeds when the flywheels are at speed, in teleop and auto.
- The index runs slowly when outtaking (0.25 instead of 0.45).
- One battery guard for the whole robot; intake and index now ramp up too.
- Diagonal driving no longer bends for the first tenth of a second.
- Flywheel direction is defined once. `FlywheelTunerPIDF` is now `Shooter` (it stopped being a PIDF tuner).
- Every file has comments that say why, not just what.
- This readme replaces CHANGES.md.

**Nov 25 (earlier)**
- Auto aim in MAIN TELEOP, one-button shooter speed with shot setup, slow mode, alliance picked at INIT.
- Only MAIN TELEOP left in the "TeamCode" group on the Driver Station.

**Nov 24**
- "TargetPositionNotSetException" crash fixed (a hub losing power for a moment).
- Stuttery flywheels fixed: our own flywheel control instead of the hub's.
- FTCLib removed (unused duplicate of SolversLib).
- Auto stops if the Pinpoint drops out or restarts, and recovers from a glitch during INIT.
- Teleop still drives if a mechanism or the Pinpoint is missing.

**Nov 23**
- New autos: Red / Blue, Strong / Weak, on a shared `AutoBase`.
- Smoother teleop drive, low battery limiter, field-centric with the Pinpoint.
- Battery drain and the 11 V to 4 V drop fixed (drive motors were in the wrong run mode).
- Colour sensor red / blue swap fixed. Pedro "Pinpoint not ready" fixed.
