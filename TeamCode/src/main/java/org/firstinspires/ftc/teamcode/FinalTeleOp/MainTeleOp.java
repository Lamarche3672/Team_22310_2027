/*
GAMEPAD 1 (mechanisms)
RIGHT TRIGGER: Intake
LEFT TRIGGER: Outtakes everything, but at different speeds (index, intake, shooter)
RIGHT BUMPER (hold): Spin the shooter to the speed for this distance
LEFT BUMPER (hold): Index inwards = shoot (it waits for the flywheels by itself)
Y: Backup shooter speed high/low (only used when the camera can't see the hive)
BACK: Shot setup. Then D-PAD up/down = speed, A = save for this distance, X = delete

GAMEPAD 2 (driving)
LEFT STICK: Drive
RIGHT STICK: Turn
LEFT TRIGGER (hold): Auto aim at our hive
RIGHT TRIGGER (hold): Slow mode
START (tap): Make current direction forward

BEFORE START (either gamepad): X = blue alliance, B = red alliance
*/

package org.firstinspires.ftc.teamcode.FinalTeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.IndexIntake;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;
import org.firstinspires.ftc.teamcode.Shared.HubReset;

/**
 * The match teleop. This file only wires things together: it reads the gamepads, hands each
 * mechanism what it needs, and puts the important lines on the Driver Station.
 *
 * Every mechanism is optional. If the intake, the shooter or the camera is missing from the
 * robot config (or its hub did not answer at power-on), that one thing is switched off with
 * a message and everything else still works. Losing one device should never cost a match.
 */
@TeleOp(name = "MAIN TELEOP", group = "TeamCode")
public class MainTeleOp extends OpMode {
    private static final double TRIGGER_HELD = 0.3;

    private Battery battery;
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();
    private final Shooter shooter = new Shooter();
    private final HiveAim aim = new HiveAim();

    private boolean intakeOk;
    private boolean shooterOk;
    private boolean aimOk;
    /** What is switched off and why, shown on the Driver Station for the whole match. */
    private String problems = "";

    // START (the SDK calls it "options") sets field-forward when it is tapped ON ITS OWN.
    // START + A and START + B are how gamepads are paired, and those must not count.
    private boolean startHeld;
    private boolean startAlone;

    // Same idea for picking red with B before the match (START + B pairs gamepad 2).
    private final boolean[] redHeld = new boolean[2];
    private final boolean[] redPressClean = new boolean[2];

    private boolean wasFieldRelative;
    private boolean lostFieldRelative;
    private boolean wasAimed;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        HubReset.clear();

        // AUTO bulk caching: each hub answers all of a loop's encoder reads with one request
        // instead of one request per read. It roughly halves the loop time.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        battery = new Battery(hardwareMap);

        //----------PLAYER ONE-------------

        try {
            intakeIndex.init(hardwareMap, battery);
            intakeOk = true;
        } catch (RuntimeException e) {
            problems += "INTAKE OFF: " + e.getMessage() + "\n";
        }
        try {
            shooter.init(hardwareMap, battery);
            shooterOk = true;
        } catch (RuntimeException e) {
            problems += "FLYWHEEL OFF: " + e.getMessage() + "\n";
        }

        // Both of those failing at once is nearly always the Expansion Hub, not the names.
        if (!intakeOk || !shooterOk) {
            String hub = HubReset.missingHubNote(hardwareMap);
            if (hub != null) problems = hub + "\n";
        }

        //----------PLAYER TWO-------------

        // The drive is the one thing we cannot do without, so it is allowed to stop INIT.
        drive.init(hardwareMap, battery);
        try {
            aim.init(hardwareMap);
            aimOk = true;
        } catch (RuntimeException e) {
            problems += "AUTO AIM OFF: " + e.getMessage() + "\n";
        }
    }

    @Override
    public void init_loop() {
        drive.updateHeading();
        chooseAlliance(gamepad1, 0);
        chooseAlliance(gamepad2, 1);
        telemetry.addLine("ALLIANCE: " + HiveAim.allianceName() + "   (X = blue, B = red)");
        telemetry.addLine("Keep robot STILL during calibration; wait for Pinpoint READY.");
        addDriveTelemetry();
        telemetry.update();
    }

    /** X = blue, B = red. After an auto the alliance is already set, so this is only a backup. */
    private void chooseAlliance(Gamepad gamepad, int index) {
        if (gamepad.x && !gamepad.start) HiveAim.blueAlliance = true;

        // B counts on release, and only if START was never down during the press.
        boolean red = gamepad.b;
        if (red && !redHeld[index]) redPressClean[index] = !gamepad.start;
        if (red && gamepad.start) redPressClean[index] = false;
        if (!red && redHeld[index] && redPressClean[index]) HiveAim.blueAlliance = false;
        redHeld[index] = red;
    }

    @Override
    public void start() {
        // Tell the driver straight away if field-centric is not available.
        if (!drive.isFieldRelative()) {
            gamepad2.rumble(800);
            lostFieldRelative = true;
        }
    }

    @Override
    public void loop() {
        // Sensors first, so everything below works from the same fresh readings.
        drive.updateHeading();
        if (aimOk) aim.update(drive.rawHeading(), drive.isHeadingReady());

        //----------PLAYER ONE-------------

        // The shooter runs before the index so the index knows, this loop, whether the
        // flywheels are ready for the next pollen.
        double feedScale = 1.0;
        if (shooterOk) {
            shooter.setHiveDistance(aimOk ? aim.distanceInches() : Double.NaN);
            shooter.loop(telemetry, gamepad1);
            feedScale = shooter.feedScale();
        }
        if (intakeOk) intakeIndex.loop(gamepad1, telemetry, feedScale);

        //----------PLAYER TWO-------------

        double forward = -gamepad2.left_stick_y;   // pushing a stick up reads negative
        double strafe = gamepad2.left_stick_x;
        double rotate = gamepad2.right_stick_x;

        boolean start = gamepad2.options;
        if (start && !startHeld) startAlone = true;
        if (start && (gamepad2.a || gamepad2.b)) startAlone = false;
        if (!start && startHeld && startAlone) drive.resetFieldHeading();
        startHeld = start;

        drive.setSlow(gamepad2.right_trigger > TRIGGER_HELD);

        // While aiming, the aim controller turns the robot and the driver keeps the left
        // stick. If there is nothing to aim at (NaN) the right stick still works, so the
        // driver can swing the robot round until the camera finds the hive.
        boolean aiming = aimOk && gamepad2.left_trigger > TRIGGER_HELD;
        double aimPower = aiming
                ? aim.rotatePower(drive.rawHeading(), drive.headingRate(), drive.isHeadingReady()) : Double.NaN;
        if (Double.isNaN(aimPower)) {
            drive.driveField(forward, strafe, rotate, false);
        } else {
            drive.driveField(forward, strafe, aimPower, true);
        }
        if (!aiming) aim.release();

        boolean aimed = aiming && aim.isAimed();
        if (aimed && !wasAimed) gamepad2.rumble(150);
        wasAimed = aimed;

        // One long buzz when field-centric is lost, two short ones when it comes back.
        boolean fieldRelative = drive.isFieldRelative();
        if (wasFieldRelative && !fieldRelative) {
            gamepad2.rumble(800);
            lostFieldRelative = true;
        } else if (!wasFieldRelative && fieldRelative && lostFieldRelative) {
            gamepad2.rumbleBlips(2);
            lostFieldRelative = false;
        }
        wasFieldRelative = fieldRelative;

        telemetry.addData("AIM (" + HiveAim.allianceName() + " hive)", aimOk ? aim.status(aiming) : "OFF");
        if (drive.isSlow()) telemetry.addLine("SLOW MODE");
        addDriveTelemetry();
        telemetry.addData("Loop time (ms)", "%.0f", loopTimer.milliseconds());
        loopTimer.reset();
    }

    private void addDriveTelemetry() {
        if (!problems.isEmpty()) telemetry.addLine(problems);
        String hubWarning = HubReset.warning();
        if (hubWarning != null) telemetry.addLine(hubWarning);
        telemetry.addData("Pinpoint", drive.getHeadingStatus());
        telemetry.addData("Field heading (deg)", drive.getFieldHeadingDegrees());
        telemetry.addData("Drive", drive.getDriveStatus());
        drive.addPowerTelemetry(telemetry);
        telemetry.addLine("START (gamepad 2): make current robot direction field-forward");
    }

    @Override
    public void stop() {
        //----------PLAYER ONE-------------

        intakeIndex.stop();
        shooter.stop();

        //----------PLAYER TWO-------------

        drive.stop();
        aim.stop();
    }
}
