/*
GAMEPAD 1 (mechanisms)
RIGHT TRIGGER: Intake
LEFT TRIGGER: Outtakes everything, but at different speeds (index, intake, shooter)
RIGHT BUMPER (hold): Spin the shooter to the speed for this distance
LEFT BUMPER (hold): Index inwards = shoot (wait for the buzz)
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

import org.firstinspires.ftc.teamcode.SavedBackupCode.Drive.Mechanisms.FlywheelTesting;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.FlywheelTunerPIDF;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.HiveAim;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.HubReset;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.IndexIntake;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.TrackingSystem.AprilTagAutoAlignMecanum;

@TeleOp(name = "MAIN TELEOP", group = "TeamCode")




public class MainTeleOp extends OpMode {
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();
    private final FlywheelTesting flywheelTesting = new FlywheelTesting();
    private final FlywheelTunerPIDF FlywheelTunerPIDF = new FlywheelTunerPIDF();
    private final AprilTagAutoAlignMecanum aprilTag = new AprilTagAutoAlignMecanum();
    private boolean optionsHeld;
    private boolean optionsAlone;
    private boolean wasFieldRelative;
    private boolean lostFieldRelative;
    private boolean intakeOk;
    private boolean flywheelOk;
    private final HiveAim aim = new HiveAim();
    private boolean aimOk;
    private boolean aiming;
    private boolean wasAimed;
    private final boolean[] redPressClean = new boolean[2];
    private final boolean[] redHeld = new boolean[2];
    private String problems = "";
    private final ElapsedTime loopTimer = new ElapsedTime();



    @Override
    public void init() {
        HubReset.clear();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //----------PLAYER ONE-------------

        try {
            intakeIndex.init(hardwareMap);
            intakeOk = true;
        } catch (RuntimeException e) {
            problems += "INTAKE OFF: " + e.getMessage() + "\n";
        }
        //flywheelTesting.init(hardwareMap);
        try {
            FlywheelTunerPIDF.init(hardwareMap, telemetry);
            flywheelOk = true;
        } catch (RuntimeException e) {
            problems += "FLYWHEEL OFF: " + e.getMessage() + "\n";
        }

        //----------PLAYER TWO-------------

        drive.init(hardwareMap);
        //aprilTag.init();
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

    private void chooseAlliance(Gamepad gamepad, int index) {
        if (gamepad.x && !gamepad.start) HiveAim.blueAlliance = true;
        boolean red = gamepad.b;
        if (red && !redHeld[index]) redPressClean[index] = !gamepad.start;
        if (red && gamepad.start) redPressClean[index] = false;
        if (!red && redHeld[index] && redPressClean[index]) HiveAim.blueAlliance = false;
        redHeld[index] = red;
    }

    @Override
    public void start() {
        if (!drive.isFieldRelative()) {
            gamepad2.rumble(800);
            lostFieldRelative = true;
        }
    }

    @Override
    public void loop() {
        drive.updateHeading();
        if (aimOk) aim.update(drive.rawHeading(), drive.isHeadingReady());

        //----------PLAYER ONE-------------

        if (flywheelOk) {
            FlywheelTunerPIDF.setHiveDistance(aimOk ? aim.distanceInches() : Double.NaN);
            FlywheelTunerPIDF.loop(telemetry, gamepad1);
        }
        //flywheelTesting.loop(gamepad1, telemetry);
        if (intakeOk) intakeIndex.loop(gamepad1, telemetry);

        //----------PLAYER TWO-------------

        //aprilTag.loop();
        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double rotate = gamepad2.right_stick_x;
        boolean options = gamepad2.options;
        if (options && !optionsHeld) optionsAlone = true;
        if (options && (gamepad2.a || gamepad2.b)) optionsAlone = false;
        if (!options && optionsHeld && optionsAlone) {
            drive.resetFieldHeading();
        }
        optionsHeld = options;

        drive.setSlow(gamepad2.right_trigger > 0.3);
        aiming = aimOk && gamepad2.left_trigger > 0.3;
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

        telemetry.addData("AIM (" + HiveAim.allianceName() + " hive)", aimOk ? aim.status(aiming) : "OFF");
        if (drive.isSlow()) telemetry.addLine("SLOW MODE");
        addDriveTelemetry();

        boolean fieldRelative = drive.isFieldRelative();
        if (wasFieldRelative && !fieldRelative) {
            gamepad2.rumble(800);
            lostFieldRelative = true;
        } else if (!wasFieldRelative && fieldRelative && lostFieldRelative) {
            gamepad2.rumbleBlips(2);
            lostFieldRelative = false;
        }
        wasFieldRelative = fieldRelative;

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
        FlywheelTunerPIDF.stop();
        //flywheelTesting.stop();

        //----------PLAYER TWO-------------

        drive.stop();
        aim.stop();

    }
}
