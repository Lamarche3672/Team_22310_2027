/*
LEFT BUMPER: Index inwards
LEFT TRIGGER: Outtakes everything, but at different speeds (index, intake, shooter)
RIGHT TRIGGER: Intake
RIGHT BUMPER: Flywheels (hold)
Y: Flywheel speed high/low
BACK: Unlock flywheel tuning
D-PAD: Tune P and F, X changes step size
A: Save P and F
*/

package org.firstinspires.ftc.teamcode.FinalTeleOp;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SavedBackupCode.Drive.Mechanisms.FlywheelTesting;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms.FlywheelTunerPIDF;
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
    private boolean headingResetPressed;
    private boolean wasFieldRelative;
    private boolean lostFieldRelative;
    private final ElapsedTime loopTimer = new ElapsedTime();



    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //----------PLAYER ONE-------------

        intakeIndex.init(hardwareMap);
        //flywheelTesting.init(hardwareMap);
        FlywheelTunerPIDF.init(hardwareMap, telemetry);

        //----------PLAYER TWO-------------

        drive.init(hardwareMap);
        //aprilTag.init();
    }

    @Override
    public void init_loop() {
        drive.updateHeading();
        telemetry.addLine("Keep robot STILL during calibration; wait for Pinpoint READY.");
        addDriveTelemetry();
        telemetry.update();
    }

    @Override
    public void loop() {
        //----------PLAYER ONE-------------

        intakeIndex.loop(gamepad1, telemetry);
        //flywheelTesting.loop(gamepad1, telemetry);
        FlywheelTunerPIDF.loop(telemetry, gamepad1);

        //----------PLAYER TWO-------------

        //aprilTag.loop();
        double forward = -gamepad2.left_stick_y;
        double strafe = gamepad2.left_stick_x;
        double rotate = gamepad2.right_stick_x;
        boolean headingReset = gamepad2.options && !gamepad2.a && !gamepad2.b;
        if (headingReset && !headingResetPressed) {
            drive.resetFieldHeading();
        }
        headingResetPressed = headingReset;

        drive.driveFieldRelative(forward, strafe, rotate);
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
        telemetry.addData("Pinpoint", drive.getHeadingStatus());
        telemetry.addData("Field heading (deg)", drive.getFieldHeadingDegrees());
        telemetry.addData("Drive", drive.getDriveStatus());
        drive.addPowerTelemetry(telemetry);
        telemetry.addLine("Options: make current robot direction field-forward");

    }

    @Override
    public void stop() {
        //----------PLAYER ONE-------------

        intakeIndex.stop();
        FlywheelTunerPIDF.stop();
        //flywheelTesting.stop();

        //----------PLAYER TWO-------------

        drive.stop();

    }
}
