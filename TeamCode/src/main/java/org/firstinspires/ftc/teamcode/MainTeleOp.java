/*
LEFT BUMPER: Index inwards
LEFT TRIGGER: Outtakes everything, but at different speeds (index, intake, shooter)
RIGHT TRIGGER: Intake
RIGHT BUMPER: Flywheels/shooter
*/

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalTeleOp.FlywheelTesting;
import org.firstinspires.ftc.teamcode.FinalTeleOp.FlywheelTunerPIDF;
import org.firstinspires.ftc.teamcode.FinalTeleOp.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.IndexIntake;
import org.firstinspires.ftc.teamcode.FinalTeleOp.TrackingSystem.AprilTagAutoAlignMecanum;

@TeleOp(name = "MAIN TELEOP", group = "TeamCode")




public class MainTeleOp extends OpMode {
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();
    private final FlywheelTesting flywheelTesting = new FlywheelTesting();
    private final FlywheelTunerPIDF FlywheelTunerPIDF = new FlywheelTunerPIDF();
    private final AprilTagAutoAlignMecanum aprilTag = new AprilTagAutoAlignMecanum();
    private boolean headingResetPressed;



    @Override
    public void init() {
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
        if (gamepad2.options && !headingResetPressed) {
            drive.resetFieldHeading();
        }
        headingResetPressed = gamepad2.options;

        drive.driveFieldRelative(forward, strafe, rotate);
        addDriveTelemetry();
    }

    private void addDriveTelemetry() {
        telemetry.addData("Pinpoint", drive.getHeadingStatus());
        telemetry.addData("Field heading (deg)", drive.getFieldHeadingDegrees());
        telemetry.addData("Drive", drive.isHeadingReady() ? "Ready" : "STOPPED: heading unavailable");
        telemetry.addLine("Options: make current robot direction field-forward");
    }

    @Override
    public void stop() {
        //----------PLAYER ONE-------------

        intakeIndex.stop();
        //flywheelTesting.stop();

        //----------PLAYER TWO-------------

        drive.stop();

    }
}
