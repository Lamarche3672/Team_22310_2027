package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalTeleOp.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.IndexIntake;
import org.firstinspires.ftc.teamcode.TrackingSystem.AprilTagAutoAlignMecanum;

@TeleOp(name = "MAIN TELEOP", group = "TeamCode")
public class MainTeleOp extends OpMode {
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();
    private final AprilTagAutoAlignMecanum aprilTag = new AprilTagAutoAlignMecanum();
    private boolean headingResetPressed;

    @Override
    public void init() {
        //intake and index
        intakeIndex.init(hardwareMap);
        //mecanum
        drive.init(hardwareMap);
        aprilTag.init();
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
        //mecanum
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        if (gamepad1.options && !headingResetPressed) {
            drive.resetFieldHeading();
        }
        headingResetPressed = gamepad1.options;

        drive.driveFieldRelative(forward, strafe, rotate);
        addDriveTelemetry();
        // IndexIntake.loop also sends the combined telemetry packet.
        intakeIndex.loop(gamepad1, telemetry);

        aprilTag.loop();
    }

    private void addDriveTelemetry() {
        telemetry.addData("Pinpoint", drive.getHeadingStatus());
        telemetry.addData("Field heading (deg)", drive.getFieldHeadingDegrees());
        telemetry.addData("Drive", drive.isHeadingReady() ? "Ready" : "STOPPED: heading unavailable");
        telemetry.addLine("Options: make current robot direction field-forward");
    }

    @Override
    public void stop() {
        //intake and index
        intakeIndex.stop();
        //mecanum
        drive.stop();
    }
}
