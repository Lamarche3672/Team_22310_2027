package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalTeleOp.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.IndexIntake;

@TeleOp(name = "TeleOpCodeTestTwo", group = "TeamCode")
public class MainTeleOp extends OpMode {
    private static final double DRIVE_DEADZONE = 0.05;
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();

    @Override
    public void init() {
        //intake and index
        intakeIndex.init(hardwareMap);
        //mecanum
        drive.init(hardwareMap);
    }

    @Override
    public void loop() {
        //intake and index
        intakeIndex.loop(gamepad1, telemetry);

        //mecanum
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        if (Math.abs(forward) < DRIVE_DEADZONE) {
            forward = 0;
        }
        if (Math.abs(strafe) < DRIVE_DEADZONE) {
            strafe = 0;
        }
        if (Math.abs(rotate) < DRIVE_DEADZONE) {
            rotate = 0;
        }
        drive.driveFieldRelative(forward, strafe, rotate);
    }

    @Override
    public void stop() {
        //intake and index
        intakeIndex.stop();
        //mecanum
        drive.stop();
    }
}