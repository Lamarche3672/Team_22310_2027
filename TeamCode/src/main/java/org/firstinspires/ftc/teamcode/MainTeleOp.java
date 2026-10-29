package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalTeleOp.MecanumDriveFinal;
import org.firstinspires.ftc.teamcode.FinalTeleOp.IndexIntake;

@TeleOp(name = "TeleOpCodeTestTwo", group = "TeamCode")
public class MainTeleOp extends OpMode {
    private final MecanumDriveFinal drive = new MecanumDriveFinal();
    private final IndexIntake intakeIndex = new IndexIntake();

    @Override
    public void init() {
        //intake and index
        intakeIndex.init();
        //mecanum
        drive.init();
    }

    @Override
    public void loop() {
        //intake and index
        intakeIndex.loop();
        //mecanum
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
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