package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Index", group = "TeamCode")
public class Index extends OpMode {

    private static final double INDEX_POWER = 0.6;
    private DcMotor indexMotor;

    @Override
    public void init() {
        indexMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setPower(0);
    }

    @Override
    public void loop() {
        double power = 0;

        if (gamepad1.a) {
            power = INDEX_POWER;
        } else if (gamepad1.b) {
            power = -INDEX_POWER; //goes backwards
        }

        indexMotor.setPower(power);
        telemetry.addData("Index power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        indexMotor.setPower(0);
    }
}
