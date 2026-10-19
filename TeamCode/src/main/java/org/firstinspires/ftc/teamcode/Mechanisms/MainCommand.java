package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpCodeTestOne", group = "TeamCode")
public class MainCommand extends OpMode {

    private static final double INTAKE_POWER = 1;
    private DcMotor intakeMotor;

    private static final double INDEX_POWER = 0.6;
    private DcMotor indexMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        indexMotor = hardwareMap.get(DcMotor.class, "index_motor");
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setPower(0);
    }

    @Override
    public void loop() {
        double power = 0;

        if (gamepad1.right_bumper) {
            power = INTAKE_POWER;
        } else if (gamepad1.left_bumper) {
            power = -INTAKE_POWER;
        }

        intakeMotor.setPower(power);
        telemetry.addData("Intake power", power);
        telemetry.update();

        double power2 = 0;

        if (gamepad1.right_bumper) {
            power2 = INDEX_POWER;
        } else if (gamepad1.left_bumper) {
            power2 = -INDEX_POWER;
        }

        intakeMotor.setPower(power2);
        telemetry.addData("Index power", power2);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
    }
}
