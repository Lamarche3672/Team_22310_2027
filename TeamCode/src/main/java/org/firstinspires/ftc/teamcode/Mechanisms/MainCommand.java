package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpCodeTestOne", group = "TeamCode")
public class MainCommand extends OpMode {

    private static final double INTAKE_POWER = 1.0;
    private static final double INDEX_POWER = 1.0;

    private DcMotor intakeMotor;
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
        double intakePower = 0.0;

        if (gamepad1.right_bumper) {
            intakePower = INTAKE_POWER;
        } else if (gamepad1.left_bumper) {
            intakePower = -INTAKE_POWER;
        }

        double indexPower = 0.0;

        if (gamepad1.a) {
            indexPower = INDEX_POWER;
        } else if (gamepad1.b) {
            indexPower = -INDEX_POWER;
        }

        intakeMotor.setPower(intakePower);
        indexMotor.setPower(indexPower);

        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Index power", indexPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0.0);
        indexMotor.setPower(0.0);
    }
}