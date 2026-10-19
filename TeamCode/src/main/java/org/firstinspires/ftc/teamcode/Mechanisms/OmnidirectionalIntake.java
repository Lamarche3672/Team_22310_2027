package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Omnidirectional Intake", group = "TeamCode")
public class OmnidirectionalIntake extends OpMode {

    private static final double INTAKE_POWER = -1;
    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
    }

    @Override
    public void loop() {
        double power = 0;

        if (gamepad1.left_bumper) {
            power = INTAKE_POWER;
        } else if (gamepad1.right_bumper) {
            power = -INTAKE_POWER;//goes backwards
        }

        intakeMotor.setPower(power);
        telemetry.addData("Intake power", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
    }
}

//fpor ome \rewson the intake and outake work togetehr in this code