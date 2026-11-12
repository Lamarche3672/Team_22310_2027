package org.firstinspires.ftc.teamcode.FinalTeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelTesting {

    private static final double LEFT_POWER = 1.0;
    private static final double RIGHT_POWER = 1.0;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public void init(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, "left_flywheel");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setPower(0);

        rightMotor = hardwareMap.get(DcMotor.class, "right_flywheel");
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setPower(0);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        double leftPower = 0.0;
        if (gamepad.right_bumper) {
            leftPower = -LEFT_POWER;
        } else if (gamepad.left_trigger > 0.1) {
            leftPower = LEFT_POWER;
        }

        double rightPower = 0.0;
        if (gamepad.right_bumper) {
            rightPower = -RIGHT_POWER;
        } else if (gamepad.left_trigger > 0.1) {
            rightPower = RIGHT_POWER + 0.5;
        }

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        telemetry.addData("Intake power", leftPower);
        telemetry.addData("Index power", rightPower);
        telemetry.update();
    }

    public void stop() {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}