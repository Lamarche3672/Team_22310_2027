package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IndexIntake {

    private static final double INTAKE_POWER = 0.95;
    private static final double INDEX_POWER = 0.95;

    private DcMotor intakeMotor;
    private DcMotor indexMotor;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);

        indexMotor = hardwareMap.get(DcMotor.class, "index_motor");
        indexMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setPower(0);
    }

    public void loop(Gamepad gamepad, Telemetry telemetry) {
        double intakePower = 0.0;
        if (gamepad.left_trigger > 0.1) {
            intakePower = -INTAKE_POWER;
        } else if (gamepad.right_trigger > 0.1) {
            intakePower = INTAKE_POWER;
        }

        double indexPower = 0.0;
        if (gamepad.left_trigger > 0.1) {
            indexPower = INDEX_POWER - 0.5;
        } else if (gamepad.left_bumper) {
            indexPower = -INDEX_POWER;
        }

        write(intakePower, indexPower);

        telemetry.addData("Intake power", intakePower);
        telemetry.addData("Index power", indexPower);
    }

    public void stop() {
        if (intakeMotor == null || indexMotor == null) return;
        write(0.0, 0.0);
    }

    private void write(double intakePower, double indexPower) {
        try {
            intakeMotor.setPower(intakePower);
            indexMotor.setPower(indexPower);
        } catch (TargetPositionNotSetException hubReset) {
            HubReset.forget(intakeMotor, indexMotor);
            try {
                intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                intakeMotor.setPower(intakePower);
                indexMotor.setPower(indexPower);
            } catch (TargetPositionNotSetException stillDown) {
            }
        }
    }
}