package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Intake", group = "TeamCode")
public class Intake{

    private DcMotor intakeMotor;

    boolean intakeButton;
    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "intake_motor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(double forward) {
        intakeButton = gamepad1.right_bumper;
        double maxPower = 0.6;
        double maxSpeed = 0.6;

        if(intakeButton){
            intakeMotor.setPower(maxSpeed * (forward / maxPower));
        }
    }
}

