/*package org.firstinspires.ftc.teamcode.SavedBackupCode.Drive.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Flywheel Test", group = "TeamCode")

public class FlywheelTunerPIDFTesting extends OpMode {

    public DcMotorEx flywheelMotor;

    public DcMotorEx flywheelMotor2;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        //get all our gampad commands
        //get target velocity
        //update telemetery
        if (gamepad1.right_bumper) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.xWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set target velocity
        flywheelMotor.setVelocity(curTargetVelocity);

        flywheelMotor2.setVelocity(curTargetVelocity);

        double currentVelocity = flywheelMotor.getVelocity();
        double currentVelocity2 = flywheelMotor2.getVelocity();

        double error = curTargetVelocity - currentVelocity;
        double error2 = curTargetVelocity - currentVelocity;


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity2);

        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Error", "%.2f", error2);

        telemetry.addLine("--------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
// keep watching video to see how to tun cuz it wont likely work the first time boiiiiiii
// also fix Velcity in Lib*/
