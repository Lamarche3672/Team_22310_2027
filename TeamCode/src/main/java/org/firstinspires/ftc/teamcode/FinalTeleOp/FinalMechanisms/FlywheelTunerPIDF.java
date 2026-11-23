package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelTunerPIDF {

    public DcMotorEx flywheelMotor;

    public DcMotorEx flywheelMotor2;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    // What was last sent to the hub, so each loop only sends what changed.
    private boolean spinning;
    private double sentVelocity = Double.NaN;
    private double sentP = Double.NaN;
    private double sentF = Double.NaN;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        sentP = P;
        sentF = F;

        // Start with the wheels off. They only run while RIGHT BUMPER is held.
        stop();

        telemetry.addLine("Init Complete");
    }

    /** Wheels off and coasting: no power is used until RIGHT BUMPER is held again. */
    public void stop() {
        for (DcMotorEx motor : new DcMotorEx[] {flywheelMotor, flywheelMotor2}) {
            if (motor == null) continue;
            motor.setVelocity(0);
            motor.setMotorDisable(); // setVelocity() switches the motor back on by itself
        }
        spinning = false;
        sentVelocity = Double.NaN;
    }


    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        //get all our gampad commands
        //get target velocity
        //update telemetery
        // RIGHT BUMPER held = wheels spin. Released = wheels off, so they stop using the battery.
        boolean wantSpin = gamepad1.right_bumper;

        // Y switches between the high and the low speed, once per press.
        if (gamepad1.yWasPressed()) {
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

        //set new PIDF coefficients, only when the d-pad changed them
        if (P != sentP || F != sentF) {
            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
            flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            sentP = P;
            sentF = F;
        }

        //set target velocity while the bumper is held; switch the wheels off when it is released
        if (wantSpin) {
            if (!spinning || curTargetVelocity != sentVelocity) {
                flywheelMotor.setVelocity(curTargetVelocity);
                flywheelMotor2.setVelocity(curTargetVelocity);
                sentVelocity = curTargetVelocity;
                spinning = true;
            }
        } else if (spinning) {
            stop();
        }

        double currentVelocity = flywheelMotor.getVelocity();
        double currentVelocity2 = flywheelMotor2.getVelocity();

        double error = curTargetVelocity - currentVelocity;
        double error2 = curTargetVelocity - currentVelocity2;


        telemetry.addData("Flywheels", spinning ? "ON (right bumper)" : "off - hold RIGHT BUMPER to spin");
        telemetry.addData("Target Velocity", "%.0f (Y switches high / low)", curTargetVelocity);
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
