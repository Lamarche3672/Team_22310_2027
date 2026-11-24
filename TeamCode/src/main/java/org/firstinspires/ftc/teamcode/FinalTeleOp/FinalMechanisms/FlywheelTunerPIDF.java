package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelControl;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelTuning;

public class FlywheelTunerPIDF {

    public FlywheelControl flywheels;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double I = 0;

    double[] stepSizes = {1.0, 0.1, 0.01};

    int stepIndex = 1;

    private static final double COAST_BELOW = 200;

    private double sentP = Double.NaN;
    private double sentF = Double.NaN;
    private boolean tuningUnlocked;
    private boolean wasAtSpeed;
    private long lastRumbleNanos;
    private String tuningSource = "typed into this file";
    private String saveMessage = "";
    private long saveMessageNanos;
    private long fullSpeedSinceNanos;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheels = new FlywheelControl(hardwareMap, DcMotorSimple.Direction.FORWARD);

        if (P == 0 && F == 0) {
            double[] start = FlywheelAuto.chooseGains();
            P = start[0];
            F = start[1];
            I = start[2];
            tuningSource = FlywheelAuto.describeGainSource();
        }
        sendGains();

        stop();

        telemetry.addLine("Init Complete");
    }

    private void sendGains() {
        flywheels.setGains(P, F, I);
        sentP = P;
        sentF = F;
    }

    public void stop() {
        if (flywheels != null) flywheels.stop();
    }

    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        long now = System.nanoTime();

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.backWasPressed()) {
            tuningUnlocked = !tuningUnlocked;
        }

        boolean stepPressed = gamepad1.xWasPressed();
        boolean fDownPressed = gamepad1.dpadLeftWasPressed();
        boolean fUpPressed = gamepad1.dpadRightWasPressed();
        boolean pUpPressed = gamepad1.dpadUpWasPressed();
        boolean pDownPressed = gamepad1.dpadDownWasPressed();
        boolean savePressed = gamepad1.aWasPressed() && !gamepad1.start;

        if (tuningUnlocked) {
            if (stepPressed) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (fDownPressed) F -= stepSizes[stepIndex];
            if (fUpPressed) F += stepSizes[stepIndex];
            if (pUpPressed) P += stepSizes[stepIndex];
            if (pDownPressed) P -= stepSizes[stepIndex];

            P = Math.max(0, P);
            F = Math.max(0, F);
            if (P != sentP || F != sentF) tuningSource = "the gamepad (not saved yet)";

            if (savePressed) {
                boolean saved = FlywheelTuning.save(P, F, I);
                saveMessage = saved ? "SAVED for teleop and auto" : "could not save";
                if (saved) tuningSource = "the gamepad (saved)";
                saveMessageNanos = now;
            }
        }

        if (P != sentP || F != sentF) {
            sendGains();
        }

        boolean wantSpin = gamepad1.right_bumper;
        boolean wantReverse = !wantSpin && gamepad1.left_trigger > 0.1;
        double wantedVelocity = wantSpin ? curTargetVelocity : wantReverse ? -lowVelocity : 0;

        double measured = flywheels.speed();
        boolean coasting = (wantedVelocity < 0 && measured > COAST_BELOW)
                || (wantedVelocity > 0 && measured < -COAST_BELOW);
        if (coasting) wantedVelocity = 0;

        if (wantedVelocity != flywheels.target()) {
            if (wantedVelocity == 0) flywheels.stop();
            else flywheels.setTarget(wantedVelocity);
        }
        flywheels.update();

        double currentVelocity = flywheels.rightVelocity();
        double currentVelocity2 = flywheels.leftVelocity();
        double error = curTargetVelocity - currentVelocity;
        double error2 = curTargetVelocity - currentVelocity2;

        boolean atSpeed = wantSpin && !coasting
                && Math.abs(error) <= FlywheelAuto.READY_TOLERANCE
                && Math.abs(error2) <= FlywheelAuto.READY_TOLERANCE;
        if (atSpeed && !wasAtSpeed && now - lastRumbleNanos > 500_000_000L) {
            gamepad1.rumble(200);
            lastRumbleNanos = now;
        }
        wasAtSpeed = atSpeed;

        if (wantSpin && !coasting && flywheels.power() > 0.2) {
            if (fullSpeedSinceNanos == 0) fullSpeedSinceNanos = now;
        } else {
            fullSpeedSinceNanos = 0;
        }
        boolean noReading = fullSpeedSinceNanos != 0 && now - fullSpeedSinceNanos > 1_000_000_000L
                && (Math.abs(currentVelocity) < 50 || Math.abs(currentVelocity2) < 50);

        telemetry.addData("Flywheels", atSpeed ? "AT SPEED - feed now" : coasting ? "slowing down first..."
                : wantSpin ? "spinning up..."
                : wantReverse ? "reversing (left trigger)" : "off - hold RIGHT BUMPER to spin");
        if (noReading) telemetry.addLine("A FLYWHEEL READS 0: check its encoder cable or a jam");
        telemetry.addData("Target Velocity", "%.0f (Y switches high / low)", curTargetVelocity);
        telemetry.addData("Current Velocity (right)", "%.2f", currentVelocity);
        telemetry.addData("Current Velocity (left)", "%.2f", currentVelocity2);

        telemetry.addData("Error (right)", "%.2f", error);
        telemetry.addData("Error (left)", "%.2f", error2);
        telemetry.addData("Flywheel power", "%.2f", flywheels.power());

        telemetry.addLine("--------------------");
        if (tuningUnlocked) {
            telemetry.addLine("TUNING UNLOCKED (BACK locks it again)");
            telemetry.addData("Tuning P", "%.3f (D-Pad U/D)", P);
            telemetry.addData("Tuning F", "%.3f (D-Pad L/R)", F);
            telemetry.addData("Step Size", "%.3f (X Button)", stepSizes[stepIndex]);
            telemetry.addData("A Button", now - saveMessageNanos < 3_000_000_000L && !saveMessage.isEmpty()
                    ? saveMessage : "save this P and F for teleop and auto");
        } else {
            telemetry.addData("Tuning", "locked: P %.3f, F %.3f (press BACK to tune)", P, F);
        }
        telemetry.addData("P and F came from", tuningSource);
    }
}
