package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelTuning;

public class FlywheelTunerPIDF {

    public DcMotorEx flywheelMotor;

    public DcMotorEx flywheelMotor2;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    double I = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    private static final double SPIN_UP_SECONDS = 0.3;
    private static final long VOLTAGE_READ_NANOS = 100_000_000L;
    private static final long RESEND_NANOS = 250_000_000L;
    private static final long IDLE_READ_NANOS = 250_000_000L;
    private static final double COAST_BELOW = 200;

    private boolean spinning;
    private double commandedVelocity;
    private long lastSendNanos;
    private long lastLoopNanos;
    private long lastReadNanos;
    private double currentVelocity;
    private double currentVelocity2;
    private double sentP = Double.NaN;
    private double sentF = Double.NaN;
    private boolean tuningUnlocked;
    private boolean wasAtSpeed;
    private long lastRumbleNanos;
    private String tuningSource = "typed into this file";
    private String saveMessage = "";
    private long saveMessageNanos;
    private long fullSpeedSinceNanos;
    private VoltageSensor battery;
    private double batteryVolts = Double.NaN;
    private long lastVoltageNanos;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (P == 0 && F == 0) {
            PIDFCoefficients start = FlywheelAuto.chooseCoefficients(flywheelMotor);
            P = start.p;
            I = start.i;
            F = start.f;
            tuningSource = FlywheelAuto.describeCoefficientSource();
        }
        sendCoefficients();

        stop();

        telemetry.addLine("Init Complete");
    }

    private void sendCoefficients() {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        sentP = P;
        sentF = F;
    }

    public void stop() {
        spinning = false;
        commandedVelocity = 0;
        try {
            release();
        } catch (TargetPositionNotSetException hubReset) {
            recoverFromHubReset();
            try {
                release();
            } catch (TargetPositionNotSetException stillDown) {
            }
        }
    }

    private void release() {
        for (DcMotorEx motor : new DcMotorEx[] {flywheelMotor, flywheelMotor2}) {
            if (motor == null) continue;
            motor.setVelocity(0);
            motor.setMotorDisable();
        }
    }

    private void recoverFromHubReset() {
        HubReset.forget(flywheelMotor, flywheelMotor2);
        try {
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sendCoefficients();
        } catch (TargetPositionNotSetException stillDown) {
            sentP = Double.NaN;
        }
        spinning = false;
        lastSendNanos = 0;
    }

    private void readBattery(long now) {
        if (battery == null || now - lastVoltageNanos < VOLTAGE_READ_NANOS) return;
        lastVoltageNanos = now;
        double volts = battery.getVoltage();
        if (Double.isFinite(volts) && volts > 1.0) batteryVolts = volts;
    }

    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        try {
            step(telemetry, gamepad1);
        } catch (TargetPositionNotSetException hubReset) {
            recoverFromHubReset();
        }
    }

    private void step(Telemetry telemetry, Gamepad gamepad1) {
        long now = System.nanoTime();
        double dt = lastLoopNanos == 0 ? 0.0 : Math.min(0.1, (now - lastLoopNanos) / 1e9);
        lastLoopNanos = now;

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
            sendCoefficients();
        }

        boolean wantSpin = gamepad1.right_bumper;
        boolean wantReverse = !wantSpin && gamepad1.left_trigger > 0.1;
        double wantedVelocity = wantSpin ? curTargetVelocity : wantReverse ? -lowVelocity : 0;

        if (spinning || wantedVelocity != 0 || now - lastReadNanos >= IDLE_READ_NANOS) {
            currentVelocity = flywheelMotor.getVelocity();
            currentVelocity2 = flywheelMotor2.getVelocity();
            lastReadNanos = now;
        }
        double measured = 0.5 * (currentVelocity + currentVelocity2);
        boolean coasting = (wantedVelocity < 0 && measured > COAST_BELOW)
                || (wantedVelocity > 0 && measured < -COAST_BELOW);
        if (coasting) wantedVelocity = 0;

        if (wantedVelocity != 0) {
            if (!spinning) {
                commandedVelocity = wantedVelocity > 0
                        ? Math.max(0, Math.min(wantedVelocity, measured))
                        : Math.min(0, Math.max(wantedVelocity, measured));
            }
            readBattery(now);
            double maxStep = dt * highVelocity / SPIN_UP_SECONDS * FlywheelAuto.rampScale(batteryVolts);
            double nextVelocity = commandedVelocity
                    + Math.max(-maxStep, Math.min(maxStep, wantedVelocity - commandedVelocity));
            if (!spinning || nextVelocity != commandedVelocity || now - lastSendNanos >= RESEND_NANOS) {
                flywheelMotor.setVelocity(nextVelocity);
                flywheelMotor2.setVelocity(nextVelocity);
                lastSendNanos = now;
            }
            commandedVelocity = nextVelocity;
            spinning = true;
        } else if (spinning) {
            stop();
        }

        double error = curTargetVelocity - currentVelocity;
        double error2 = curTargetVelocity - currentVelocity2;


        boolean atSpeed = wantSpin && commandedVelocity == curTargetVelocity
                && Math.abs(error) <= FlywheelAuto.READY_TOLERANCE
                && Math.abs(error2) <= FlywheelAuto.READY_TOLERANCE;
        if (atSpeed && !wasAtSpeed && now - lastRumbleNanos > 500_000_000L) {
            gamepad1.rumble(200);
            lastRumbleNanos = now;
        }
        wasAtSpeed = atSpeed;

        if (wantSpin && !coasting && commandedVelocity == curTargetVelocity) {
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

        telemetry.addLine("--------------------");
        if (tuningUnlocked) {
            telemetry.addLine("TUNING UNLOCKED (BACK locks it again)");
            telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
            telemetry.addData("Tuning F", "%.4f (D-Pad L/R)", F);
            telemetry.addData("Step Size", "%.4f (X Button)", stepSizes[stepIndex]);
            telemetry.addData("A Button", now - saveMessageNanos < 3_000_000_000L && !saveMessage.isEmpty()
                    ? saveMessage : "save this P and F for teleop and auto");
        } else {
            telemetry.addData("Tuning", "locked: P %.4f, F %.4f (press BACK to tune)", P, F);
        }
        telemetry.addData("P and F came from", tuningSource);
    }
}
