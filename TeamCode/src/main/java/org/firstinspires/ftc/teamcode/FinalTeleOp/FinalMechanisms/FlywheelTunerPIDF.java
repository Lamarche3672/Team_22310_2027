package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelTuning;

/**
 * Shooter wheels for MAIN TELEOP (gamepad 1).
 *
 *   RIGHT BUMPER (hold)   wheels spin; let go = off and coasting
 *   Y                     high / low speed
 *   LEFT TRIGGER          wheels turn slowly backwards (outtake)
 *   BACK                  unlock / lock tuning. Only while unlocked:
 *       D-PAD up / down = P     D-PAD left / right = F     X = step size     A = save
 */
public class FlywheelTunerPIDF {

    public DcMotorEx flywheelMotor;

    public DcMotorEx flywheelMotor2;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;
    /** The integral part. Not tuned from the gamepad; it comes with the starting values. */
    double I = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    /**
     * The wheels are walked up to their speed over about this long instead of being thrown to
     * full power in one step. Two motors starting from rest at full power is a big hit on the battery.
     */
    private static final double SPIN_UP_SECONDS = 0.4;
    /** While spinning, the speed is sent again this often, so the wheels come back after a hub brown-out. */
    private static final long RESEND_NANOS = 250_000_000L;
    /** While the wheels are off their speed is only read this often, to keep the loop fast. */
    private static final long IDLE_READ_NANOS = 250_000_000L;

    // What was last sent to the hub, so each loop only sends what changed.
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

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // P = 0 and F = 0 give the wheels no power at all, so the shooter did nothing until
        // someone tuned it with the d-pad on every run. Until tuned numbers are typed in above,
        // start from the numbers autonomous uses (see FlywheelAuto.chooseCoefficients): P and F
        // typed into FlywheelAuto, else the ones saved with the A button, else REV's starting
        // values for this motor. The d-pad still adjusts from there.
        if (P == 0 && F == 0) {
            PIDFCoefficients start = FlywheelAuto.chooseCoefficients(flywheelMotor);
            P = start.p;
            I = start.i;
            F = start.f;
            tuningSource = FlywheelAuto.describeCoefficientSource();
        }
        sendCoefficients();

        // Start with the wheels off. They only run while RIGHT BUMPER is held.
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

    /** Wheels off and coasting: no power is used until RIGHT BUMPER is held again. */
    public void stop() {
        for (DcMotorEx motor : new DcMotorEx[] {flywheelMotor, flywheelMotor2}) {
            if (motor == null) continue;
            motor.setVelocity(0);
            motor.setMotorDisable(); // setVelocity() switches the motor back on by itself
        }
        spinning = false;
        commandedVelocity = 0;
    }


    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        long now = System.nanoTime();
        double dt = lastLoopNanos == 0 ? 0.0 : Math.min(0.1, (now - lastLoopNanos) / 1e9);
        lastLoopNanos = now;

        // Y switches between the high and the low speed, once per press.
        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        // BACK unlocks and locks tuning, so a bumped d-pad cannot change P or F in a match.
        if (gamepad1.backWasPressed()) {
            tuningUnlocked = !tuningUnlocked;
        }

        // The tuning buttons are read every loop, also while locked: a press is remembered until
        // it is read, and a remembered press must not go off the moment tuning is unlocked.
        boolean stepPressed = gamepad1.xWasPressed();
        boolean fDownPressed = gamepad1.dpadLeftWasPressed();
        boolean fUpPressed = gamepad1.dpadRightWasPressed();
        boolean pUpPressed = gamepad1.dpadUpWasPressed();
        boolean pDownPressed = gamepad1.dpadDownWasPressed();
        // START + A is how gamepad 1 is paired. That is not a save.
        boolean savePressed = gamepad1.aWasPressed() && !gamepad1.start;

        if (tuningUnlocked) {
            if (stepPressed) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }
            if (fDownPressed) F -= stepSizes[stepIndex];
            if (fUpPressed) F += stepSizes[stepIndex];
            if (pUpPressed) P += stepSizes[stepIndex];
            if (pDownPressed) P -= stepSizes[stepIndex];

            // P and F below zero make the wheels push against themselves.
            P = Math.max(0, P);
            F = Math.max(0, F);

            // A saves the P and F on screen. Teleop and autonomous both start from them from
            // then on, also after the robot has been switched off.
            if (savePressed) {
                saveMessage = FlywheelTuning.save(P, F, I) ? "SAVED for teleop and auto" : "could not save";
                saveMessageNanos = now;
            }
        }

        //set new PIDF coefficients, only when the d-pad changed them
        if (P != sentP || F != sentF) {
            sendCoefficients();
        }

        // RIGHT BUMPER held = wheels spin. Released = wheels off, so they stop using the battery.
        boolean wantSpin = gamepad1.right_bumper;
        // LEFT TRIGGER outtakes everything (intake, index and shooter, see MainTeleOp): the wheels
        // turn slowly backwards so a pollen sitting in the shooter is pushed back out as well.
        boolean wantReverse = !wantSpin && gamepad1.left_trigger > 0.1;
        double wantedVelocity = wantSpin ? curTargetVelocity : wantReverse ? -lowVelocity : 0;

        //set target velocity while a button asks for it; switch the wheels off when it is released
        if (wantedVelocity != 0) {
            // Walk the speed up (or down) to the wanted one instead of jumping there.
            double maxStep = dt * highVelocity / SPIN_UP_SECONDS;
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

        // Reading the speed is a request to the hub. While the wheels are off it is only for show.
        if (spinning || now - lastReadNanos >= IDLE_READ_NANOS) {
            currentVelocity = flywheelMotor.getVelocity();
            currentVelocity2 = flywheelMotor2.getVelocity();
            lastReadNanos = now;
        }

        double error = curTargetVelocity - currentVelocity;
        double error2 = curTargetVelocity - currentVelocity2;


        // Both wheels close to the target: a short buzz on the controller tells the driver to feed.
        boolean atSpeed = wantSpin && commandedVelocity == curTargetVelocity
                && Math.abs(error) <= FlywheelAuto.READY_TOLERANCE
                && Math.abs(error2) <= FlywheelAuto.READY_TOLERANCE;
        if (atSpeed && !wasAtSpeed && now - lastRumbleNanos > 500_000_000L) {
            gamepad1.rumble(200);
            lastRumbleNanos = now;
        }
        wasAtSpeed = atSpeed;

        telemetry.addData("Flywheels", atSpeed ? "AT SPEED - feed now" : wantSpin ? "spinning up..."
                : wantReverse ? "reversing (left trigger)" : "off - hold RIGHT BUMPER to spin");
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
