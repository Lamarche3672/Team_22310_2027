package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;
import org.firstinspires.ftc.teamcode.Shared.ShooterSettings;

/**
 * Teleop buttons for the shooter flywheels (gamepad 1).
 *
 *   RIGHT BUMPER (hold)   spin the wheels up to the set speed
 *   LEFT TRIGGER          run the wheels backwards, slower than a shot (part of "spit everything out")
 *   D-PAD left / right    pick a line on the screen: Speed, P, I or F
 *   D-PAD up / down       change the picked number (hold it down to keep going)
 *   X (hold 1 second)     put all four numbers back to their defaults
 *
 * There is one speed. The camera does not change it: what the screen says is what the wheels
 * are asked for, and only the D-PAD changes it. The four numbers are saved on the Control Hub
 * as soon as the D-PAD is let go, and autonomous uses the same ones (see ShooterSettings).
 *
 * The wheels themselves are run by Shared/FlywheelControl, the same code autonomous uses.
 */
public class Shooter {
    /**
     * LEFT TRIGGER runs the wheels backwards to clear a jam. Slower than shooting on purpose:
     * it only has to push a pollen back out, not throw it across the robot.
     */
    public static final double REVERSE_VELOCITY = 1600;

    // How much one press of the D-PAD changes each number.
    private static final double SPEED_STEP = 50;
    private static final double P_STEP = 0.05;
    private static final double I_STEP = 0.05;
    private static final double F_STEP = 0.01;
    // Holding the D-PAD: after this long it starts stepping by itself, this often.
    private static final long REPEAT_AFTER_NANOS = 400_000_000L;
    private static final long REPEAT_EVERY_NANOS = 100_000_000L;
    /** X has to be HELD this long to reset the numbers, so brushing it in a match does nothing. */
    private static final long RESET_HOLD_NANOS = 1_000_000_000L;

    /** Wheels still turning faster than this the other way are left to coast before reversing. */
    private static final double COAST_BELOW = 200;
    private static final double TRIGGER_PRESSED = 0.1;

    private static final String[] LINE_NAMES = {"Speed", "P", "I", "F"};

    private FlywheelControl flywheels;
    private ShooterSettings settings = new ShooterSettings();

    // The little menu on the screen.
    private int selectedLine;
    private boolean leftWasDown;
    private boolean rightWasDown;
    private int heldDirection;
    private long heldSinceNanos;
    private long lastStepNanos;
    private boolean unsaved;
    private long xDownSinceNanos;
    private boolean resetDone;
    private String note = "";
    private long noteNanos;

    private boolean coasting;
    /** +1 or -1: the direction the wheels were last driven in. 0 = not driven yet. */
    private double lastDriveSign;
    private boolean wasReady;
    private long lastRumbleNanos;
    private long drivenSinceNanos;

    public void init(HardwareMap hardwareMap, Battery battery) {
        flywheels = new FlywheelControl(hardwareMap, battery);
        settings = ShooterSettings.load();
        applyGains();
        stop();
    }

    public void stop() {
        if (flywheels != null) flywheels.stop();
    }

    public double shotSpeed() {
        return settings.speed;
    }

    /** The numbers this robot will shoot with, for the INIT screen: a wrong one shows before the match. */
    public String describeSpeeds() {
        return "Shooter: " + settings.describe();
    }

    private void applyGains() {
        flywheels.setGains(settings.p, settings.f, settings.i);
    }

    /**
     * 1 = the index may run, 0 = not yet. In teleop the driver is in charge: LEFT BUMPER
     * always runs the index, at once. (It used to wait for the flywheels, and on the real
     * robot that wait left the index standing still.) The gamepad buzzes and the screen says
     * READY when the wheels are up, and that is the cue to press it.
     *
     * The one exception is the half second after an outtake while the wheels are still
     * turning backwards: a pollen fed into that would be thrown back down the robot.
     */
    public double feedScale() {
        return coasting ? 0.0 : 1.0;
    }

    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        long now = System.nanoTime();
        runMenu(gamepad1, now);

        boolean feeding = gamepad1.left_bumper;
        // LEFT TRIGGER (spit everything out) beats RIGHT BUMPER, the same as it does for the
        // intake and the index.
        boolean wantReverse = gamepad1.left_trigger > TRIGGER_PRESSED;
        boolean wantSpin = gamepad1.right_bumper && !wantReverse;
        double wantedVelocity = wantSpin ? settings.speed : wantReverse ? -REVERSE_VELOCITY : 0;

        // Going straight from forwards to backwards (or back) would make the motors fight the
        // spinning wheels: a big current spike for nothing. Let them coast down first.
        // "Still going the other way" only counts if WE drove them the other way last. Going
        // by the reading alone was a trap: an encoder that counts backwards looked like wheels
        // spinning the wrong way, and the wheels were then never driven at all.
        double measured = flywheels.speed();
        coasting = wantedVelocity != 0 && lastDriveSign != 0 && Math.signum(wantedVelocity) != lastDriveSign
                && Math.abs(measured) > COAST_BELOW;
        if (coasting) wantedVelocity = 0;
        else if (wantedVelocity != 0) lastDriveSign = Math.signum(wantedVelocity);

        if (wantedVelocity == 0) {
            if (flywheels.target() != 0) flywheels.stop();
        } else {
            flywheels.setTarget(wantedVelocity);
        }
        // Feeding for real: bumper held, wheels going forwards, and the index allowed to move.
        flywheels.setFeeding(feeding && wantSpin && !coasting);
        flywheels.update();

        boolean ready = wantSpin && !coasting && flywheels.isReady();
        if (ready && !wasReady && now - lastRumbleNanos > 500_000_000L) {
            gamepad1.rumble(200);
            lastRumbleNanos = now;
        }
        wasReady = ready;

        // A wheel that reads zero while we have been driving it for a second means a loose
        // encoder cable or a jam. Worth a line on the screen: the shot will be off.
        if (wantSpin && !coasting && flywheels.power() > 0.2) {
            if (drivenSinceNanos == 0) drivenSinceNanos = now;
        } else {
            drivenSinceNanos = 0;
        }
        boolean noReading = drivenSinceNanos != 0 && now - drivenSinceNanos > 1_000_000_000L
                && flywheels.encoderLooksDead();

        telemetry.addData("SHOOTER", ready ? "READY - shoot (LEFT BUMPER)" : coasting ? "slowing down first..."
                : wantSpin ? "spinning up..."
                : wantReverse ? "reversing (left trigger)" : "off - hold RIGHT BUMPER to spin");
        String encoderNote = flywheels.encoderNote();
        if (encoderNote != null) telemetry.addLine(encoderNote);
        else if (noReading) telemetry.addLine("A FLYWHEEL READS 0: check its encoder cable or a jam");

        // The menu. The arrow marks the number the D-PAD is changing.
        double[] values = {settings.speed, settings.p, settings.i, settings.f};
        for (int line = 0; line < LINE_NAMES.length; line++) {
            String value = line == 0 ? String.format("%.0f", values[line]) : String.format("%.2f", values[line]);
            telemetry.addLine((line == selectedLine ? ">> " : "    ") + LINE_NAMES[line] + ":  " + value);
        }
        telemetry.addLine("D-PAD left/right = pick a line, up/down = change it. Hold X = defaults.");
        if (now - noteNanos < 3_000_000_000L && !note.isEmpty()) telemetry.addLine(note);

        // The power is here so "slow" can be told apart: a low number with the speed on target
        // is just a low set speed, a high number with a low speed is the wheels being held back.
        double apart = flywheels.wheelsApart();
        telemetry.addData("Flywheel right / left", "%.0f / %.0f  (%.0f apart%s, power %.0f%%)",
                flywheels.rightVelocity(), flywheels.leftVelocity(), apart,
                wantSpin && ready && apart > FlywheelControl.MATCHED_WITHIN ? " - TOO FAR APART" : "",
                flywheels.power() * 100);
    }

    /** The D-PAD menu: left / right picks a number, up / down changes it, holding X resets all four. */
    private void runMenu(Gamepad gamepad1, long now) {
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        if (left && !leftWasDown) selectedLine = (selectedLine + LINE_NAMES.length - 1) % LINE_NAMES.length;
        if (right && !rightWasDown) selectedLine = (selectedLine + 1) % LINE_NAMES.length;
        leftWasDown = left;
        rightWasDown = right;

        int steps = stepFromDpad(gamepad1.dpad_up, gamepad1.dpad_down, now);
        if (steps != 0) {
            if (selectedLine == 0) settings.speed += steps * SPEED_STEP;
            else if (selectedLine == 1) settings.p += steps * P_STEP;
            else if (selectedLine == 2) settings.i += steps * I_STEP;
            else settings.f += steps * F_STEP;
            settings.clamp();
            applyGains();
            unsaved = true;
        }
        // Save once the D-PAD is let go, not on every step: holding it would otherwise write
        // the file ten times a second.
        if (unsaved && heldDirection == 0) {
            unsaved = false;
            say(settings.save() ? "saved: " + settings.describe() : "COULD NOT SAVE - these numbers are lost on restart", now);
        }

        // START + X is not a pairing combo, but ignore it anyway so nothing odd happens while pairing.
        boolean x = gamepad1.x && !gamepad1.start;
        if (!x) {
            xDownSinceNanos = 0;
            resetDone = false;
        } else {
            if (xDownSinceNanos == 0) xDownSinceNanos = now;
            if (!resetDone && now - xDownSinceNanos >= RESET_HOLD_NANOS) {
                resetDone = true;
                settings.resetToDefaults();
                applyGains();
                say(settings.save() ? "back to defaults: " + settings.describe() : "defaults set, but could not save", now);
            }
        }
    }

    /** +1, -1 or 0 for this loop: one step on a press, then repeating steps while it is held. */
    private int stepFromDpad(boolean up, boolean down, long now) {
        int direction = up == down ? 0 : (up ? 1 : -1);
        if (direction == 0) {
            heldDirection = 0;
            return 0;
        }
        if (direction != heldDirection) {
            heldDirection = direction;
            heldSinceNanos = now;
            lastStepNanos = now;
            return direction;
        }
        if (now - heldSinceNanos >= REPEAT_AFTER_NANOS && now - lastStepNanos >= REPEAT_EVERY_NANOS) {
            lastStepNanos = now;
            return direction;
        }
        return 0;
    }

    private void say(String text, long now) {
        note = text;
        noteNanos = now;
    }
}
