package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.FeedPulser;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;
import org.firstinspires.ftc.teamcode.Shared.ShooterSettings;

/**
 * Teleop buttons for the shooter flywheels (gamepad 1).
 *
 *   RIGHT BUMPER (hold)   spin the wheels up to the set speed
 *   LEFT TRIGGER          run the wheels backwards, slower than a shot (part of "spit everything out")
 *   BACK                  open / close the tuning menu. While it is closed the D-PAD and X do
 *                         nothing, so the numbers cannot be changed by accident in a match.
 *   with the menu open:
 *   D-PAD left / right    pick a line on the screen: Speed, P, I, F, Feed on, Feed off
 *   D-PAD up / down       change the picked number (hold it down to keep going)
 *   X (hold 1 second)     put every number back to its default
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
    private static final double SPEED_STEP = 25;
    private static final double P_STEP = 0.05;
    private static final double I_STEP = 0.05;
    private static final double F_STEP = 0.01;
    private static final double FEED_ON_STEP = 25;
    private static final double FEED_OFF_STEP = 50;
    // Holding the D-PAD: after this long it starts stepping by itself, this often.
    private static final long REPEAT_AFTER_NANOS = 400_000_000L;
    private static final long REPEAT_EVERY_NANOS = 100_000_000L;
    /** X has to be HELD this long to reset the numbers, so brushing it in a match does nothing. */
    private static final long RESET_HOLD_NANOS = 1_000_000_000L;

    private static final long FEED_DIP_AFTER_NANOS = 400_000_000L;
    private static final double FEED_DIP_WORTH_SAYING = 60;

    /** Wheels still turning faster than this the other way are left to coast before reversing. */
    private static final double COAST_BELOW = 200;
    private static final double TRIGGER_PRESSED = 0.1;

    private static final String[] LINE_NAMES = {"Speed", "P", "I", "F", "Feed on (ms)", "Feed off (ms)"};

    private FlywheelControl flywheels;
    private ShooterSettings settings = new ShooterSettings();
    /** Runs the belt while LEFT BUMPER is held (or in pulses, if Feed off is given a value). */
    private final FeedPulser pulser = new FeedPulser();
    private boolean beltMayRun;

    // The little menu on the screen. It only listens to the D-PAD while it is open (BACK).
    private boolean menuOpen;
    private boolean backWasDown;
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
    // How far the wheels fell during the last feed, so the drivers can see whether the set
    // speed is one the wheels can actually keep while pollen go through.
    private long feedingSinceNanos;
    private double lowestThisFeed = Double.NaN;
    private double lowestLastFeed = Double.NaN;
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
        pulser.setTimes(settings.feedOnMs, settings.feedOffMs);
    }

    /**
     * 1 = the index should run right now, 0 = not. Hold LEFT BUMPER and the belt runs; let go
     * and it stops. (With "Feed off" above 0 on the menu it goes on / off / on instead, one
     * pollen at a time, see FeedPulser.) It never waits for the flywheels: the gamepad buzzes
     * and the screen says READY when the wheels are up, and that is the cue to press it.
     *
     * It is also 0 for the half second after an outtake while the wheels are still turning
     * backwards: a pollen fed into that would be thrown back down the robot.
     */
    public double feedScale() {
        return beltMayRun ? 1.0 : 0.0;
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
        beltMayRun = pulser.update(feeding && !coasting, flywheels, now);

        trackFeedDip(feeding && wantSpin && !coasting, now);

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

        // Say plainly which mode the wheels are in, and what they are really doing next to
        // what was asked: "held at 850, now 846" is working; "held at 850, now 700" means
        // the wheels cannot reach it (battery, friction) and the number should come down.
        boolean fullPower = settings.speed >= FlywheelControl.MAX_TICKS_PER_SECOND;
        telemetry.addLine(fullPower ? String.format("FLYWHEELS: FULL POWER, no speed control (Speed %.0f)", settings.speed)
                : String.format("FLYWHEELS: held at %.0f%s", settings.speed,
                        wantSpin ? String.format(", now %.0f", flywheels.speed()) : ""));

        if (!fullPower && lowestLastFeed < settings.speed - FEED_DIP_WORTH_SAYING) {
            telemetry.addLine(String.format("Last feed the wheels fell to %.0f. For every pollen the same, set Speed to about %.0f.",
                    lowestLastFeed, 25 * Math.floor(lowestLastFeed / 25)));
        }

        if (menuOpen) {
            // The arrow marks the number the D-PAD is changing.
            telemetry.addLine("---- TUNING (BACK = done) ----");
            double[] values = {settings.speed, settings.p, settings.i, settings.f, settings.feedOnMs, settings.feedOffMs};
            for (int line = 0; line < LINE_NAMES.length; line++) {
                boolean wholeNumber = line == 0 || line >= 4;
                String value = wholeNumber ? String.format("%.0f", values[line]) : String.format("%.2f", values[line]);
                telemetry.addLine((line == selectedLine ? ">> " : "    ") + LINE_NAMES[line] + ":  " + value);
            }
            telemetry.addLine(settings.feedOffMs <= 0 ? "Feed off 0 = the belt runs for as long as LEFT BUMPER is held."
                    : "Feed off above 0 = one pollen at a time while LEFT BUMPER is held.");
            telemetry.addLine("D-PAD left/right = pick a line, up/down = change it. Hold X = defaults.");
        } else {
            telemetry.addLine("Shooter numbers: " + settings.describe());
            telemetry.addLine("BACK = change them (the D-PAD does nothing until then).");
        }
        if (now - noteNanos < 3_000_000_000L && !note.isEmpty()) telemetry.addLine(note);

        // The power is here so "slow" can be told apart: a low number with the speed on target
        // is just a low set speed, a high number with a low speed is the wheels being held back.
        double apart = flywheels.wheelsApart();
        telemetry.addData("Flywheel right / left", "%.0f / %.0f  (%.0f apart, power %.0f%%)",
                flywheels.rightVelocity(), flywheels.leftVelocity(), apart, flywheels.power() * 100);
    }

    /** The tuning menu: BACK opens and closes it; open, left / right picks a number, up / down changes it, holding X resets them. */
    private void runMenu(Gamepad gamepad1, long now) {
        boolean back = gamepad1.back;
        if (back && !backWasDown) menuOpen = !menuOpen;
        backWasDown = back;
        if (!menuOpen) {
            // Closed: the D-PAD and X are dead, so a thumb resting on them changes nothing.
            // Anything changed just before closing still gets saved.
            heldDirection = 0;
            leftWasDown = false;
            rightWasDown = false;
            xDownSinceNanos = 0;
            resetDone = false;
            if (unsaved) {
                unsaved = false;
                say(settings.save() ? "saved: " + settings.describe() : "COULD NOT SAVE - these numbers are lost on restart", now);
            }
            return;
        }

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
            else if (selectedLine == 3) settings.f += steps * F_STEP;
            else if (selectedLine == 4) settings.feedOnMs += steps * FEED_ON_STEP;
            else settings.feedOffMs += steps * FEED_OFF_STEP;
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

    /**
     * Remembers the lowest average wheel speed during a feed (after the first moments, once a
     * pollen has had time to arrive). If that is well under the set speed, the wheels cannot
     * keep that speed with the index running, and the first pollen will fly further than the
     * rest. The screen then says what speed they could keep.
     */
    private void trackFeedDip(boolean feedingNow, long now) {
        if (!feedingNow) {
            if (feedingSinceNanos != 0 && !Double.isNaN(lowestThisFeed)) lowestLastFeed = lowestThisFeed;
            feedingSinceNanos = 0;
            lowestThisFeed = Double.NaN;
            return;
        }
        if (feedingSinceNanos == 0) feedingSinceNanos = now;
        if (now - feedingSinceNanos < FEED_DIP_AFTER_NANOS || !flywheels.hasSpeedReading()) return;
        double speed = flywheels.speed();
        if (Double.isNaN(lowestThisFeed) || speed < lowestThisFeed) lowestThisFeed = speed;
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
