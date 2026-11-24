package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

/**
 * One volley: flywheels up to speed, the pollen fed through, everything off again.
 * Every autonomous shoots through this, so they all shoot the same way and a fix here fixes them all.
 *
 *   Volley.shoot(flywheel, intake, velocity)
 */
public final class Volley {

    /** Longest wait for the flywheels to be ready before shooting anyway. */
    public static final double SPIN_UP_TIMEOUT_MS = 2500;
    /** How long the index pushes pollen into the flywheels for one volley of 4 (pauses come on top). */
    public static final double FEED_MS = 2500;
    /** After a pollen slows the wheels, feeding pauses this long at most before carrying on. */
    public static final double FLYWHEEL_RECOVERY_MS = 600;
    /** All the pauses of one volley together may take this long; after that it feeds straight through. */
    public static final double MAX_PAUSED_MS = 1500;

    private Volley() { }

    public static Command shoot(FlywheelAuto flywheel, IndexIntakeAuto intake, double velocity) {
        return sequential(
                flywheel.spinUpCommand(velocity),
                flywheel.waitUntilReady(SPIN_UP_TIMEOUT_MS),
                feed(flywheel, intake),
                flywheel.stopCommand());
    }

    /**
     * Pushes the pollen through for FEED_MS of feeding.
     *
     * When the volley starts with the wheels properly at their target speed, the index only runs
     * while they are at speed: each pollen slows the wheels, the index pauses until they are back,
     * and every shot gets the same strength. Paused time does not count against FEED_MS.
     *
     * When the wheels are NOT at their target (they levelled off below it, or the wait timed out)
     * there is nothing to wait for: pausing would only make the index stutter, because switching
     * the intake and index on is itself enough to slow wheels that are already flat out. Then it
     * feeds straight through. It also stops pausing once the pauses add up to MAX_PAUSED_MS, so
     * pollen is never left in the robot because of a flywheel problem.
     */
    public static Command feed(FlywheelAuto flywheel, IndexIntakeAuto intake) {
        long[] lastNanos = new long[1];
        double[] fedMs = new double[1];
        double[] pausedMs = new double[1];
        double[] pauseMs = new double[1];
        boolean[] waitForSpeed = new boolean[1];
        return new CommandBuilder()
                .setStart(() -> {
                    lastNanos[0] = System.nanoTime();
                    fedMs[0] = 0;
                    pausedMs[0] = 0;
                    pauseMs[0] = 0;
                    waitForSpeed[0] = flywheel.isAtSpeed();
                })
                .setExecute(() -> {
                    long now = System.nanoTime();
                    double elapsedMs = (now - lastNanos[0]) / 1e6;
                    lastNanos[0] = now;

                    boolean pause = waitForSpeed[0] && !flywheel.isAtSpeed()
                            && pauseMs[0] < FLYWHEEL_RECOVERY_MS && pausedMs[0] < MAX_PAUSED_MS;
                    if (pause) {
                        intake.stop();
                        pauseMs[0] += elapsedMs;
                        pausedMs[0] += elapsedMs;
                    } else {
                        intake.feedShooter();
                        fedMs[0] += elapsedMs;
                        if (flywheel.isAtSpeed()) pauseMs[0] = 0;   // recovered: the next pause starts fresh
                    }
                })
                .setDone(() -> fedMs[0] >= FEED_MS)
                // Also runs when the park deadline cuts a volley short: the feed never stays on.
                .setEnd(reason -> intake.stop());
    }
}
