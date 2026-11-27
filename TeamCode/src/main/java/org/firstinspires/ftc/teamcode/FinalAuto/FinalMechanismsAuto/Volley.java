package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;

/**
 * One volley in autonomous. Every auto shoots through here, so they all shoot the same way
 * and a fix here fixes all of them.
 *
 * The timing is fixed and simple on purpose:
 *
 *   1. start the flywheels
 *   2. turn to face the hive using the camera, if it can see it (while the wheels spin up)
 *   3. wait until the wheels have been spinning for SPIN_UP_MS
 *   4. feed in pulses: belt on for FEED_ON_MS, off for FEED_PAUSE_MS, and so on, until the
 *      belt has run for FEED_TOTAL_ON_MS in all. The pauses let each pollen leave on its own
 *      and give the wheels a moment to get their speed back before the next one.
 *   5. everything off
 *
 * It goes by the clock and not by "the wheels say they are ready": on this robot that signal
 * could not be trusted, and a volley that waits on it either fires early or not at all.
 * Nothing in here can hang: every step has a fixed length.
 */
public final class Volley {

    /** Longest the robot spends turning to the hive before it shoots as it is. */
    public static final double AIM_TIMEOUT_MS = 1200;
    /**
     * The wheels spin for at least this long before the first pollen. Wheels that were
     * started earlier (on the drive to the shooting spot) have usually had it already.
     */
    public static final double SPIN_UP_MS = 2500;
    /** The belt runs for this long in all during one volley. */
    public static final double FEED_TOTAL_ON_MS = 4000;
    /** One pulse: about one pollen's worth of belt. */
    public static final double FEED_ON_MS = 1000;
    /** The gap after each pulse. Four pulses and three gaps make a volley 5.5 s long. */
    public static final double FEED_PAUSE_MS = 500;

    private Volley() { }

    public static Command shoot(FlywheelAuto flywheel, IndexIntakeAuto intake, MecanumDriveAuto drive,
                                HiveAim aim, double velocity) {
        // The speed does not depend on the camera: it is the number passed in (HIGH_VELOCITY
        // means the team's set speed). The camera is only used to turn toward the hive.
        return sequential(
                flywheel.spinUpCommand(velocity),
                drive.aimAtHive(aim, AIM_TIMEOUT_MS),
                flywheel.waitUntilSpunFor(SPIN_UP_MS),
                feed(flywheel, intake),
                flywheel.stopCommand());
    }

    /** Step 4 above: belt on, pause, belt on, pause, until it has run for FEED_TOTAL_ON_MS. */
    public static Command feed(FlywheelAuto flywheel, IndexIntakeAuto intake) {
        long[] lastNanos = new long[1];
        long[] phaseStartNanos = new long[1];
        boolean[] beltOn = new boolean[1];
        double[] onMs = new double[1];
        return new CommandBuilder()
                .setStart(() -> {
                    lastNanos[0] = System.nanoTime();
                    phaseStartNanos[0] = lastNanos[0];
                    beltOn[0] = true;
                    onMs[0] = 0;
                    flywheel.setFeeding(true);
                })
                .setExecute(() -> {
                    long now = System.nanoTime();
                    if (beltOn[0]) onMs[0] += (now - lastNanos[0]) / 1e6;
                    lastNanos[0] = now;
                    double inPhaseMs = (now - phaseStartNanos[0]) / 1e6;
                    if (beltOn[0] && inPhaseMs >= FEED_ON_MS) {
                        beltOn[0] = false;
                        phaseStartNanos[0] = now;
                    } else if (!beltOn[0] && inPhaseMs >= FEED_PAUSE_MS) {
                        beltOn[0] = true;
                        phaseStartNanos[0] = now;
                    }
                    intake.feedShooter(beltOn[0] ? 1.0 : 0.0);
                })
                .setDone(() -> onMs[0] >= FEED_TOTAL_ON_MS)
                // Also runs when the park deadline cuts a volley short, so the feed never stays on.
                .setEnd(reason -> {
                    intake.stop();
                    flywheel.setFeeding(false);
                });
    }
}
