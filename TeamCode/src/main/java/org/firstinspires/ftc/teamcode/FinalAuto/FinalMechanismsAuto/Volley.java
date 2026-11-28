package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;

/**
 * One volley in autonomous. Every auto shoots through here, so they all shoot the same way
 * and a fix here fixes all of them.
 *
 *   1. start the flywheels
 *   2. turn to face the hive using the camera, if it can see it (while the wheels spin up)
 *   3. wait until the wheels have been spinning for SPIN_UP_MS
 *   4. feed one pollen at a time: run the belt for FEED_ON_MS, then stop it and wait for the
 *      wheels to be back up to speed, then the next. A pollen takes speed out of the wheels,
 *      and one that arrives before they have it back lands short.
 *   5. everything off
 *
 * Nothing in here can hang: every wait has a longest time.
 */
public final class Volley {

    /** Longest the robot spends turning to the hive before it shoots as it is. */
    public static final double AIM_TIMEOUT_MS = 1200;
    /**
     * The wheels spin for at least this long before the first pollen. Wheels that were
     * started earlier (on the drive to the shooting spot) have usually had it already.
     */
    public static final double SPIN_UP_MS = 2500;
    /** One pulse of the belt: "a bit", about one pollen's worth. */
    public static final double FEED_ON_MS = 500;
    // The wait after each pulse ends as soon as the wheels are back at speed, but is never
    // shorter than MIN (the pollen needs a moment to actually leave) or longer than MAX
    // (wheels that cannot get back must not stall the volley).
    public static final double FEED_MIN_WAIT_MS = 300;
    public static final double FEED_MAX_WAIT_MS = 800;
    /** The belt runs for this long in all during one volley: eight pulses. */
    public static final double FEED_TOTAL_ON_MS = 4000;
    /** However long the waits turn out, a volley's feed is over by now. */
    public static final double FEED_LONGEST_MS = 8000;

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

    /** Step 4 above: belt on a bit, wait for the wheels, belt on a bit, wait, and so on. */
    public static Command feed(FlywheelAuto flywheel, IndexIntakeAuto intake) {
        long[] startNanos = new long[1];
        long[] lastNanos = new long[1];
        long[] phaseStartNanos = new long[1];
        boolean[] beltOn = new boolean[1];
        double[] onMs = new double[1];
        return new CommandBuilder()
                .setStart(() -> {
                    startNanos[0] = System.nanoTime();
                    lastNanos[0] = startNanos[0];
                    phaseStartNanos[0] = startNanos[0];
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
                    } else if (!beltOn[0] && (inPhaseMs >= FEED_MAX_WAIT_MS
                            || (inPhaseMs >= FEED_MIN_WAIT_MS && flywheel.isBackUpToSpeed()))) {
                        beltOn[0] = true;
                        phaseStartNanos[0] = now;
                    }
                    intake.feedShooter(beltOn[0] ? 1.0 : 0.0);
                })
                .setDone(() -> onMs[0] >= FEED_TOTAL_ON_MS
                        || (System.nanoTime() - startNanos[0]) / 1e6 >= FEED_LONGEST_MS)
                // Also runs when the park deadline cuts a volley short, so the feed never stays on.
                .setEnd(reason -> {
                    intake.stop();
                    flywheel.setFeeding(false);
                });
    }
}
