package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;

/**
 * One volley in autonomous. Every auto shoots through here, so they all shoot the same way
 * and a fix here fixes all of them.
 *
 *   1. start the flywheels (so they spin up while the robot aims)
 *   2. turn to face the hive using the camera, if it can see it
 *   3. wait until the wheels are ready, and have had MIN_SPIN_UP_MS to get there
 *   4. feed the pollen through for FEED_MS
 *   5. everything off
 *
 * Nothing in here can hang: every wait has a time limit, so a dead camera, a dead flywheel or
 * a jam only costs a second or two and the auto carries on.
 */
public final class Volley {

    /** Longest the robot spends turning to the hive before it shoots as it is. */
    public static final double AIM_TIMEOUT_MS = 1200;
    /** Longest wait for the flywheels before shooting anyway. */
    public static final double SPIN_UP_TIMEOUT_MS = 2500;
    /**
     * The wheels always get at least this long to spin up before the first pollen, whatever
     * "ready" says. On this robot the flywheel encoders read nothing, so "ready" is only a
     * guess by the clock, and at 1.5 s the wheels were not up yet: the first auto shot went
     * nowhere. A driver waits about this long in teleop without thinking about it.
     */
    public static final double MIN_SPIN_UP_MS = 2500;
    /**
     * How long a volley feeds for, pauses between pollen included. 2.5 s was not enough on
     * the real robot: the pollen had not reached the wheels when the belt stopped. In teleop
     * the driver just holds the bumper until they are gone; this is the auto's version of that.
     * At the default 150 ms on / 450 ms off it is seven or eight pollen's worth for four.
     */
    public static final double FEED_MS = 4500;
    /** Holding the belt for the wheels before the first pollen may stretch a volley by at most this much. */
    public static final double MAX_EXTRA_FEED_MS = 1500;

    private Volley() { }

    public static Command shoot(FlywheelAuto flywheel, IndexIntakeAuto intake, MecanumDriveAuto drive,
                                HiveAim aim, double velocity) {
        // The speed does not depend on the camera: it is the number passed in (HIGH_VELOCITY
        // means the team's set speed). The camera is only used to turn toward the hive.
        return sequential(
                flywheel.spinUpCommand(velocity),
                drive.aimAtHive(aim, AIM_TIMEOUT_MS),
                flywheel.waitUntilReady(SPIN_UP_TIMEOUT_MS),
                flywheel.waitUntilSpunFor(MIN_SPIN_UP_MS),
                feed(flywheel, intake),
                flywheel.stopCommand());
    }

    /**
     * Feeds for FEED_MS, one pollen at a time: the belt goes on / off / on the same way it
     * does in teleop (FeedPulser), and the pauses count as part of the feed. Before the first
     * pollen it may hold the belt a moment for the wheels (FlywheelControl.feedScale()); that
     * wait does not count, up to MAX_EXTRA_FEED_MS.
     */
    public static Command feed(FlywheelAuto flywheel, IndexIntakeAuto intake) {
        long[] lastNanos = new long[1];
        double[] feedingMs = new double[1];
        double[] totalMs = new double[1];
        return new CommandBuilder()
                .setStart(() -> {
                    lastNanos[0] = System.nanoTime();
                    feedingMs[0] = 0;
                    totalMs[0] = 0;
                    flywheel.setFeeding(true);
                })
                .setExecute(() -> {
                    long now = System.nanoTime();
                    double elapsedMs = (now - lastNanos[0]) / 1e6;
                    lastNanos[0] = now;
                    boolean started = flywheel.feedScale() >= 1.0;
                    boolean beltOn = flywheel.beltShouldRun(started);
                    intake.feedShooter(beltOn ? 1.0 : 0.0);
                    if (started) feedingMs[0] += elapsedMs;
                    totalMs[0] += elapsedMs;
                })
                .setDone(() -> feedingMs[0] >= FEED_MS || totalMs[0] >= FEED_MS + MAX_EXTRA_FEED_MS)
                // Also runs when the park deadline cuts a volley short, so the feed never stays on.
                .setEnd(reason -> {
                    intake.stop();
                    flywheel.setFeeding(false);
                    flywheel.beltShouldRun(false);
                });
    }
}
