package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import org.firstinspires.ftc.teamcode.Shared.HiveAim;
import org.firstinspires.ftc.teamcode.Shared.ShotTable;

import java.util.List;

/**
 * One volley in autonomous. Every auto shoots through here, so they all shoot the same way
 * and a fix here fixes all of them.
 *
 *   1. start the flywheels (so they spin up while the robot aims)
 *   2. turn to face the hive using the camera, if it can see it
 *   3. set the final speed from the hive distance, if a shot is saved for it
 *   4. wait until the wheels are ready
 *   5. feed the pollen through, letting the wheels recover between each one
 *   6. everything off
 *
 * Nothing in here can hang: every wait has a time limit, so a dead camera, a dead flywheel or
 * a jam only costs a second or two and the auto carries on.
 */
public final class Volley {

    /** Longest the robot spends turning to the hive before it shoots as it is. */
    public static final double AIM_TIMEOUT_MS = 1200;
    /** Longest wait for the flywheels before shooting anyway. */
    public static final double SPIN_UP_TIMEOUT_MS = 2500;
    /** How long the index has to actually run to push 4 pollen through. */
    public static final double FEED_MS = 2500;
    /** Waiting for the wheels between pollen may stretch a volley by at most this much. */
    public static final double MAX_EXTRA_FEED_MS = 1500;

    private Volley() { }

    public static Command shoot(FlywheelAuto flywheel, IndexIntakeAuto intake, MecanumDriveAuto drive,
                                HiveAim aim, List<double[]> shotPoints, double fallbackVelocity) {
        return sequential(
                flywheel.spinUpCommand(fallbackVelocity),
                drive.aimAtHive(aim, AIM_TIMEOUT_MS),
                instant(() -> flywheel.spinUp(speedFor(aim, shotPoints, fallbackVelocity))),
                flywheel.waitUntilReady(SPIN_UP_TIMEOUT_MS),
                feed(flywheel, intake),
                flywheel.stopCommand());
    }

    /** The saved speed for the distance the camera measures, or the fallback when there is none. */
    static double speedFor(HiveAim aim, List<double[]> shotPoints, double fallbackVelocity) {
        if (aim == null || shotPoints == null) return fallbackVelocity;
        double saved = ShotTable.speedFor(shotPoints, aim.distanceInches());
        return Double.isFinite(saved) ? saved : fallbackVelocity;
    }

    /**
     * Runs the index into the flywheels until it has fed for FEED_MS. How hard it feeds comes
     * from the flywheels every loop: full when they are at speed, easing off while they catch
     * up after a pollen. Time spent waiting does not count as feeding, up to a limit.
     */
    public static Command feed(FlywheelAuto flywheel, IndexIntakeAuto intake) {
        long[] lastNanos = new long[1];
        double[] fedMs = new double[1];
        double[] totalMs = new double[1];
        return new CommandBuilder()
                .setStart(() -> {
                    lastNanos[0] = System.nanoTime();
                    fedMs[0] = 0;
                    totalMs[0] = 0;
                    flywheel.setFeeding(true);
                })
                .setExecute(() -> {
                    long now = System.nanoTime();
                    double elapsedMs = (now - lastNanos[0]) / 1e6;
                    lastNanos[0] = now;
                    double scale = flywheel.feedScale();
                    intake.feedShooter(scale);
                    fedMs[0] += elapsedMs * scale;
                    totalMs[0] += elapsedMs;
                })
                .setDone(() -> fedMs[0] >= FEED_MS || totalMs[0] >= FEED_MS + MAX_EXTRA_FEED_MS)
                // Also runs when the park deadline cuts a volley short, so the feed never stays on.
                .setEnd(reason -> {
                    intake.stop();
                    flywheel.setFeeding(false);
                });
    }
}
