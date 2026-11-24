package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

public final class Volley {

    public static final double SPIN_UP_TIMEOUT_MS = 2500;
    public static final double FEED_MS = 2500;
    public static final double FLYWHEEL_RECOVERY_MS = 600;
    public static final double MAX_PAUSED_MS = 1500;

    private Volley() { }

    public static Command shoot(FlywheelAuto flywheel, IndexIntakeAuto intake, double velocity) {
        return sequential(
                flywheel.spinUpCommand(velocity),
                flywheel.waitUntilReady(SPIN_UP_TIMEOUT_MS),
                feed(flywheel, intake),
                flywheel.stopCommand());
    }

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
                        if (flywheel.isAtSpeed()) pauseMs[0] = 0;
                    }
                })
                .setDone(() -> fedMs[0] >= FEED_MS)
                .setEnd(reason -> intake.stop());
    }
}
