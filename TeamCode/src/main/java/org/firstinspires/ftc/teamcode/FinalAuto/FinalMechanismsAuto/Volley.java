package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;

public final class Volley {

    public static final double SPIN_UP_TIMEOUT_MS = 2500;
    public static final double FEED_MS = 2500;

    private Volley() { }

    public static Command shoot(FlywheelAuto flywheel, IndexIntakeAuto intake, double velocity) {
        return sequential(
                flywheel.spinUpCommand(velocity),
                flywheel.waitUntilReady(SPIN_UP_TIMEOUT_MS),
                feed(flywheel, intake),
                flywheel.stopCommand());
    }

    public static Command feed(FlywheelAuto flywheel, IndexIntakeAuto intake) {
        long[] startNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> {
                    startNanos[0] = System.nanoTime();
                    intake.feedShooter();
                })
                .setExecute(intake::feedShooter)
                .setDone(() -> System.nanoTime() - startNanos[0] >= FEED_MS * 1e6)
                .setEnd(reason -> intake.stop());
    }
}
