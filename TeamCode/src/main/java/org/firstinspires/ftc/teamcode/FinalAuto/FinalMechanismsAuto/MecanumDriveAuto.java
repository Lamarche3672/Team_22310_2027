package org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning.Constants;
import org.firstinspires.ftc.teamcode.FinalAuto.pedro.PedroTuning.PinpointGuard;

public class MecanumDriveAuto {

    public static final double SETTLED_DISTANCE_INCHES = 1.5;
    public static final double SETTLED_HEADING_DEGREES = 3.0;
    public static final double SETTLED_SPEED_INCHES_PER_SECOND = 2.0;
    public static final double SETTLED_TURN_DEGREES_PER_SECOND = 10.0;

    public static final double STALLED_SPEED_INCHES_PER_SECOND = 1.0;
    public static final double STALLED_TURN_DEGREES_PER_SECOND = 5.0;
    public static final double STALLED_MS = 600;
    public static final double SLOWEST_RELIABLE_INCHES_PER_SECOND = 24;
    public static final int BLIND_READS = 3;

    private Follower follower;

    public void init(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
    }

    public void update() {
        Localizer localizer = follower.localizer;
        if (localizer instanceof PinpointGuard && ((PinpointGuard) localizer).badStreak() >= BLIND_READS) {
            localizer.update();
            follower.drivetrain.stop(true);
            return;
        }
        follower.update();
    }

    public Pose pose() {
        return follower.pose();
    }

    public Follower follower() {
        return follower;
    }

    public void stop() {
        if (follower == null) return;
        follower.stop();
        follower.drivetrain.stop();
    }

    public Command follow(Path path, double timeoutMs) {
        long[] startedNanos = new long[1];
        long[] lastMovedNanos = new long[1];
        return new CommandBuilder()
                .setStart(() -> {
                    follower.follow(path);
                    startedNanos[0] = System.nanoTime();
                    lastMovedNanos[0] = startedNanos[0];
                })
                .setDone(() -> {
                    long now = System.nanoTime();
                    if (isMoving()) lastMovedNanos[0] = now;
                    return follower.atParametricEnd()
                            || now - startedNanos[0] >= timeoutMs * 1e6
                            || now - lastMovedNanos[0] >= STALLED_MS * 1e6;
                })
                .setEnd(reason -> {
                    if (follower.following() && !follower.atParametricEnd()) {
                        follower.hold(follower.pose());
                    }
                });
    }

    public Command followSlowly(Path path, double inchesPerSecond, double timeoutMs) {
        double limit = Math.max(inchesPerSecond, SLOWEST_RELIABLE_INCHES_PER_SECOND);
        return follow(path.with(Constants.foresightConfig.maxVelocityConstraint.at(limit)), timeoutMs);
    }

    public Command waitUntilSettled(Pose target, double timeoutMs) {
        return waitUntil(() -> isSettledAt(target)).raceWith(waitMs(timeoutMs));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public Command holdCommand() {
        return instant(() -> follower.hold(follower.pose()));
    }

    public boolean isMoving() {
        Velocity velocity = follower.velocity();
        return Math.hypot(velocity.vx, velocity.vy) > STALLED_SPEED_INCHES_PER_SECOND
                || Math.abs(velocity.omega) > Math.toRadians(STALLED_TURN_DEGREES_PER_SECOND);
    }

    public boolean isNear(Pose target, double inches) {
        return follower.pose().distance(target) <= inches;
    }

    public boolean isFacing(Pose target, double degrees) {
        return Math.abs(Angle.normalizeSigned(target.heading() - follower.pose().heading()))
                <= Math.toRadians(degrees);
    }

    public boolean isSettledAt(Pose target) {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        return pose.distance(target) <= SETTLED_DISTANCE_INCHES
                && Math.abs(Angle.normalizeSigned(target.heading() - pose.heading())) <= Math.toRadians(SETTLED_HEADING_DEGREES)
                && Math.hypot(velocity.vx, velocity.vy) <= SETTLED_SPEED_INCHES_PER_SECOND
                && Math.abs(velocity.omega) <= Math.toRadians(SETTLED_TURN_DEGREES_PER_SECOND);
    }

    public void addTelemetry(Telemetry telemetry) {
        Pose pose = follower.pose();
        telemetry.addData("x / y (in)", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        if (follower.currentPath() != null) {
            telemetry.addData("Distance left on path (in)", "%.1f", follower.distanceToEndpoint());
        }
    }
}
