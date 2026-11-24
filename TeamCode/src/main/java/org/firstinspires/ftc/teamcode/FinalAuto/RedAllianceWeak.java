package org.firstinspires.ftc.teamcode.FinalAuto;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.IndexIntakeAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.Volley;

@Autonomous(name = "RedAllianceWeak", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class RedAllianceWeak extends AutoBase {

    private static final double SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double SPIN_UP_AFTER_MS = 1200;
    private static final double APPROACH_TIMEOUT_MS = 9000;
    private static final double SHOOT_SPOT_REACH_INCHES = 6;

    private static final boolean FLOWER_PICKUP = false; // cant pick up flowers yet
    private static final double FLOWER_SPEED_INCHES_PER_SECOND = 25;
    private static final double FLOWER_INTAKE_MS = 1500;
    private static final double FLOWER_TIMEOUT_MS = 4000;
    private static final double FLOWER_LATEST_START_SECONDS = 14;

    private final PoseFactory poseFactory = alliancePoseFactory();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose rotateDeadSide2 = poseFactory.of(65.222, 46.3369, 270);
    private final Pose rotateforShooting = poseFactory.of(66.0643, 95.3369, 270);
    private final Pose shootStart = poseFactory.of(66.0643, 95.3369, 270);
    private final Pose shoot = poseFactory.of(56.6, 113.4287, 90);
    private final Pose pickUpRefresh = poseFactory.of(45.4524, 133.9783, 90);
    private final Pose getFrontZoneForShooting = poseFactory.of(57.1054, 124.7381, 90);
    private final Pose parkatSpawn = poseFactory.of(12.0, 106.3905, 90);

    @Override
    protected String autoName() {
        return (isBlueAlliance() ? "Blue" : "Red") + " Alliance Weak";
    }

    @Override
    protected Pose startPose() {
        return start;
    }

    @Override
    protected Pose parkPose() {
        return parkatSpawn;
    }

    @Override
    protected Command scoringRoutine() {
        return sequential(
                step("1  Drive to the shooting spot", sequential(
                        drive.follow(toShootingSpot(), APPROACH_TIMEOUT_MS).with(sequential(
                                waitMs(SPIN_UP_AFTER_MS),
                                flywheel.spinUpCommand(SHOT_VELOCITY))),
                        drive.waitUntilSettled(shoot, SETTLE_TIMEOUT_MS))),

                step("2  Shoot preloads", conditional(
                        () -> drive.isNear(shoot, SHOOT_SPOT_REACH_INCHES),
                        Volley.shoot(flywheel, intake, SHOT_VELOCITY),
                        flywheel.stopCommand())),

                conditional(
                        () -> FLOWER_PICKUP && autoTimer.seconds() <= FLOWER_LATEST_START_SECONDS
                                && drive.isNear(shoot, SHOOT_SPOT_REACH_INCHES),
                        flowerCycle(),
                        instant(() -> { })));
    }

    private Command flowerCycle() {
        return sequential(
                step("Flower: collect 4 more", sequential(
                        intake.intakeCommand(IndexIntakeAuto.INTAKE_POWER, 0.3),
                        drive.followSlowly(pickUpRefresh(), FLOWER_SPEED_INCHES_PER_SECOND, FLOWER_TIMEOUT_MS),
                        waitMs(FLOWER_INTAKE_MS),
                        intake.stopCommand())),
                step("Flower: to the front zone", sequential(
                        flywheel.spinUpCommand(SHOT_VELOCITY),
                        drive.follow(getFrontZoneForShooting(), FLOWER_TIMEOUT_MS),
                        drive.waitUntilSettled(getFrontZoneForShooting, SETTLE_TIMEOUT_MS))),
                step("Flower: shoot", conditional(
                        () -> drive.isNear(getFrontZoneForShooting, SHOOT_SPOT_REACH_INCHES),
                        Volley.shoot(flywheel, intake, SHOT_VELOCITY),
                        flywheel.stopCommand())));
    }

    public Path toShootingSpot() {
        return Paths.path(rotateDeadSide2(), rotateforShooting(), shoot());
    }

    public Path rotateDeadSide2() {
        return Paths.line(start, rotateDeadSide2).linear(start, rotateDeadSide2);
    }

    public Path rotateforShooting() {
        return Paths.line(rotateDeadSide2, rotateforShooting).constant(rotateforShooting);
    }

    public Path shoot() {
        return Paths.line(shootStart, shoot).linear(shootStart, shoot);
    }

    public Path pickUpRefresh() {
        return Paths.line(shoot, pickUpRefresh).constant(pickUpRefresh);
    }

    public Path getFrontZoneForShooting() {
        return Paths.line(pickUpRefresh, getFrontZoneForShooting).constant(getFrontZoneForShooting);
    }
}
