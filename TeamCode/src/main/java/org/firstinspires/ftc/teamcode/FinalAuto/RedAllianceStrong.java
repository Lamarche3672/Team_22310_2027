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

@Autonomous(name = "Red Alliance Strong", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class RedAllianceStrong extends AutoBase {

    private static final double FIRST_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double SECOND_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double FLOWER_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;

    private static final double GARDEN_SPEED_INCHES_PER_SECOND = 30;
    private static final double GARDEN_TURN_DONE_AT = 0.4;
    private static final double GARDEN_INTAKE_POWER = IndexIntakeAuto.INTAKE_POWER;
    private static final double GARDEN_INDEX_SPEED = 0.3;
    private static final double GARDEN_DWELL_MS = 400;
    private static final double INTAKE_FINISH_MS = 900;
    private static final double CROSS_TURN_DONE_AT = 0.65;

    private static final boolean FLOWER_CYCLE = false; // cant pick up flowers yet
    private static final double FLOWER_SPEED_INCHES_PER_SECOND = 25;
    private static final double FLOWER_INTAKE_MS = 1500;
    private static final double FLOWER_TIMEOUT_MS = 3000;
    private static final double FLOWER_LATEST_START_SECONDS = 16;

    private static final double GARDEN_TIMEOUT_MS = 4500;
    private static final double CROSS_TIMEOUT_MS = 6000;
    private static final double SHOOT_SPOT_REACH_INCHES = 6;

    private final PoseFactory poseFactory = alliancePoseFactory();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose point12CollectLeftGarden = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose point33GoforSecondshot = poseFactory.of(42.6823, 124.02, 100);
    private final Pose flowerPickup = poseFactory.of(45.4524, 133.9783, 90);
    private final Pose parking = poseFactory.of(12.0, 107.982, 0);

    @Override
    protected String autoName() {
        return (isBlueAlliance() ? "Blue" : "Red") + " Alliance Strong";
    }

    @Override
    protected Pose startPose() {
        return start;
    }

    @Override
    protected Pose parkPose() {
        return parking;
    }

    @Override
    protected Command scoringRoutine() {
        return sequential(
                step("1  Shoot preloads", sequential(
                        drive.holdCommand(),
                        Volley.shoot(flywheel, intake, FIRST_SHOT_VELOCITY))),

                step("2  Start -> garden, intake on", sequential(
                        intake.intakeCommand(GARDEN_INTAKE_POWER, GARDEN_INDEX_SPEED),
                        drive.followSlowly(path12CollectLeftGarden(), GARDEN_SPEED_INCHES_PER_SECOND, GARDEN_TIMEOUT_MS),
                        waitMs(GARDEN_DWELL_MS))),

                step("3  To the other side", sequential(
                        drive.follow(gardenToSecondShot(), CROSS_TIMEOUT_MS).with(sequential(
                                waitMs(INTAKE_FINISH_MS),
                                intake.stopCommand(),
                                flywheel.spinUpCommand(SECOND_SHOT_VELOCITY))),
                        drive.waitUntilSettled(point33GoforSecondshot, SETTLE_TIMEOUT_MS))),

                step("4  Shoot garden pollen", shootIfAtSecondShotSpot(SECOND_SHOT_VELOCITY)),

                conditional(
                        () -> FLOWER_CYCLE && autoTimer.seconds() <= FLOWER_LATEST_START_SECONDS
                                && drive.isNear(point33GoforSecondshot, SHOOT_SPOT_REACH_INCHES),
                        flowerCycle(),
                        instant(() -> { })));
    }

    private Command flowerCycle() {
        return sequential(
                step("Flower: collect 4 more", sequential(
                        intake.intakeCommand(GARDEN_INTAKE_POWER, GARDEN_INDEX_SPEED),
                        drive.followSlowly(toFlower(), FLOWER_SPEED_INCHES_PER_SECOND, FLOWER_TIMEOUT_MS),
                        waitMs(FLOWER_INTAKE_MS),
                        intake.stopCommand())),

                step("Flower: back to the shooting spot", sequential(
                        flywheel.spinUpCommand(FLOWER_SHOT_VELOCITY),
                        drive.follow(fromFlower(), FLOWER_TIMEOUT_MS),
                        drive.waitUntilSettled(point33GoforSecondshot, SETTLE_TIMEOUT_MS))),

                step("Flower: shoot", shootIfAtSecondShotSpot(FLOWER_SHOT_VELOCITY)));
    }

    private Command shootIfAtSecondShotSpot(double velocity) {
        return conditional(
                () -> drive.isNear(point33GoforSecondshot, SHOOT_SPOT_REACH_INCHES)
                        && drive.isFacing(point33GoforSecondshot, SHOOT_AIM_DEGREES),
                Volley.shoot(flywheel, intake, velocity),
                flywheel.stopCommand());
    }

    public Path path12CollectLeftGarden() {
        return Paths.line(start, point12CollectLeftGarden)
                .linear(start, point12CollectLeftGarden, GARDEN_TURN_DONE_AT);
    }

    public Path gardenToSecondShot() {
        return Paths.line(point12CollectLeftGarden, point33GoforSecondshot)
                .linear(point12CollectLeftGarden, point33GoforSecondshot, CROSS_TURN_DONE_AT);
    }

    public Path toFlower() {
        return Paths.line(point33GoforSecondshot, flowerPickup).linear(point33GoforSecondshot, flowerPickup);
    }

    public Path fromFlower() {
        return Paths.line(flowerPickup, point33GoforSecondshot).linear(flowerPickup, point33GoforSecondshot);
    }
}
