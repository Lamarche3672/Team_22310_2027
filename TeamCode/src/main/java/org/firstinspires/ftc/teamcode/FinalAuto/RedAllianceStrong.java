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

/**
 * RED ALLIANCE STRONG: the 8-pollen auto.
 *
 *   1. shoot the 4 preloads from the start position
 *   2. drive into the garden with the intake on and collect 4 more
 *   3. drive across to the second shooting spot, spinning the flywheels up on the way
 *   4. shoot those 4
 *   then AutoBase parks.
 *
 * Every shot is aimed with the camera when it can see the hive (see Volley). The headings
 * below only have to be close; they are what is used when the camera sees nothing.
 *
 * All positions are red-alliance coordinates in inches. BlueAllianceStrong reuses this class
 * with everything turned half a turn, so do not write blue numbers in here.
 */
@Autonomous(name = "Red Alliance Strong", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class RedAllianceStrong extends AutoBase {

    // HIGH_VELOCITY = the team's set speed, the one dialled in with the D-PAD in teleop.
    // Separate names so a spot can be given its own number here if it ever needs one.
    private static final double FIRST_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double SECOND_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double FLOWER_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;

    // ---- garden run
    /** Slow enough to pick pollen up instead of pushing them along. 20 was too slow to move reliably. */
    private static final double GARDEN_SPEED_INCHES_PER_SECOND = 30;
    /** The turn toward the garden is finished 40% of the way along, so the intake arrives facing it. */
    private static final double GARDEN_TURN_DONE_AT = 0.4;
    /** The index creeps while intaking so each pollen makes room for the next. */
    private static final double GARDEN_INDEX_SPEED = 0.3;
    /** A short pause at the end of the garden for the last pollen to come in. */
    private static final double GARDEN_DWELL_MS = 400;

    // ---- crossing to the second shot
    /** Keep intaking this long after leaving the garden, then switch to spinning up. */
    private static final double INTAKE_FINISH_MS = 900;
    private static final double CROSS_TURN_DONE_AT = 0.65;

    // ---- optional third cycle from a flower. Off: the robot cannot take pollen out of flowers yet.
    private static final boolean FLOWER_CYCLE = false;
    private static final double FLOWER_SPEED_INCHES_PER_SECOND = 25;
    private static final double FLOWER_INTAKE_MS = 1500;
    private static final double FLOWER_TIMEOUT_MS = 3000;
    /** Starting the flower trip later than this would not leave time to shoot and park. */
    private static final double FLOWER_LATEST_START_SECONDS = 16;

    // ---- safety limits: a drive that takes longer than this is abandoned
    private static final double GARDEN_TIMEOUT_MS = 4500;
    private static final double CROSS_TIMEOUT_MS = 6000;
    /** Only shoot if we actually got this close to the shooting spot. */
    private static final double SHOOT_SPOT_REACH_INCHES = 6;

    private final PoseFactory poseFactory = alliancePoseFactory();

    // x, y in inches, heading in degrees. The shooter is on the BACK of the robot, so a
    // shooting heading points the back at the hive.
    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose point12CollectLeftGarden = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose point33GoforSecondshot = poseFactory.of(42.6823, 124.02, 100);
    private final Pose flowerPickup = poseFactory.of(45.4524, 133.9783, 90);
    /** x = 12 keeps the robot clear of the wall: touching it at the end loses the LEAVE points. */
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
                        volley(FIRST_SHOT_VELOCITY))),

                step("2  Start -> garden, intake on", sequential(
                        intake.intakeCommand(GARDEN_INDEX_SPEED),
                        drive.followSlowly(path12CollectLeftGarden(), GARDEN_SPEED_INCHES_PER_SECOND, GARDEN_TIMEOUT_MS),
                        waitMs(GARDEN_DWELL_MS))),

                // .with(...) runs alongside the drive: finish intaking, then start the
                // flywheels early so they are ready when we arrive.
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
                        intake.intakeCommand(GARDEN_INDEX_SPEED),
                        drive.followSlowly(toFlower(), FLOWER_SPEED_INCHES_PER_SECOND, FLOWER_TIMEOUT_MS),
                        waitMs(FLOWER_INTAKE_MS),
                        intake.stopCommand())),

                step("Flower: back to the shooting spot", sequential(
                        flywheel.spinUpCommand(FLOWER_SHOT_VELOCITY),
                        drive.follow(fromFlower(), FLOWER_TIMEOUT_MS),
                        drive.waitUntilSettled(point33GoforSecondshot, SETTLE_TIMEOUT_MS))),

                step("Flower: shoot", shootIfAtSecondShotSpot(FLOWER_SHOT_VELOCITY)));
    }

    /**
     * Shoots only if the robot really reached the spot and roughly faces the right way.
     * If the crossing was blocked, firing 4 pollen from the wrong place helps nobody.
     */
    private Command shootIfAtSecondShotSpot(double velocity) {
        return conditional(
                () -> drive.isNear(point33GoforSecondshot, SHOOT_SPOT_REACH_INCHES)
                        && drive.isFacing(point33GoforSecondshot, SHOOT_AIM_DEGREES),
                volley(velocity),
                flywheel.stopCommand());
    }

    // ---- paths. ".linear(a, b, t)" turns the robot from a's heading to b's while it
    // drives, finishing the turn at fraction t of the way.

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
