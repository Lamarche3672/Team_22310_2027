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
 *   2. drive into the garden with the intake on and collect 4 more: the turn toward it is
 *      finished early, the last stretch is a straight line, and the robot lets go of its
 *      wheels when it gets there instead of fidgeting on the spot
 *   3. back straight out, then turn and drive beside the start position, spinning the
 *      flywheels up on the way
 *   4. shoot those 4 from there, into the same cell as the preloads
 *   then AutoBase parks.
 *
 * Both volleys go in from the start side on purpose. Only the cell that is tilted up can be
 * scored in, and at the start of a match that is the one on this side. Shooting the second
 * volley from across the field (how this auto used to run) only scores if the hive has
 * tipped by then, and if it has not, four pollen are wasted. See SECOND_SHOT_FROM_START_SIDE.
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
    /**
     * How hard the index runs while intaking (0 to 1). It is 0: the belt on this robot is
     * stiff enough to need most of the motor's strength just to turn, so the 0.3 it used to
     * get here never moved it. It only sat stalled, pulling amps, through the whole drive.
     * Teleop intakes with the belt off too.
     */
    private static final double GARDEN_INDEX_SPEED = 0.0;
    /** A short pause at the end of the garden for the last pollen to come in. */
    private static final double GARDEN_DWELL_MS = 400;

    // ---- where the second volley is shot from
    /**
     * true:  beside the start position, at the cell that is up when the match starts. Scores
     *        whether or not the hive tips.
     * false: across the field at point33GoforSecondshot, at the other cell. Only scores if the
     *        hive has tipped by the time we get there.
     */
    private static final boolean SECOND_SHOT_FROM_START_SIDE = true;

    // ---- driving from the garden to the second shot
    /** Keep intaking this long after leaving the garden, then switch to spinning up. */
    private static final double INTAKE_FINISH_MS = 900;
    private static final double CROSS_TURN_DONE_AT = 0.65;
    private static final double RETURN_TIMEOUT_MS = 4000;

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
    /**
     * The end of the garden run. It was (10.7481, 10.2282), which puts the robot's front
     * corners touching both walls of the corner. The robot could never quite get there, kept
     * pushing, and pivoted against the walls in among the pollen. 1.5 inches further out on
     * both axes it has room to simply stop. Same heading, so the intake points where it did.
     */
    private final Pose point12CollectLeftGarden = poseFactory.of(12.25, 11.73, 200);
    /**
     * Part way back from the garden, still facing it. The robot backs out in a straight line
     * to here before it starts turning toward the hive, so it does not swing round while the
     * intake is still in the pollen.
     */
    private final Pose clearOfGarden = poseFactory.of(32.0, 11.4, 200);
    private final Pose point33GoforSecondshot = poseFactory.of(42.6823, 124.02, 100);
    /**
     * The start-side spot for the second volley: the start position moved 3 inches off the
     * wall. Driving back to a spot that touches the wall would have the robot leaning on it
     * and never quite "arriving"; 3 inches makes no difference to the shot.
     */
    private final Pose secondShotNearStart = poseFactory.of(56, 11, 270);
    private final Pose secondShot = SECOND_SHOT_FROM_START_SIDE ? secondShotNearStart : point33GoforSecondshot;
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
                        // Arrived: let go of the wheels for the pause. Left "holding" the
                        // spot, the robot keeps nudging and twisting to fix the last half
                        // inch, and that twisting is what scattered the pollen.
                        drive.stopCommand(),
                        waitMs(GARDEN_DWELL_MS))),

                // .with(...) runs alongside the drive: finish intaking, then start the
                // flywheels early so they are ready when we arrive.
                step(SECOND_SHOT_FROM_START_SIDE ? "3  Back to the start side" : "3  To the other side", sequential(
                        drive.follow(gardenToSecondShot(), SECOND_SHOT_FROM_START_SIDE ? RETURN_TIMEOUT_MS : CROSS_TIMEOUT_MS)
                                .with(sequential(
                                        waitMs(INTAKE_FINISH_MS),
                                        intake.stopCommand(),
                                        flywheel.spinUpCommand(SECOND_SHOT_VELOCITY))),
                        drive.waitUntilSettled(secondShot, SETTLE_TIMEOUT_MS))),

                step("4  Shoot garden pollen", shootIfAtSecondShotSpot(SECOND_SHOT_VELOCITY)),

                conditional(
                        // The flower is on the far side, so this only makes sense from the far shooting spot.
                        () -> FLOWER_CYCLE && !SECOND_SHOT_FROM_START_SIDE
                                && autoTimer.seconds() <= FLOWER_LATEST_START_SECONDS
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
                () -> drive.isNear(secondShot, SHOOT_SPOT_REACH_INCHES)
                        && drive.isFacing(secondShot, SHOOT_AIM_DEGREES),
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
        if (!SECOND_SHOT_FROM_START_SIDE) {
            return Paths.line(point12CollectLeftGarden, secondShot)
                    .linear(point12CollectLeftGarden, secondShot, CROSS_TURN_DONE_AT);
        }
        // Two legs joined into one path, so the robot flows through without stopping:
        // straight back out of the garden without turning, then the turn toward the hive
        // on the way to the shooting spot.
        return Paths.path(
                Paths.line(point12CollectLeftGarden, clearOfGarden).constant(clearOfGarden),
                Paths.line(clearOfGarden, secondShot).linear(clearOfGarden, secondShot));
    }

    public Path toFlower() {
        return Paths.line(point33GoforSecondshot, flowerPickup).linear(point33GoforSecondshot, flowerPickup);
    }

    public Path fromFlower() {
        return Paths.line(flowerPickup, point33GoforSecondshot).linear(flowerPickup, point33GoforSecondshot);
    }
}
