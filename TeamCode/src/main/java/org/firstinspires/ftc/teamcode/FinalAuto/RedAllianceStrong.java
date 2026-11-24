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

// The autonomous mechanisms. Each is the auto version of the teleop class it is named after.
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;      // shooter wheels
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.IndexIntakeAuto;   // intake + index
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.Volley;            // one volley of 4

/**
 * RED ALLIANCE STRONG  (BlueAllianceStrong runs this same routine with the field turned around)
 *
 *   1. START            hold position, shoot the 4 preloaded pollen     (flywheel + index)
 *   2. START -> GARDEN  turn the intake toward the garden, then sweep along the wall with the
 *                       intake running                                  (intake + slow index)
 *   3. -> OTHER SIDE    one smooth drive: turns gently on the way, the intake finishes,
 *                       then the flywheel spins up                      (intake, then flywheel)
 *   4. OTHER SIDE       stop, then shoot the garden pollen              (flywheel + index)
 *   5. PARK             in the loading zone, clear of the wall          (done by AutoBase)
 *
 *   OPTIONAL, switched off (FLOWER_CYCLE): between 4 and 5, drive 10 inches to the flower on the
 *   rear wall, take the 4 pollen stored in it, and shoot a third volley. Turn it on once the
 *   robot can pull pollen out of a flower.
 *
 * Why the order matters: a HIVE TIP is worth 20 points, LEAVE 3 and PARK 5. Only the cell that
 * faces up counts, and a tip turns the other cell up, so the volleys come from both sides of the hive.
 *
 * If something goes wrong (blocked by a wall or a robot, a flywheel that never reaches speed)
 * every step has a time limit, a blocked drive is given up after 0.6 s, and at the park deadline
 * the robot drops everything and parks. The loop, the parking and the clean-up are in AutoBase.
 */
@Autonomous(name = "Red Alliance Strong", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class RedAllianceStrong extends AutoBase {

    // ------------------------------------------------------------------ numbers to tune

    /** Flywheel speed for each volley, in encoder ticks per second. The shots are from different distances. */
    private static final double FIRST_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double SECOND_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;
    private static final double FLOWER_SHOT_VELOCITY = FlywheelAuto.HIGH_VELOCITY;

    /**
     * Start -> garden: the speed limit while the robot intakes. Pedro turns a speed limit into a
     * fixed motor power (about 0.009 x the number), it does not measure the speed. 20 came out as
     * 18 % power, which is about where a mecanum robot sticks and slips, so the robot crept and
     * stuttered toward the garden. 30 is about 28 % power. Go lower only if it still moves cleanly.
     */
    private static final double GARDEN_SPEED_INCHES_PER_SECOND = 30;
    /**
     * Start -> garden: the robot has turned its intake toward the garden this far along the way
     * (0.4 = after the first 40 %), so it drives the pollen line nose first instead of sideways.
     */
    private static final double GARDEN_TURN_DONE_AT = 0.4;
    /** Start -> garden: intake motor power (1.0 = same as holding the right trigger). */
    private static final double GARDEN_INTAKE_POWER = IndexIntakeAuto.INTAKE_POWER;
    /**
     * Start -> garden: the index belt creeps inwards at this fraction of its shooting speed,
     * moving each pollen further in so there is room for the next. 0 = belt off while intaking.
     * Too high and the first pollen gets pushed into the flywheels, which are not spinning.
     */
    private static final double GARDEN_INDEX_SPEED = 0.3;
    /** Stay in the garden corner this long with the intake on, to get the pollen right in the corner. */
    private static final double GARDEN_DWELL_MS = 400;
    /** Keep intaking this long while leaving the garden so the last pollen is fully inside. */
    private static final double INTAKE_FINISH_MS = 900;
    /**
     * Garden -> other side: the robot turns to its shooting heading while it drives and has
     * finished this far along the way. The turn is spread over most of the drive on purpose: Pedro
     * gives turning first call on the motors, so a 100 degree turn squeezed into the first third
     * left nothing for driving and the robot surged. (This also replaces the old 6-inch "turning
     * point" path, where the robot had to stop, spin on the spot and start again.)
     */
    private static final double CROSS_TURN_DONE_AT = 0.65;

    /**
     * Third volley: take the 4 pollen stored in the flower next to the second-shot spot.
     * OFF because the robot cannot pull pollen out of a flower yet. Set to true when it can.
     */
    private static final boolean FLOWER_CYCLE = false;
    private static final double FLOWER_SPEED_INCHES_PER_SECOND = 25;
    /** How long the intake works at the flower before the robot backs away. */
    private static final double FLOWER_INTAKE_MS = 1500;
    private static final double FLOWER_TIMEOUT_MS = 3000;
    /** The flower trip takes about 9 s. Later than this it is skipped so there is time to park. */
    private static final double FLOWER_LATEST_START_SECONDS = 16;

    /** Time limits per drive, so a blocked robot moves on to the next step. */
    private static final double GARDEN_TIMEOUT_MS = 4500;
    private static final double CROSS_TIMEOUT_MS = 6000;
    /** A volley only fires if the robot is within this distance of its shooting spot. */
    private static final double SHOOT_SPOT_REACH_INCHES = 6;

    // ------------------------------------------------------------------ poses (written for RED)

    private final PoseFactory poseFactory = alliancePoseFactory();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose point12CollectLeftGarden = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose point33GoforSecondshot = poseFactory.of(42.6823, 124.02, 100);
    /** The flower on the rear wall: the same spot as pickUpRefresh in RedAllianceWeak. */
    private final Pose flowerPickup = poseFactory.of(45.4524, 133.9783, 90);
    /**
     * x was 7.1435, closer to the wall than half the robot, so the robot finished pressed against
     * the wall. LEAVE (3 points) needs the robot NOT touching the perimeter wall when AUTO ends.
     * 12 leaves about 3.5 inches of air, enough for the robot to roll a little past the spot, and
     * is still inside the loading zone for PARK.
     */
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

    // ------------------------------------------------------------------ the routine

    @Override
    protected Command scoringRoutine() {
        return sequential(
                // 1. BEGINNING: shoot the 4 preloads from the start position, holding still.
                step("1  Shoot preloads", sequential(
                        drive.holdCommand(),
                        Volley.shoot(flywheel, intake, FIRST_SHOT_VELOCITY))),

                // 2. START -> GARDEN: intake on, index belt creeping inwards to make room, at a
                //    steady reduced speed so the pollen is not knocked away.
                step("2  Start -> garden, intake on", sequential(
                        intake.intakeCommand(GARDEN_INTAKE_POWER, GARDEN_INDEX_SPEED),
                        drive.followSlowly(path12CollectLeftGarden(), GARDEN_SPEED_INCHES_PER_SECOND, GARDEN_TIMEOUT_MS),
                        waitMs(GARDEN_DWELL_MS))),

                // 3. TO THE OTHER SIDE in one smooth drive. While it drives: the intake finishes pulling
                //    the last pollen in, switches off, and only then the flywheels spin up, so they are
                //    ready on arrival and nothing is fired on the way.
                step("3  To the other side", sequential(
                        drive.follow(gardenToSecondShot(), CROSS_TIMEOUT_MS).with(sequential(
                                waitMs(INTAKE_FINISH_MS),
                                intake.stopCommand(),
                                flywheel.spinUpCommand(SECOND_SHOT_VELOCITY))),
                        drive.waitUntilSettled(point33GoforSecondshot, SETTLE_TIMEOUT_MS))),

                // 4. OTHER SIDE: shoot what was collected in the garden.
                //    Only if the robot really got there; a volley from the wrong place scores nothing.
                step("4  Shoot garden pollen", shootIfAtSecondShotSpot(SECOND_SHOT_VELOCITY)),

                // OPTIONAL FLOWER TRIP: 4 more pollen from the flower beside the shooting spot, then a
                // third volley. Skipped when it is switched off, the robot is not at the spot, or time is short.
                conditional(
                        () -> FLOWER_CYCLE && autoTimer.seconds() <= FLOWER_LATEST_START_SECONDS
                                && drive.isNear(point33GoforSecondshot, SHOOT_SPOT_REACH_INCHES),
                        flowerCycle(),
                        instant(() -> { })));
    }

    private Command flowerCycle() {
        return sequential(
                // Into the flower slowly with the intake on, and give it time to pull the pollen out.
                step("Flower: collect 4 more", sequential(
                        intake.intakeCommand(GARDEN_INTAKE_POWER, GARDEN_INDEX_SPEED),
                        drive.followSlowly(toFlower(), FLOWER_SPEED_INCHES_PER_SECOND, FLOWER_TIMEOUT_MS),
                        waitMs(FLOWER_INTAKE_MS),
                        // Intake and index off BEFORE the flywheels start, so nothing is fired on the way back.
                        intake.stopCommand())),

                // Back to the shooting spot with the flywheels spinning up.
                step("Flower: back to the shooting spot", sequential(
                        flywheel.spinUpCommand(FLOWER_SHOT_VELOCITY),
                        drive.follow(fromFlower(), FLOWER_TIMEOUT_MS),
                        drive.waitUntilSettled(point33GoforSecondshot, SETTLE_TIMEOUT_MS))),

                // Third volley.
                step("Flower: shoot", shootIfAtSecondShotSpot(FLOWER_SHOT_VELOCITY)));
    }

    private Command shootIfAtSecondShotSpot(double velocity) {
        return conditional(
                () -> drive.isNear(point33GoforSecondshot, SHOOT_SPOT_REACH_INCHES),
                Volley.shoot(flywheel, intake, velocity),
                flywheel.stopCommand());
    }

    // ------------------------------------------------------------------ paths

    public Path path12CollectLeftGarden() {
        return Paths.line(start, point12CollectLeftGarden)
                .linear(start, point12CollectLeftGarden, GARDEN_TURN_DONE_AT);
    }

    /**
     * Garden corner straight to the second-shot spot. The old turning point (13.28, 15.37) was
     * only an inch off this line; the turn it was there for now happens while driving.
     */
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
