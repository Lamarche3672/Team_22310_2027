package org.firstinspires.ftc.teamcode.FinalAuto;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * RED ALLIANCE WEAK: park and nothing else, for when our partner runs the scoring auto.
 *
 * The robot starts in the field corner right beside our parking zone, drives one straight
 * line into the zone and stops. No turning, no shooting, no intake, and it never goes near
 * the hives or the garden, so it cannot get in our partner's way.
 *
 * The drive itself is AutoBase's parking step: it goes in a straight line from wherever the
 * robot is to parkPose(), so this class only has to say where the robot starts and parks.
 *
 * Positions are red coordinates in inches; BlueAllianceWeak reuses this class turned half a
 * turn. The route this auto used to drive (up the middle, with an optional preload volley)
 * is in the git history, last in commit 9b33143.
 */
@Autonomous(name = "RedAllianceWeak", group = "Autonomous", preselectTeleOp = "MAIN TELEOP")
public class RedAllianceWeak extends AutoBase {

    private final PoseFactory poseFactory = alliancePoseFactory();

    /** In the corner, touching both walls: 8 in is from the wall to the middle of the robot. */
    private final Pose start = poseFactory.of(8, 136, 90);

    /**
     * Same heading as the start, so the robot never turns beside the wall.
     *
     * x is how far the middle of the robot ends up from the side wall. 11 is the edge of the
     * zone on the wall side (for blue, looking up the field from the corner, the right-hand
     * edge) with a couple of inches of air left, because touching the wall at the end loses
     * the LEAVE points. A bigger x parks further out into the field.
     */
    private final Pose parkatSpawn = poseFactory.of(11, 106.3905, 90);

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
        // Nothing to score: AutoBase goes straight on to parking.
        return instant(() -> { });
    }
}
