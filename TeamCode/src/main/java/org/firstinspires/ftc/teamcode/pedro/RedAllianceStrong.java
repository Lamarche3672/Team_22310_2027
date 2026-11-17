package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.PedroTuning.Constants;

@Autonomous(name = "Red Alliance Strong", group = "Autonomous")
public class RedAllianceStrong extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 270);
    private final Pose point12CollectLeftGarden = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose turningPointStart = poseFactory.of(10.7481, 10.2282, 200);
    private final Pose turningPoint = poseFactory.of(15.6752, 42.082, 100);
    private final Pose point33GoforSecondshotStart = poseFactory.of(15.6752, 42.082, 100);
    private final Pose point33GoforSecondshot = poseFactory.of(43.0508, 118.3084, 100);
    private final Pose point44CollectflowersStart = poseFactory.of(43.0508, 118.3084, 100);
    private final Pose point44Collectflowers = poseFactory.of(42.4066, 132.2775, 45);
    private final Pose goforthirdshotStart = poseFactory.of(42.4066, 132.2775, 45);
    private final Pose goforthirdshot = poseFactory.of(56.42, 118.68, 90);
    private final Pose parkingStart = poseFactory.of(56.42, 118.68, 90);
    private final Pose parking = poseFactory.of(7.1435, 107.982, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path12CollectLeftGarden()),
                follow(follower, turningPoint()),
                follow(follower, path33GoforSecondshot()),
                follow(follower, path44Collectflowers()),
                follow(follower, goforthirdshot()),
                follow(follower, parking())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path12CollectLeftGarden() {
        return Paths.line(start, point12CollectLeftGarden).linear(start, point12CollectLeftGarden);
    }

    public Path turningPoint() {
        return Paths.line(turningPointStart, turningPoint).linear(turningPointStart, turningPoint);
    }

    public Path path33GoforSecondshot() {
        return Paths.line(point33GoforSecondshotStart, point33GoforSecondshot).linear(point33GoforSecondshotStart, point33GoforSecondshot);
    }

    public Path path44Collectflowers() {
        return Paths.line(point44CollectflowersStart, point44Collectflowers).linear(point44CollectflowersStart, point44Collectflowers);
    }

    public Path goforthirdshot() {
        return Paths.line(goforthirdshotStart, goforthirdshot).linear(goforthirdshotStart, goforthirdshot);
    }

    public Path parking() {
        return Paths.line(parkingStart, parking).linear(parkingStart, parking);
    }
}
