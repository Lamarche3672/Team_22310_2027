package org.firstinspires.ftc.teamcode.FinalAuto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.MecanumDriveAuto;

@Autonomous(name = "Angled Line Test", group = "Autonomous")
public class AngledLineTest extends LinearOpMode {
        private Follower follower;
        private final MecanumDriveAuto drive = new MecanumDriveAuto();

        /** Longest one drive may take. A blocked robot is given up after 0.6 s (see MecanumDriveAuto). */
        private static final double PATH_TIMEOUT_MS = 6000;
        private static final double SETTLE_TIMEOUT_MS = 1500;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(56, 8, 90);
        private final Pose path1 = poseFactory.of(20.625, 8.179, 90);
        private final Pose point2Start = poseFactory.of(20.625, 8.179, 90);
        private final Pose point2 = poseFactory.of(39.6346, 117.1388, 90);
        private final Pose point3Start = poseFactory.of(39.6346, 117.1388, 90);
        private final Pose point3 = poseFactory.of(105.1688, 109.3948, 90);
        private final Pose point4Start = poseFactory.of(105.1688, 109.3948, 90);
        private final Pose point4 = poseFactory.of(132.204, 8.8143, 90);
        private final Pose point5Start = poseFactory.of(132.204, 8.8143, 90);
        private final Pose point5 = poseFactory.of(68.7122, 22.0113, 90);
        private final Pose point6Start = poseFactory.of(68.7122, 22.0113, 90);
        private final Pose point6 = poseFactory.of(71.1347, 121.5759, 90);

        // Autonomous routine
        public Command autoRoutine() {
            return sequential(
                    drive.follow(path1(), PATH_TIMEOUT_MS),
                    drive.follow(path2(), PATH_TIMEOUT_MS),
                    drive.follow(path3(), PATH_TIMEOUT_MS),
                    drive.follow(path4(), PATH_TIMEOUT_MS),
                    drive.follow(path5(), PATH_TIMEOUT_MS),
                    drive.follow(path6(), PATH_TIMEOUT_MS),
                    // Let go of the wheels at the end, so the motors are not left pushing (for example against a wall).
                    drive.waitUntilSettled(point6, SETTLE_TIMEOUT_MS),
                    drive.stopCommand()
            );
        }

        @Override
        public void runOpMode() {
            Scheduler.reset();
            try {
                drive.init(hardwareMap, start);
                follower = drive.follower();

                waitForStart();
                if (!opModeIsActive()) return;
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
            } finally {
                // Runs on a normal finish, on Stop, and on an error: the wheels are always released.
                Scheduler.reset();
                drive.stop();
            }
        }

        public Path path1() {
            return Paths.line(start, path1).linear(start, path1);
        }

        public Path path2() {
            return Paths.line(point2Start, point2).linear(point2Start, point2);
        }

        public Path path3() {
            return Paths.line(point3Start, point3).linear(point3Start, point3);
        }

        public Path path4() {
            return Paths.line(point4Start, point4).linear(point4Start, point4);
        }

        public Path path5() {
            return Paths.line(point5Start, point5).linear(point5Start, point5);
        }

        public Path path6() {
            return Paths.line(point6Start, point6).linear(point6Start, point6);
        }
    }
