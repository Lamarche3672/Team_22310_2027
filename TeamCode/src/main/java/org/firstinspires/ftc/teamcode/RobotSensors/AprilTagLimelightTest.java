package org.firstinspires.ftc.teamcode.RobotSensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "AprilTag Limelight Test", group = "TeamCode")
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // AprilTag pipeline
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            List<LLResultTypes.FiducialResult> tags = llResult.getFiducialResults();
            if (tags.isEmpty()) {
                telemetry.addData("AprilTag", "No tag found");
            } else {
                // Show the ID and left/right angle for each detected tag.
                for (LLResultTypes.FiducialResult tag : tags) {
                    telemetry.addData("AprilTag", "ID %d | Tx %.1f deg",
                            tag.getFiducialId(), tag.getTargetXDegrees());
                }
            }
        } else {
            telemetry.addData("AprilTag", "No tag found");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
