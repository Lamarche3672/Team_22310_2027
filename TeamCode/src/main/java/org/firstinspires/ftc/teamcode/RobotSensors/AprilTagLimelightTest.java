package org.firstinspires.ftc.teamcode.RobotSensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagLimelightTest extends OpMode {
    
    
    public LLResult callId = null;
    
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
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
        callId = llResult;
    }
    
    
    public void update() {
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    public LLResultTypes.FiducialResult getTagBySpecificId(int fiducialId) {
        if (callId == null || !callId.isValid()) {
            return null;
        }

        for (LLResultTypes.FiducialResult tag : callId.getFiducialResults()) {
            if (tag.getFiducialId() == fiducialId) {
                return tag;
            }
        }
        return null;
    }
}
