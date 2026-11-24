package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

/*
Limelight results:
Red hive FRONT : 34, 35, 36, 37
Red hive BACK : 30, 31, 32, 33
Blue hive FRONT : 38, 39, 40, 41
Blue hive BACK : 42, 43, 44, 45
 */


public class AprilTagLimelightTest extends OpMode {
    
    
    private static final long MAX_RESULT_AGE_MS = 250;

    public LLResult callId = null;
    
    private Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
    }

    @Override
    public void init() {
        init(hardwareMap);
    }

    @Override
    public void start() {
        limelight.start();
    }

    public void update() {
        LLResult llResult = limelight.getLatestResult();
        boolean fresh = llResult != null && llResult.isValid() && llResult.getStaleness() <= MAX_RESULT_AGE_MS;
        callId = fresh ? llResult : null;
    }

    @Override
    public void loop() {
        update();
        LLResult llResult = callId;
        if (llResult != null) {
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
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
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
