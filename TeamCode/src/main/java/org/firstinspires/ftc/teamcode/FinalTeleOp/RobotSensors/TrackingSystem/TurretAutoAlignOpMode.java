package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors.TrackingSystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret April Tag Align", group = "TeamCode")
public class TurretAutoAlignOpMode extends OpMode {
    private static final int LIMELIGHT_PIPELINE = 8;
    /** The Limelight keeps handing back its last result forever, so anything older than this is ignored. */
    private static final long MAX_RESULT_AGE_MS = 250;

    private Limelight3A limelight;
    private final TurretTracking turret = new TurretTracking();

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        turret.init(hardwareMap);
    }

    @Override
    public void start() {
        turret.resetTimer();
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() <= MAX_RESULT_AGE_MS) {
            double bearing = result.getTx();
            telemetry.addData("Limelight tx (deg)", bearing);
            turret.update(bearing);
        } else {
            telemetry.addData("Limelight", "No valid target");
            turret.update(Double.NaN);
        }
    }

    @Override
    public void stop() {
        turret.stop();
        if (limelight != null) limelight.stop();
    }
}
