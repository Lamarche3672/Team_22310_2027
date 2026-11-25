package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelAuto;
import org.firstinspires.ftc.teamcode.FinalAuto.FinalMechanismsAuto.FlywheelControl;

import java.util.ArrayList;
import java.util.List;

public class FlywheelTunerPIDF {

    public FlywheelControl flywheels;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    private static final double SPEED_STEP = 25;
    private static final double SLOWEST_SHOT = 400;
    private static final double FASTEST_SHOT = FlywheelControl.HOLDABLE_TICKS_PER_SECOND;
    private static final double SPEED_CHANGE = 15;
    private static final double KEEP_AUTO_SPEED_SECONDS = 4.0;
    private static final double COAST_BELOW = 200;

    private List<double[]> shotPoints = new ArrayList<>();
    private double hiveDistance = Double.NaN;
    private double shotSpeed = highVelocity;
    private boolean usingDistance;
    private long lastDistanceSpeedNanos;

    private boolean tuningUnlocked;
    private boolean wasAtSpeed;
    private long lastRumbleNanos;
    private String saveMessage = "";
    private long saveMessageNanos;
    private long fullSpeedSinceNanos;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        flywheels = new FlywheelControl(hardwareMap, DcMotorSimple.Direction.FORWARD);
        double[] gains = FlywheelAuto.chooseGains();
        flywheels.setGains(gains[0], gains[1], gains[2]);
        shotPoints = ShotTable.load();
        stop();

        telemetry.addLine("Init Complete");
    }

    public void stop() {
        if (flywheels != null) flywheels.stop();
    }

    public void setHiveDistance(double inches) {
        hiveDistance = inches;
    }

    public double shotSpeed() {
        return shotSpeed;
    }

    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        long now = System.nanoTime();

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        if (gamepad1.backWasPressed()) {
            tuningUnlocked = !tuningUnlocked;
        }

        boolean upPressed = gamepad1.dpadUpWasPressed();
        boolean downPressed = gamepad1.dpadDownWasPressed();
        boolean savePressed = gamepad1.aWasPressed() && !gamepad1.start;
        boolean deletePressed = gamepad1.xWasPressed();
        boolean feeding = gamepad1.left_bumper;

        if (tuningUnlocked) {
            usingDistance = false;
            if (upPressed) shotSpeed += SPEED_STEP;
            if (downPressed) shotSpeed -= SPEED_STEP;
            shotSpeed = Math.max(SLOWEST_SHOT, Math.min(FASTEST_SHOT, shotSpeed));

            if (savePressed) {
                if (!Double.isFinite(hiveDistance)) {
                    saveMessage = "NOT saved: the camera has to see the hive";
                } else if (!ShotTable.add(shotPoints, hiveDistance, shotSpeed)) {
                    saveMessage = "NOT saved: too many shots saved, delete one with X";
                } else {
                    saveMessage = ShotTable.save(shotPoints)
                            ? String.format("SAVED %.0f at %.0f in", shotSpeed, hiveDistance) : "could not save";
                }
                saveMessageNanos = now;
            }
            if (deletePressed) {
                boolean removed = ShotTable.removeNear(shotPoints, hiveDistance);
                if (removed) ShotTable.save(shotPoints);
                saveMessage = removed ? "deleted the saved shot nearest this distance" : "nothing saved near this distance";
                saveMessageNanos = now;
            }
        } else {
            double fromDistance = ShotTable.speedFor(shotPoints, hiveDistance);
            if (Double.isFinite(fromDistance)) {
                fromDistance = Math.max(SLOWEST_SHOT, Math.min(FASTEST_SHOT, fromDistance));
                if (!usingDistance || (!feeding && Math.abs(fromDistance - shotSpeed) >= SPEED_CHANGE)) {
                    shotSpeed = fromDistance;
                }
                usingDistance = true;
                lastDistanceSpeedNanos = now;
            } else if (!usingDistance || now - lastDistanceSpeedNanos > KEEP_AUTO_SPEED_SECONDS * 1e9) {
                usingDistance = false;
                shotSpeed = curTargetVelocity;
            }
        }

        boolean wantSpin = gamepad1.right_bumper;
        boolean wantReverse = !wantSpin && gamepad1.left_trigger > 0.1;
        double wantedVelocity = wantSpin ? shotSpeed : wantReverse ? -lowVelocity : 0;

        double measured = flywheels.speed();
        boolean coasting = (wantedVelocity < 0 && measured > COAST_BELOW)
                || (wantedVelocity > 0 && measured < -COAST_BELOW);
        if (coasting) wantedVelocity = 0;

        if (wantedVelocity != flywheels.target()) {
            if (wantedVelocity == 0) flywheels.stop();
            else flywheels.setTarget(wantedVelocity);
        }
        flywheels.update();

        double currentVelocity = flywheels.rightVelocity();
        double currentVelocity2 = flywheels.leftVelocity();
        double error = shotSpeed - currentVelocity;
        double error2 = shotSpeed - currentVelocity2;

        boolean atSpeed = wantSpin && !coasting
                && Math.abs(shotSpeed - flywheels.speed()) <= FlywheelAuto.READY_TOLERANCE;
        if (atSpeed && !wasAtSpeed && now - lastRumbleNanos > 500_000_000L) {
            gamepad1.rumble(200);
            lastRumbleNanos = now;
        }
        wasAtSpeed = atSpeed;

        if (wantSpin && !coasting && flywheels.power() > 0.2) {
            if (fullSpeedSinceNanos == 0) fullSpeedSinceNanos = now;
        } else {
            fullSpeedSinceNanos = 0;
        }
        boolean noReading = fullSpeedSinceNanos != 0 && now - fullSpeedSinceNanos > 1_000_000_000L
                && (Math.abs(currentVelocity) < 50 || Math.abs(currentVelocity2) < 50);

        telemetry.addData("SHOOTER", atSpeed ? "AT SPEED - feed now (LEFT BUMPER)" : coasting ? "slowing down first..."
                : wantSpin ? "spinning up..."
                : wantReverse ? "reversing (left trigger)" : "off - hold RIGHT BUMPER to spin");
        if (noReading) telemetry.addLine("A FLYWHEEL READS 0: check its encoder cable or a jam");
        telemetry.addData("Shot speed", "%.0f  %s", shotSpeed, speedSource());
        telemetry.addData("Flywheel right / left", "%.0f / %.0f", currentVelocity, currentVelocity2);
        telemetry.addData("Error right / left", "%.0f / %.0f", error, error2);

        if (tuningUnlocked) {
            telemetry.addLine("---- SHOT SETUP (BACK = done) ----");
            telemetry.addData("Hive distance", Double.isFinite(hiveDistance)
                    ? String.format("%.0f in", hiveDistance) : "camera can't see the hive");
            telemetry.addLine("D-PAD up/down = speed.  A = save for this distance.  X = delete nearest.");
            if (now - saveMessageNanos < 3_000_000_000L && !saveMessage.isEmpty()) telemetry.addLine(saveMessage);
            telemetry.addData("Saved shots", ShotTable.describe(shotPoints));
        }
    }

    private String speedSource() {
        if (tuningUnlocked) return "(shot setup)";
        if (usingDistance) {
            return Double.isFinite(hiveDistance)
                    ? String.format("AUTO, hive %.0f in", hiveDistance) : "AUTO, last distance";
        }
        String preset = curTargetVelocity == highVelocity ? "HIGH" : "LOW";
        if (shotPoints.isEmpty()) return preset + " (Y switches)";
        return preset + " (Y switches) - no hive in view";
    }
}
