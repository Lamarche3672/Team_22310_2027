package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Shared.Battery;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;
import org.firstinspires.ftc.teamcode.Shared.ShotTable;

import java.util.ArrayList;
import java.util.List;

/**
 * Teleop buttons for the shooter flywheels (gamepad 1).
 *
 *   RIGHT BUMPER (hold)  spin up to the right speed for this distance
 *   LEFT TRIGGER         run the wheels backwards, slower than a shot (part of "spit everything out")
 *   Y                    backup speed high / low, used when the camera cannot see the hive
 *   BACK                 open / close shot setup
 *     in shot setup:  D-PAD up/down = speed,  A = save for this distance,  X = delete nearest
 *                     (with no hive in view, A saves the speed as the selected Y preset)
 *
 * Where the speed comes from, best first:
 *   1. the saved shots (ShotTable) and the hive distance from the camera
 *   2. the last distance speed, for a few seconds after the camera loses the hive
 *   3. the Y preset
 *
 * The wheels themselves are run by Shared/FlywheelControl, the same code autonomous uses.
 */
public class Shooter {
    // The Y presets, used when there is no saved shot for the distance.
    // HIGH is the fastest speed the wheels can HOLD while pollen go through, not the fastest
    // they can spin. At 2600 the motors were already at full power just to stay there, so the
    // moment the index started (the battery dips a little) or a pollen went through, the
    // wheels slowed and there was nothing left to push back with. Up to about 2400 a tenth
    // of the power is kept in hand, and that is what keeps the 2nd, 3rd and 4th pollen at the
    // same speed as the 1st. LOW is for shots from right next to the hive.
    // These are only the starting numbers. The drive team can change either one from the
    // gamepad in shot setup (BACK), and what they save there is what gets used from then on.
    public double highVelocity = 2300;
    public double lowVelocity = 1500;
    /**
     * LEFT TRIGGER runs the wheels backwards to clear a jam. Slower than shooting on purpose:
     * it only has to push a pollen back out, not throw it across the robot.
     */
    public static final double REVERSE_VELOCITY = 1600;

    private static final double SPEED_STEP = 25;
    private static final double SLOWEST_SHOT = 400;
    private static final double FASTEST_SHOT = FlywheelControl.MAX_TICKS_PER_SECOND;
    /** Ignore distance changes worth less than this, so the wheels are not always hunting. */
    private static final double SPEED_CHANGE = 15;
    private static final double KEEP_DISTANCE_SPEED_SECONDS = 4.0;
    /** Wheels still turning faster than this the other way are left to coast before reversing. */
    private static final double COAST_BELOW = 200;
    private static final double TRIGGER_PRESSED = 0.1;

    private FlywheelControl flywheels;
    private List<double[]> shotPoints = new ArrayList<>();
    private boolean highSelected = true;
    private double hiveDistance = Double.NaN;
    private double shotSpeed = highVelocity;
    private boolean usingDistance;
    private long lastDistanceSpeedNanos;

    private boolean setupOpen;
    private boolean feeding;
    private boolean coasting;
    /** +1 or -1: the direction the wheels were last driven in. 0 = not driven yet. */
    private double lastDriveSign;
    private boolean wasReady;
    private long lastRumbleNanos;
    private String saveMessage = "";
    private long saveMessageNanos;
    private long drivenSinceNanos;

    public void init(HardwareMap hardwareMap, Battery battery) {
        flywheels = new FlywheelControl(hardwareMap, battery);
        shotPoints = ShotTable.load();
        double[] savedPresets = ShotTable.loadPresets();
        if (savedPresets != null) {
            highVelocity = Math.max(SLOWEST_SHOT, Math.min(FASTEST_SHOT, savedPresets[0]));
            lowVelocity = Math.max(SLOWEST_SHOT, Math.min(FASTEST_SHOT, savedPresets[1]));
        }
        shotSpeed = highVelocity;
        stop();
    }

    public void stop() {
        if (flywheels != null) flywheels.stop();
    }

    /** The camera's distance to the hive in inches, or NaN when it cannot see it. Set every loop. */
    public void setHiveDistance(double inches) {
        hiveDistance = inches;
    }

    public double shotSpeed() {
        return shotSpeed;
    }

    /**
     * 1 = the index may run, 0 = not yet. In teleop the driver is in charge: LEFT BUMPER
     * always runs the index, at once. (It used to wait for the flywheels, and on the real
     * robot that wait left the index standing still.) The gamepad buzzes and the screen says
     * READY when the wheels are up, and that is the cue to press it.
     *
     * The one exception is the half second after an outtake while the wheels are still
     * turning backwards: a pollen fed into that would be thrown back down the robot.
     */
    public double feedScale() {
        return coasting ? 0.0 : 1.0;
    }

    public void loop(Telemetry telemetry, Gamepad gamepad1) {
        long now = System.nanoTime();

        if (gamepad1.yWasPressed()) {
            highSelected = !highSelected;
        }
        if (gamepad1.backWasPressed()) {
            setupOpen = !setupOpen;
        }

        // Read every "was pressed" each loop even when we ignore it. They remember a press
        // until they are read, so a D-PAD tap during the match would otherwise fire the moment
        // shot setup is opened.
        boolean upPressed = gamepad1.dpadUpWasPressed();
        boolean downPressed = gamepad1.dpadDownWasPressed();
        // START + A is how gamepad 1 is paired. That must not count as "save".
        boolean savePressed = gamepad1.aWasPressed() && !gamepad1.start;
        boolean deletePressed = gamepad1.xWasPressed();
        feeding = gamepad1.left_bumper;

        if (setupOpen) {
            runShotSetup(upPressed, downPressed, savePressed, deletePressed, now);
        } else {
            chooseSpeed(feeding, now);
        }

        // LEFT TRIGGER (spit everything out) beats RIGHT BUMPER, the same as it does for the
        // intake and the index.
        boolean wantReverse = gamepad1.left_trigger > TRIGGER_PRESSED;
        boolean wantSpin = gamepad1.right_bumper && !wantReverse;
        double wantedVelocity = wantSpin ? shotSpeed : wantReverse ? -REVERSE_VELOCITY : 0;

        // Going straight from forwards to backwards (or back) would make the motors fight the
        // spinning wheels: a big current spike for nothing. Let them coast down first.
        // "Still going the other way" only counts if WE drove them the other way last. Going
        // by the reading alone was a trap: an encoder that counts backwards looked like wheels
        // spinning the wrong way, and the wheels were then never driven at all.
        double measured = flywheels.speed();
        coasting = wantedVelocity != 0 && lastDriveSign != 0 && Math.signum(wantedVelocity) != lastDriveSign
                && Math.abs(measured) > COAST_BELOW;
        if (coasting) wantedVelocity = 0;
        else if (wantedVelocity != 0) lastDriveSign = Math.signum(wantedVelocity);

        if (wantedVelocity == 0) {
            if (flywheels.target() != 0) flywheels.stop();
        } else {
            flywheels.setTarget(wantedVelocity);
        }
        // Feeding for real: bumper held, wheels going forwards, and the index allowed to move.
        flywheels.setFeeding(feeding && wantSpin && !coasting);
        flywheels.update();

        boolean ready = wantSpin && !coasting && flywheels.isReady();
        if (ready && !wasReady && now - lastRumbleNanos > 500_000_000L) {
            gamepad1.rumble(200);
            lastRumbleNanos = now;
        }
        wasReady = ready;

        // A wheel that reads zero while we have been driving it for a second means a loose
        // encoder cable or a jam. Worth a line on the screen: the shot will be off.
        if (wantSpin && !coasting && flywheels.power() > 0.2) {
            if (drivenSinceNanos == 0) drivenSinceNanos = now;
        } else {
            drivenSinceNanos = 0;
        }
        boolean noReading = drivenSinceNanos != 0 && now - drivenSinceNanos > 1_000_000_000L
                && flywheels.encoderLooksDead();

        telemetry.addData("SHOOTER", ready ? "READY - shoot (LEFT BUMPER)" : coasting ? "slowing down first..."
                : wantSpin ? "spinning up..."
                : wantReverse ? "reversing (left trigger)" : "off - hold RIGHT BUMPER to spin");
        String encoderNote = flywheels.encoderNote();
        if (encoderNote != null) telemetry.addLine(encoderNote);
        else if (noReading) telemetry.addLine("A FLYWHEEL READS 0: check its encoder cable or a jam");
        telemetry.addData("Shot speed", "%.0f  %s", shotSpeed, speedSource());
        // The power is here so "slow" can be told apart: a low number with the speed on target
        // is just a low shot speed, a high number with a low speed is the wheels being held back.
        telemetry.addData("Flywheel right / left", "%.0f / %.0f  (power %.0f%%)",
                flywheels.rightVelocity(), flywheels.leftVelocity(), flywheels.power() * 100);

        if (setupOpen) {
            telemetry.addLine("---- SHOT SETUP (BACK = done) ----");
            telemetry.addData("Hive distance", Double.isFinite(hiveDistance)
                    ? String.format("%.0f in", hiveDistance) : "camera can't see the hive");
            telemetry.addLine("D-PAD up/down = speed.  A = save for this distance.  X = delete nearest.");
            telemetry.addLine("Camera can't see the hive? A saves it as the Y preset instead.");
            if (now - saveMessageNanos < 3_000_000_000L && !saveMessage.isEmpty()) telemetry.addLine(saveMessage);
            telemetry.addData("Saved shots", ShotTable.describe(shotPoints));
        }
    }

    /** Shot setup: the driver sets the speed by hand and saves it against the current distance. */
    private void runShotSetup(boolean up, boolean down, boolean save, boolean delete, long now) {
        usingDistance = false;
        if (up) shotSpeed += SPEED_STEP;
        if (down) shotSpeed -= SPEED_STEP;
        shotSpeed = Math.max(SLOWEST_SHOT, Math.min(FASTEST_SHOT, shotSpeed));

        if (save) {
            if (!Double.isFinite(hiveDistance)) {
                // No distance to save it against, so it becomes the Y preset that is selected.
                if (highSelected) highVelocity = shotSpeed;
                else lowVelocity = shotSpeed;
                saveMessage = ShotTable.savePresets(highVelocity, lowVelocity)
                        ? String.format("SAVED %.0f as the %s preset (camera can't see the hive)", shotSpeed, highSelected ? "HIGH" : "LOW")
                        : "could not save";
            } else if (!ShotTable.add(shotPoints, hiveDistance, shotSpeed)) {
                saveMessage = "NOT saved: too many shots saved, delete one with X";
            } else {
                saveMessage = ShotTable.save(shotPoints)
                        ? String.format("SAVED %.0f at %.0f in", shotSpeed, hiveDistance) : "could not save";
            }
            saveMessageNanos = now;
        }
        if (delete) {
            boolean removed = ShotTable.removeNear(shotPoints, hiveDistance);
            if (removed) ShotTable.save(shotPoints);
            saveMessage = removed ? "deleted the saved shot nearest this distance" : "nothing saved near this distance";
            saveMessageNanos = now;
        }
    }

    /** Normal play: pick the speed from the hive distance if we can, the Y preset if we cannot. */
    private void chooseSpeed(boolean feeding, long now) {
        double fromDistance = ShotTable.speedFor(shotPoints, hiveDistance);
        if (Double.isFinite(fromDistance)) {
            fromDistance = Math.max(SLOWEST_SHOT, Math.min(FASTEST_SHOT, fromDistance));
            // While pollen are going through, hold the speed. The distance reading jitters a
            // little and a volley should all be shot at one speed.
            if (!usingDistance || (!feeding && Math.abs(fromDistance - shotSpeed) >= SPEED_CHANGE)) {
                shotSpeed = fromDistance;
            }
            usingDistance = true;
            lastDistanceSpeedNanos = now;
        } else if (!usingDistance || now - lastDistanceSpeedNanos > KEEP_DISTANCE_SPEED_SECONDS * 1e9) {
            // The tag blinks out now and then. Only fall back to the preset once it has been
            // gone for a while, so the wheels do not jump speed mid-shot.
            usingDistance = false;
            shotSpeed = highSelected ? highVelocity : lowVelocity;
        }
    }

    private String speedSource() {
        if (setupOpen) return "(shot setup)";
        if (usingDistance) {
            return Double.isFinite(hiveDistance)
                    ? String.format("AUTO, hive %.0f in", hiveDistance) : "AUTO, last distance";
        }
        String preset = highSelected ? "HIGH" : "LOW";
        if (shotPoints.isEmpty()) return preset + " (Y switches)";
        return preset + " (Y switches) - no hive in view";
    }
}
