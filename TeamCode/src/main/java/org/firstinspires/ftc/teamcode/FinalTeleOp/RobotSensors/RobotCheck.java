package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;

import java.util.ArrayList;
import java.util.List;

/**
 * One button, about 45 seconds: tests the robot and leaves a report on the Driver Station to
 * photograph. No pollen in the robot, hands clear, then press START. It runs by itself.
 *
 * What it tests, in order:
 *   - both hubs are there, and the battery voltage at rest
 *   - the Pinpoint and the Limelight answer
 *   - the flywheels: each wheel alone both ways, both together as teleop runs them, and both
 *     with the right one reversed
 *   - the intake and the index
 *   - that the four drive motors are in the robot config. They are NOT run: the wheels never
 *     turn in this test, so the robot stays where it is
 *
 * Every motor line shows the encoder speed (raw, sign and all), the amps it drew and the
 * lowest hub voltage while it ran. For the flywheels it shows BOTH wheels' speeds each time:
 * a wheel that turns while it is not powered is being driven by the other one.
 *
 * Why it exists: at full power a healthy flywheel motor reads about 2500 to 2800. Ours read
 * 840 and 1400 with the battery falling 2 V, and guessing at the cause from the code cost
 * days. The numbers this prints say which it is:
 *   one wheel slow even alone            -> that wheel drags (rubbing, tight belt, bearing, motor)
 *   each fine alone, slow together       -> the two wheels work against each other
 *   much faster with the right reversed  -> one motor runs the other way round; one line of code
 */
@TeleOp(name = "TEST: Robot check", group = "Test")
public class RobotCheck extends LinearOpMode {
    private static final double FLYWHEEL_RUN_SECONDS = 3.0;
    /** Readings are averaged over the last part of a run, once the speed has settled. */
    private static final double MEASURE_LAST_SECONDS = 1.0;
    private static final double COAST_SECONDS = 2.5;
    private static final double MECHANISM_RUN_SECONDS = 1.5;
    private static final double HEALTHY_FLYWHEEL_SPEED = 2300;

    // {right power, left power} for each flywheel run, and what to call it on the screen.
    private static final double[][] FLYWHEEL_RUNS = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}};
    private static final String[] FLYWHEEL_NAMES = {
            "R alone fwd", "R alone back", "L alone fwd", "L alone back", "both (teleop)", "both, R reversed"};
    private static final String[] DRIVE_MOTORS = {"fl_motor", "fr_motor", "bl_motor", "br_motor"};

    private DcMotorEx right;
    private DcMotorEx left;

    /** The report so far, one short line each, in the order things were tested. */
    private final List<String> report = new ArrayList<>();
    // What each flywheel run measured: right speed, left speed, right amps, left amps, volts.
    private final double[][] flywheel = new double[FLYWHEEL_RUNS.length][];
    private double restVolts = Double.NaN;

    @Override
    public void runOpMode() {
        telemetry.addLine("ROBOT CHECK - about 45 seconds, runs by itself.");
        telemetry.addLine("NO pollen in the robot. Hands clear of the flywheels, intake and index.");
        telemetry.addLine("The drive wheels do NOT turn: the robot stays where it is.");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        try {
            checkHubsAndBattery();
            checkSensors();
            checkFlywheels();
            checkMechanism("INTAKE", "intake_motor", 0.95);
            // Negative is the direction that carries pollen toward the shooter.
            checkMechanism("INDEX", "index_motor", -1.0);
            checkDriveMotorsAreThere();
            String verdict = verdict();
            if (verdict != null) report.add(verdict);
        } finally {
            stopFlywheels();
        }

        // In the robot log too, in case the photo is blurry.
        for (String line : report) RobotLog.ii("RobotCheck", line);

        // Leave the answer up until somebody presses stop.
        while (opModeIsActive()) {
            show("DONE - photo this screen (scroll down for the rest)");
            sleep(200);
        }
    }

    // ------------------------------------------------------------------ hubs, battery, sensors

    private void checkHubsAndBattery() {
        int hubs = hardwareMap.getAll(LynxModule.class).size();
        StringBuilder volts = new StringBuilder();
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (volts.length() > 0) volts.append(" / ");
            volts.append(String.format("%.1f", sensor.getVoltage()));
        }
        restVolts = lowestVolts();
        report.add("HUBS: " + hubs + " found" + (hubs < 2 ? " - EXPANSION HUB MISSING" : "") + ". Volts at rest: " + volts);
    }

    private void checkSensors() {
        String pinpoint;
        try {
            GoBildaPinpointDriver device = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            device.update();
            pinpoint = String.valueOf(device.getDeviceStatus());
        } catch (RuntimeException problem) {
            pinpoint = "NOT FOUND";
        }

        String camera;
        try {
            Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            // Give it a moment to answer before deciding it is not there.
            long until = System.nanoTime() + 1_500_000_000L;
            while (opModeIsActive() && System.nanoTime() < until && !limelight.isConnected()) {
                show("waiting for the camera...");
                sleep(50);
            }
            if (!limelight.isConnected()) {
                camera = "NOT ANSWERING";
            } else {
                LLResult result = limelight.getLatestResult();
                StringBuilder tags = new StringBuilder();
                if (result != null && result.isValid() && result.getFiducialResults() != null) {
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) tags.append(' ').append(tag.getFiducialId());
                }
                camera = "answering, " + (tags.length() == 0 ? "no tags in view" : "tags" + tags);
            }
            limelight.stop();
        } catch (RuntimeException problem) {
            camera = "NOT FOUND";
        }
        report.add("PINPOINT: " + pinpoint + "   CAMERA: " + camera);
    }

    // ------------------------------------------------------------------ flywheels

    private void checkFlywheels() {
        right = motor("right_flywheel");
        left = motor("left_flywheel");
        if (right == null || left == null) {
            report.add("FLYWHEELS: NOT FOUND in the robot config");
            return;
        }
        for (DcMotorEx wheel : new DcMotorEx[] {right, left}) {
            wheel.setDirection(FlywheelControl.DIRECTION);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        report.add("FLYWHEELS  speed R/L | amps R/L | volts");
        for (int run = 0; run < FLYWHEEL_RUNS.length && opModeIsActive(); run++) {
            double[] r = runFlywheels(run);
            flywheel[run] = r;
            report.add(r == null ? " " + FLYWHEEL_NAMES[run] + ": stopped early"
                    : String.format(" %s: %.0f/%.0f | %.1f/%.1f | %.1f", FLYWHEEL_NAMES[run], r[0], r[1], r[2], r[3], r[4]));
            stopFlywheels();
            // Let the wheels stop, so every run starts from rest.
            long until = System.nanoTime() + (long) (COAST_SECONDS * 1e9);
            while (opModeIsActive() && System.nanoTime() < until) {
                show("flywheels stopping...");
                sleep(50);
            }
        }
    }

    /** One flywheel case at full power: the settled speeds, currents and voltage. */
    private double[] runFlywheels(int run) {
        double[] sum = new double[5];
        int samples = 0;
        long start = System.nanoTime();
        while (opModeIsActive()) {
            double seconds = (System.nanoTime() - start) / 1e9;
            if (seconds >= FLYWHEEL_RUN_SECONDS) break;
            // Up to full power over the first 0.3 s rather than in one step.
            double ramp = Math.min(1.0, seconds / 0.3);
            right.setPower(FLYWHEEL_RUNS[run][0] * ramp);
            left.setPower(FLYWHEEL_RUNS[run][1] * ramp);
            if (seconds >= FLYWHEEL_RUN_SECONDS - MEASURE_LAST_SECONDS) {
                sum[0] += right.getVelocity();
                sum[1] += left.getVelocity();
                sum[2] += right.getCurrent(CurrentUnit.AMPS);
                sum[3] += left.getCurrent(CurrentUnit.AMPS);
                sum[4] += lowestVolts();
                samples++;
            }
            show("flywheels: " + FLYWHEEL_NAMES[run]);
            sleep(20);
        }
        if (samples == 0) return null;
        for (int i = 0; i < sum.length; i++) sum[i] /= samples;
        return sum;
    }

    private void stopFlywheels() {
        if (right != null) right.setPower(0);
        if (left != null) left.setPower(0);
    }

    // ------------------------------------------------------------------ intake, index, drive

    private void checkMechanism(String label, String name, double power) {
        DcMotorEx device = motor(name);
        if (device == null) {
            report.add(label + ": NOT FOUND in the robot config");
            return;
        }
        double[] r = runOne(device, power, MECHANISM_RUN_SECONDS, label);
        report.add(r == null ? label + ": stopped early"
                : String.format("%s: encoder %.0f | %.1f A | %.1f V", label, r[0], r[1], r[2]));
    }

    /**
     * Only looks the drive motors up; it never powers them. The robot has no room to move
     * during this test, and driving is the one thing on the robot that already works.
     */
    private void checkDriveMotorsAreThere() {
        StringBuilder missing = new StringBuilder();
        for (String name : DRIVE_MOTORS) {
            try {
                hardwareMap.get(DcMotorEx.class, name);
            } catch (RuntimeException notThere) {
                missing.append(' ').append(name);
            }
        }
        report.add(missing.length() == 0 ? "DRIVE: all 4 motors found (not run, wheels stay still)"
                : "DRIVE: MISSING from the robot config:" + missing);
    }

    /** Runs one motor for a moment and returns {encoder speed, amps, lowest volts}, averaged over the second half. */
    private double[] runOne(DcMotorEx device, double power, double runSeconds, String label) {
        double[] sum = new double[3];
        int samples = 0;
        long start = System.nanoTime();
        try {
            while (opModeIsActive()) {
                double seconds = (System.nanoTime() - start) / 1e9;
                if (seconds >= runSeconds) break;
                device.setPower(power * Math.min(1.0, seconds / 0.2));
                if (seconds >= runSeconds / 2) {
                    sum[0] += device.getVelocity();
                    sum[1] += device.getCurrent(CurrentUnit.AMPS);
                    sum[2] += lowestVolts();
                    samples++;
                }
                show("running: " + label);
                sleep(20);
            }
        } finally {
            device.setPower(0);
        }
        if (samples == 0) return null;
        for (int i = 0; i < sum.length; i++) sum[i] /= samples;
        return sum;
    }

    // ------------------------------------------------------------------ helpers

    /** The motor set up to run on plain power, or null if the config has no such name. */
    private DcMotorEx motor(String name) {
        try {
            DcMotorEx device = hardwareMap.get(DcMotorEx.class, name);
            device.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            return device;
        } catch (RuntimeException notThere) {
            return null;
        }
    }

    /** The lowest hub voltage: the hub that sags most is the one feeding what is running. */
    private double lowestVolts() {
        double lowest = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (volts > 1 && (Double.isNaN(lowest) || volts < lowest)) lowest = volts;
        }
        return lowest;
    }

    private void show(String status) {
        telemetry.addLine(status);
        for (String line : report) telemetry.addLine(line);
        telemetry.update();
    }

    /** What the flywheel numbers mean, in words. Null if the flywheels could not be run. */
    private String verdict() {
        for (double[] r : flywheel) if (r == null) return null;
        StringBuilder text = new StringBuilder("---- WHAT IT MEANS ----");
        double rightAlone = Math.max(Math.abs(flywheel[0][0]), Math.abs(flywheel[1][0]));
        double leftAlone = Math.max(Math.abs(flywheel[2][1]), Math.abs(flywheel[3][1]));
        double together = Math.abs(flywheel[4][0]) + Math.abs(flywheel[4][1]);
        double rightReversed = Math.abs(flywheel[5][0]) + Math.abs(flywheel[5][1]);

        // A wheel that turns while only the other one is powered is linked to it.
        double leftPulledAlong = Math.max(Math.abs(flywheel[0][1]), Math.abs(flywheel[1][1]));
        double rightPulledAlong = Math.max(Math.abs(flywheel[2][0]), Math.abs(flywheel[3][0]));
        if (leftPulledAlong > 200 || rightPulledAlong > 200) {
            text.append("\nThe wheels are LINKED: powering one turns the other.");
        }
        if (rightAlone < 50 && leftAlone < 50) {
            text.append("\nNeither flywheel encoder reads anything: check both encoder cables.");
            return text.toString();
        }
        if (rightAlone < HEALTHY_FLYWHEEL_SPEED) {
            text.append(String.format("\nRIGHT wheel is slow on its own (%.0f, healthy is %.0f+): it is dragging.", rightAlone, HEALTHY_FLYWHEEL_SPEED));
        }
        if (leftAlone < HEALTHY_FLYWHEEL_SPEED) {
            text.append(String.format("\nLEFT wheel is slow on its own (%.0f, healthy is %.0f+): it is dragging.", leftAlone, HEALTHY_FLYWHEEL_SPEED));
        }
        if (rightReversed > 1.3 * together) {
            text.append("\nMUCH FASTER with the right wheel reversed: the two are fighting in teleop.");
        } else if (rightAlone >= HEALTHY_FLYWHEEL_SPEED && leftAlone >= HEALTHY_FLYWHEEL_SPEED
                && together < 0.8 * (rightAlone + leftAlone)) {
            text.append("\nEach wheel is fine alone but slow together: they rub or work against each other.");
        } else if (rightAlone >= HEALTHY_FLYWHEEL_SPEED && leftAlone >= HEALTHY_FLYWHEEL_SPEED) {
            text.append("\nBoth wheels reach a healthy speed. The shooter itself is fine.");
        }
        double sag = restVolts - flywheel[4][4];
        if (sag > 1.2) {
            text.append(String.format("\nBattery falls %.1f V with both flywheels on: too much drag, or a weak battery.", sag));
        }
        return text.toString();
    }
}
