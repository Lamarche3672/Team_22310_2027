package org.firstinspires.ftc.teamcode.FinalTeleOp.RobotSensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Shared.FlywheelControl;

/**
 * A 35 second test that finds out why the flywheels are slow. Press START and keep hands
 * clear: it runs by itself and leaves the results on the screen. No pollen in the robot.
 *
 * Why it exists: at full power a healthy flywheel motor reads about 2500 to 2800. Ours read
 * 840 and 1400, and the battery fell 2 V from the flywheels alone. Something is taking most
 * of the power, and the cure depends on what:
 *
 *   - one wheel slow even when it runs on its own      -> that wheel is dragging (rubbing,
 *                                                         a tight belt, a bad bearing or motor)
 *   - each fine on its own, slow when both run          -> the two wheels are working against
 *                                                         each other
 *   - both much faster with one of them reversed        -> one motor is wired the other way
 *                                                         round, and one line of code fixes it
 *
 * So it runs each wheel alone, both ways, then both together the way teleop does, then both
 * with the right one reversed. For each it shows the speed of BOTH wheels (a wheel that turns
 * when it is not powered is being driven by the other one), the amps each motor draws, and
 * the battery voltage. The numbers are the raw encoder readings, sign and all.
 */
@TeleOp(name = "TEST: Flywheel check", group = "Test")
public class FlywheelCheck extends LinearOpMode {
    private static final double RUN_SECONDS = 3.0;
    /** The reading is averaged over the last part of each run, once the speed has settled. */
    private static final double MEASURE_LAST_SECONDS = 1.0;
    private static final double COAST_SECONDS = 2.5;
    private static final double HEALTHY_SPEED = 2300;

    private DcMotorEx right;
    private DcMotorEx left;

    // {right power, left power} for each run, and what to call it.
    private static final double[][] RUNS = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, 1}};
    private static final String[] NAMES = {
            "RIGHT alone, forwards", "RIGHT alone, backwards",
            "LEFT alone, forwards", "LEFT alone, backwards",
            "BOTH, as teleop does", "BOTH, right reversed"};

    // What each run measured: right speed, left speed, right amps, left amps, volts.
    private final double[][] results = new double[RUNS.length][];

    @Override
    public void runOpMode() {
        right = hardwareMap.get(DcMotorEx.class, "right_flywheel");
        left = hardwareMap.get(DcMotorEx.class, "left_flywheel");
        for (DcMotorEx motor : new DcMotorEx[] {right, left}) {
            motor.setDirection(FlywheelControl.DIRECTION);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addLine("Flywheel check: about 35 seconds, runs by itself.");
        telemetry.addLine("NO pollen in the robot. Hands clear of the wheels.");
        telemetry.addLine("Press START.");
        telemetry.update();
        waitForStart();

        try {
            for (int run = 0; run < RUNS.length && opModeIsActive(); run++) {
                results[run] = measure(run);
                setPowers(0, 0);
                // Let the wheels stop before the next run, so each one starts from rest.
                long until = System.nanoTime() + (long) (COAST_SECONDS * 1e9);
                while (opModeIsActive() && System.nanoTime() < until) {
                    show("stopping...");
                    sleep(50);
                }
            }
        } finally {
            setPowers(0, 0);
        }

        // Leave the answer up until somebody presses stop.
        while (opModeIsActive()) {
            show("DONE - take a photo of this screen");
            sleep(200);
        }
    }

    /** Runs one case at full power and returns the settled speeds, currents and voltage. */
    private double[] measure(int run) {
        double[] sum = new double[5];
        int samples = 0;
        long start = System.nanoTime();
        while (opModeIsActive()) {
            double seconds = (System.nanoTime() - start) / 1e9;
            if (seconds >= RUN_SECONDS) break;
            // Up to full power over the first 0.3 s rather than in one step.
            double ramp = Math.min(1.0, seconds / 0.3);
            setPowers(RUNS[run][0] * ramp, RUNS[run][1] * ramp);
            if (seconds >= RUN_SECONDS - MEASURE_LAST_SECONDS) {
                sum[0] += right.getVelocity();
                sum[1] += left.getVelocity();
                sum[2] += right.getCurrent(CurrentUnit.AMPS);
                sum[3] += left.getCurrent(CurrentUnit.AMPS);
                sum[4] += lowestVolts();
                samples++;
            }
            show("running: " + NAMES[run]);
            sleep(20);
        }
        if (samples == 0) return null;
        for (int i = 0; i < sum.length; i++) sum[i] /= samples;
        return sum;
    }

    private void setPowers(double rightPower, double leftPower) {
        right.setPower(rightPower);
        left.setPower(leftPower);
    }

    /** The lowest hub voltage: the hub that sags most is the one feeding the flywheels. */
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
        telemetry.addLine("speed R / L  |  amps R / L  |  volts");
        for (int run = 0; run < RUNS.length; run++) {
            double[] r = results[run];
            telemetry.addLine(NAMES[run] + ":");
            telemetry.addLine(r == null ? "    ..." : String.format("    %.0f / %.0f  |  %.1f / %.1f A  |  %.1f V", r[0], r[1], r[2], r[3], r[4]));
        }
        String verdict = verdict();
        if (verdict != null) telemetry.addLine(verdict);
        telemetry.update();
    }

    /** What the numbers mean, in words, once every run is in. */
    private String verdict() {
        for (double[] r : results) if (r == null) return null;
        StringBuilder text = new StringBuilder("---- WHAT IT MEANS ----\n");
        double rightAlone = Math.max(Math.abs(results[0][0]), Math.abs(results[1][0]));
        double leftAlone = Math.max(Math.abs(results[2][1]), Math.abs(results[3][1]));
        double together = Math.abs(results[4][0]) + Math.abs(results[4][1]);
        double rightReversed = Math.abs(results[5][0]) + Math.abs(results[5][1]);

        // A wheel that turns while only the other one is powered is linked to it.
        double leftPulledAlong = Math.max(Math.abs(results[0][1]), Math.abs(results[1][1]));
        double rightPulledAlong = Math.max(Math.abs(results[2][0]), Math.abs(results[3][0]));
        boolean linked = leftPulledAlong > 200 || rightPulledAlong > 200;
        if (linked) text.append("The wheels are LINKED: powering one turns the other.\n");

        if (rightAlone < HEALTHY_SPEED) {
            text.append(String.format("RIGHT wheel is slow on its own (%.0f, healthy is %.0f+): it is dragging.\n", rightAlone, HEALTHY_SPEED));
        }
        if (leftAlone < HEALTHY_SPEED) {
            text.append(String.format("LEFT wheel is slow on its own (%.0f, healthy is %.0f+): it is dragging.\n", leftAlone, HEALTHY_SPEED));
        }
        if (rightReversed > 1.3 * together) {
            text.append("MUCH FASTER with the right wheel reversed: the two are fighting in teleop.\n");
            text.append("Tell the programmer: the right flywheel direction has to be flipped.\n");
        } else if (rightAlone >= HEALTHY_SPEED && leftAlone >= HEALTHY_SPEED && together < 0.8 * (rightAlone + leftAlone)) {
            text.append("Each wheel is fine alone but slow together: they rub or work against each other.\n");
        } else if (rightAlone >= HEALTHY_SPEED && leftAlone >= HEALTHY_SPEED) {
            text.append("Both wheels reach a healthy speed. The shooter itself is fine.\n");
        }
        return text.toString();
    }
}
