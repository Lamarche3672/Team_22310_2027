package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MecanumDriveFinal {
    private static final double DRIVE_DEADZONE = 0.05;
    /** How much of the stick curve is a straight line (the rest is cubic). See stickCurve(). */
    private static final double STICK_LINEAR_PART = 0.4;
    private static final double POWER_WRITE_THRESHOLD = 0.01;
    private static final long POWER_RESEND_NANOS = 250_000_000L;

    // ------------------------------------------------------------------ battery protection
    // Added because driving pulled the battery from 11 V to 4 V. If the drive feels wrong, tune
    // these instead of removing them: MAX_POWER_RISE_PER_SECOND = 1000 switches the ramp off,
    // MIN_POWER_LIMIT = 1.0 switches the low-battery limiter off.
    /** Drive power may rise this much per second (4 = zero to full in a quarter second). Slowing down is instant. */
    private static final double MAX_POWER_RISE_PER_SECOND = 4.0;
    /** At this battery voltage and above the drive gets full power. */
    private static final double FULL_POWER_VOLTAGE = 10.5;
    /** Between the two voltages the power slides down smoothly; at this one it is at its lowest. */
    private static final double LOW_VOLTAGE = 7.5;
    /** The drive is never cut below this fraction of the stick, so the robot can always move. */
    private static final double MIN_POWER_LIMIT = 0.35;
    /** The voltage is averaged over about this long, so the limit glides instead of jumping. */
    private static final double VOLTAGE_SMOOTHING_SECONDS = 0.35;
    private static final double LIMIT_DROP_PER_SECOND = 2.0;
    private static final double LIMIT_RISE_PER_SECOND = 1.0;
    private static final long VOLTAGE_READ_NANOS = 100_000_000L;  // 10 times a second
    private static final long CURRENT_READ_NANOS = 100_000_000L;  // one motor each time, so 4 motors in 0.4 s

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private VoltageSensor battery;

    private final double[] wheelPowers = new double[4];   // FL, BL, FR, BR as last sent
    private final long[] lastWriteNanos = new long[4];
    private double rampedForward;
    private double rampedStrafe;
    private double rampedRotate;
    private final double[] wheelAmps = new double[4];
    private double powerLimit = 1.0;
    private double batteryVolts = Double.NaN;
    private double smoothedVolts = Double.NaN;
    private double lowestVolts = Double.NaN;
    private int nextCurrentMotor;
    private long lastDriveNanos;
    private long lastVoltageNanos;
    private long lastCurrentNanos;
    private GoBildaPinpointDriver pinpoint;
    private double headingOffsetRadians;
    private double headingRadians = Double.NaN;
    private boolean headingReady;

    /** A Pinpoint read that fails now and then is normal: keep using the last good heading this long. */
    private static final double HEADING_HOLD_SECONDS = 0.25;
    /** At the start, wait this long for the Pinpoint to calibrate before driving without it. */
    private static final double HEADING_WAIT_SECONDS = 2.0;
    private boolean hadHeading;
    private long lastGoodHeadingNanos;
    private long initNanos;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl_motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br_motor");
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        // RUN_WITHOUT_ENCODER: the stick sets the motor power directly, the same way Pedro drives
        // these motors in autonomous. RUN_USING_ENCODER makes the hub chase a wheel speed, and if a
        // drive motor's encoder cable is missing or mismatched it jumps to FULL power for any stick
        // movement, even a small turn, which pulled the battery from 11 V down to 4 V.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();
        powerLimit = 1.0;
        lowestVolts = Double.NaN;
        smoothedVolts = Double.NaN;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        headingOffsetRadians = 0;
        headingRadians = Double.NaN;
        headingReady = false;
        hadHeading = false;
        initNanos = System.nanoTime();
        // Keep the robot stationary during INIT while the IMU calibrates.
        pinpoint.resetPosAndIMU();
    }

    public void drive(double forward, double strafe, double rotate) {
        driveCubic(forward, strafe, rotate, 0, true);
    }

    /**
     * Like drive(), but rotatePower is already a motor power worked out by a controller (auto-align),
     * not a stick. It skips the stick curve, the deadzone and the ramp, which would shrink or delay
     * a small correction.
     */
    public void driveWithRotatePower(double forward, double strafe, double rotatePower) {
        driveCubic(forward, strafe, rotatePower, 0, false);
    }

    /**
     * The stick curve: part straight line, part cubic. A pure cubic gives 0.125 power at half stick,
     * and with the motors on direct power that is not enough to start a mecanum robot moving, so the
     * first half of the stick did nothing and then the robot lurched. This starts moving early and
     * still gives fine control near the middle. STICK_LINEAR_PART = 0 is the old pure cubic.
     */
    private static double stickCurve(double amount) {
        return STICK_LINEAR_PART * amount + (1.0 - STICK_LINEAR_PART) * amount * amount * amount;
    }

    private void driveCubic(double forward, double strafe, double rotate, double heading, boolean rotateIsStick) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(rotate)) {
            stop();
            return;
        }

        // Curve the stick magnitude, not each axis: preserve the requested direction.
        double magnitude = Math.hypot(forward, strafe);
        double limitedMagnitude = Math.min(1.0, magnitude);
        double scale = magnitude > DRIVE_DEADZONE
                ? stickCurve(limitedMagnitude) / magnitude : 0;
        forward *= scale;
        strafe *= scale;
        rotate = Math.max(-1.0, Math.min(1.0, rotate));
        if (rotateIsStick) {
            rotate = Math.abs(rotate) > DRIVE_DEADZONE ? Math.signum(rotate) * stickCurve(Math.abs(rotate)) : 0;
        }

        long now = System.nanoTime();
        double gap = (now - lastDriveNanos) / 1e9;
        // After a pause (first loop, or the drive was stopped for a while) the ramp starts from now.
        double dt = lastDriveNanos == 0 || gap > 0.2 ? 0.0 : gap;
        lastDriveNanos = now;
        updateBatteryProtection(now, dt);

        // The ramp is on forward / strafe / rotate, before they are mixed into wheel powers, so the
        // wheels always keep the right balance. (Ramping each wheel on its own left the robot still
        // turning for a moment after the turn stick was let go.) Speeding up is ramped so four motors
        // never jump to full power at once; slowing down and changing direction are immediate.
        double maxRise = MAX_POWER_RISE_PER_SECOND * dt;
        rampedForward = ramp(rampedForward, forward, maxRise);
        rampedStrafe = ramp(rampedStrafe, strafe, maxRise);
        rampedRotate = rotateIsStick ? ramp(rampedRotate, rotate, maxRise) : rotate;

        // Pinpoint heading is CCW-positive; strafe input is right-positive.
        double robotForward = rampedForward * Math.cos(heading) - rampedStrafe * Math.sin(heading);
        double robotStrafe = rampedForward * Math.sin(heading) + rampedStrafe * Math.cos(heading);
        applyMotorPowers(robotForward, robotStrafe, rampedRotate, now);
    }

    private static double ramp(double current, double target, double maxRise) {
        if (target * current < 0) current = 0.0;                  // changing direction: let go at once
        if (Math.abs(target) <= Math.abs(current)) return target;  // slowing down: at once
        return current + Math.max(-maxRise, Math.min(maxRise, target - current));
    }

    private void applyMotorPowers(double forward, double strafe, double rotate, long now) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        double[] targets = {frontLeftPower / maxPower, backLeftPower / maxPower,
                frontRightPower / maxPower, backRightPower / maxPower};

        // Low battery: the strongest wheel is capped at the limit and the others shrink with it,
        // so the robot keeps its direction. Commands already under the limit are not touched.
        double strongest = 0.0;
        for (double target : targets) strongest = Math.max(strongest, Math.abs(target));
        double cap = strongest > powerLimit ? powerLimit / strongest : 1.0;

        DcMotorEx[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (int i = 0; i < 4; i++) {
            double power = targets[i] * cap;
            // A change smaller than 1 % is not worth a command to the hub (each one takes a few
            // milliseconds), but the power is sent again every quarter second regardless, so the
            // motors recover by themselves if the hub dropped a command or restarted.
            boolean changed = power != wheelPowers[i]
                    && (power == 0.0 || Math.abs(power - wheelPowers[i]) >= POWER_WRITE_THRESHOLD);
            if (changed || now - lastWriteNanos[i] >= POWER_RESEND_NANOS) {
                motors[i].setPower(power);
                wheelPowers[i] = power;
                lastWriteNanos[i] = now;
            }
        }
    }

    /**
     * Watches the battery so the Control Hub does not brown out. The drive power follows the
     * (averaged) battery voltage on a slope: full power at FULL_POWER_VOLTAGE and above, sliding
     * down to MIN_POWER_LIMIT at LOW_VOLTAGE. A slope settles at a steady power; switching the
     * power down and back up at two fixed voltages made the drive surge.
     */
    private void updateBatteryProtection(long now, double dt) {
        readBattery(now);
        if (Double.isNaN(batteryVolts)) return;
        smoothedVolts = Double.isNaN(smoothedVolts) ? batteryVolts
                : smoothedVolts + (batteryVolts - smoothedVolts) * Math.min(1.0, dt / VOLTAGE_SMOOTHING_SECONDS);
        double fraction = (smoothedVolts - LOW_VOLTAGE) / (FULL_POWER_VOLTAGE - LOW_VOLTAGE);
        double wanted = MIN_POWER_LIMIT + (1.0 - MIN_POWER_LIMIT) * Math.max(0.0, Math.min(1.0, fraction));
        powerLimit += Math.max(-LIMIT_DROP_PER_SECOND * dt, Math.min(LIMIT_RISE_PER_SECOND * dt, wanted - powerLimit));
    }

    private void readBattery(long now) {
        if (battery == null || now - lastVoltageNanos < VOLTAGE_READ_NANOS) return;
        lastVoltageNanos = now;
        double volts = battery.getVoltage();
        if (volts > 1.0) {   // 0 means the read failed
            batteryVolts = volts;
            if (!(lowestVolts <= volts)) lowestVolts = volts;
        }
    }

    /** Battery and per-motor current on the Driver Station, to find what is pulling the battery down. */
    public void addPowerTelemetry(Telemetry telemetry) {
        if (frontLeftMotor == null) return;   // init() failed: nothing to read
        long now = System.nanoTime();
        readBattery(now);
        if (now - lastCurrentNanos >= CURRENT_READ_NANOS) {
            // Each current read is a separate request to the hub. Reading one motor per turn
            // instead of all four at once keeps the drive loop from hiccuping.
            lastCurrentNanos = now;
            DcMotorEx[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
            wheelAmps[nextCurrentMotor] = motors[nextCurrentMotor].getCurrent(CurrentUnit.AMPS);
            nextCurrentMotor = (nextCurrentMotor + 1) % motors.length;
        }
        telemetry.addData("Battery (V)", "%.1f now, %.1f lowest", batteryVolts, lowestVolts);
        telemetry.addData("Drive power limit", powerLimit < 0.999
                ? String.format("%.0f%% - battery low, power eased off", powerLimit * 100) : "100%");
        telemetry.addData("Drive amps FL / BL / FR / BR", "%.1f / %.1f / %.1f / %.1f",
                wheelAmps[0], wheelAmps[1], wheelAmps[2], wheelAmps[3]);
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        updateHeading();
        if (isWaitingForHeading()) {
            // Just started and the Pinpoint is still calibrating: stay still so it calibrates properly.
            stop();
            return;
        }
        // Field-centric while the heading is good (one failed read is bridged with the last heading).
        // If the Pinpoint is lost for longer, drive robot-centric instead of leaving the robot dead.
        driveCubic(forward, strafe, rotate, isFieldRelative()
                ? AngleUnit.normalizeRadians(headingRadians - headingOffsetRadians) : 0, true);
    }

    public void updateHeading() {
        // Heading-only reads do not refresh calibration/fault status.
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        // The heading comes from the Pinpoint's gyro, so it is still good when a pod cable is loose
        // (the pod-not-detected faults). Only a failed read, calibration or a gyro fault makes it unusable.
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        boolean gyroUsable = status != GoBildaPinpointDriver.DeviceStatus.NOT_READY
                && status != GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                && status != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                && status != GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY;
        headingReady = gyroUsable && Double.isFinite(heading);
        if (headingReady) {
            // Only a good read replaces the heading: a failed read reports 0, which is not a real heading.
            headingRadians = heading;
            lastGoodHeadingNanos = System.nanoTime();
            hadHeading = true;
        }
    }

    /** True while the drive is field-centric: a good heading now, or one from the last quarter second. */
    public boolean isFieldRelative() {
        return hadHeading && (headingReady
                || System.nanoTime() - lastGoodHeadingNanos <= HEADING_HOLD_SECONDS * 1e9);
    }

    /** True for the first moments after init while the Pinpoint has not given a heading yet. */
    public boolean isWaitingForHeading() {
        return !hadHeading && System.nanoTime() - initNanos <= HEADING_WAIT_SECONDS * 1e9;
    }

    public String getDriveStatus() {
        if (isFieldRelative()) return "Ready (field-centric)";
        if (isWaitingForHeading()) return "STOPPED: waiting for Pinpoint";
        return "ROBOT-CENTRIC: Pinpoint heading unavailable";
    }

    public void resetFieldHeading() {
        updateHeading();
        if (headingReady) {
            // Software zero only: do not recalibrate the IMU while driving.
            headingOffsetRadians = headingRadians;
        }
    }

    public boolean isHeadingReady() {
        return headingReady;
    }

    public GoBildaPinpointDriver.DeviceStatus getHeadingStatus() {
        return pinpoint.getDeviceStatus();
    }

    public double getFieldHeadingDegrees() {
        return isFieldRelative() ? Math.toDegrees(AngleUnit.normalizeRadians(
                headingRadians - headingOffsetRadians)) : Double.NaN;
    }

    public void stop() {
        java.util.Arrays.fill(wheelPowers, 0.0);
        rampedForward = 0.0;
        rampedStrafe = 0.0;
        rampedRotate = 0.0;
        // The motors are null if init() failed (for example a wrong name in the configuration).
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);
    }
}
