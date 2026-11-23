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

    // ------------------------------------------------------------------ battery protection
    // Added because driving pulled the battery from 11 V to 4 V. If the drive feels wrong, tune
    // these instead of removing them: MAX_POWER_RISE_PER_SECOND = 1000 switches the ramp off,
    // MIN_POWER_LIMIT = 1.0 switches the low-battery limiter off.
    /** Wheel power may rise this much per second (4 = zero to full in a quarter second). Slowing down is instant. */
    private static final double MAX_POWER_RISE_PER_SECOND = 4.0;
    /** When the battery drops below this while driving, the drive power is cut back... */
    private static final double LOW_VOLTAGE = 9.0;
    /** ...and given back once the battery is above this again. */
    private static final double RECOVERED_VOLTAGE = 10.0;
    /** The drive is never cut below this fraction of the stick, so the robot can always move. */
    private static final double MIN_POWER_LIMIT = 0.35;
    private static final double LIMIT_DROP_PER_SECOND = 3.0;
    private static final double LIMIT_RISE_PER_SECOND = 0.5;
    private static final long VOLTAGE_READ_NANOS = 50_000_000L;   // 20 times a second
    private static final long CURRENT_READ_NANOS = 250_000_000L;  // 4 times a second

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private VoltageSensor battery;

    private final double[] wheelPowers = new double[4];   // FL, BL, FR, BR as last sent
    private final double[] wheelAmps = new double[4];
    private double powerLimit = 1.0;
    private double batteryVolts = Double.NaN;
    private double lowestVolts = Double.NaN;
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
     * not a stick. It skips the stick curve and deadzone, which would shrink a small correction to nothing.
     */
    public void driveWithRotatePower(double forward, double strafe, double rotatePower) {
        driveCubic(forward, strafe, rotatePower, 0, false);
    }

    private void driveCubic(double forward, double strafe, double rotate, double heading, boolean rotateIsStick) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(rotate)) {
            stop();
            return;
        }

        // Cube the stick magnitude, not each axis: preserve the requested direction.
        double magnitude = Math.hypot(forward, strafe);
        double limitedMagnitude = Math.min(1.0, magnitude);
        double scale = magnitude > DRIVE_DEADZONE
                ? Math.pow(limitedMagnitude, 3) / magnitude : 0;
        forward *= scale;
        strafe *= scale;
        rotate = Math.max(-1.0, Math.min(1.0, rotate));
        if (rotateIsStick) {
            rotate = Math.abs(rotate) > DRIVE_DEADZONE ? Math.pow(rotate, 3) : 0;
        }

        // Pinpoint heading is CCW-positive; strafe input is right-positive.
        double robotForward = forward * Math.cos(heading) - strafe * Math.sin(heading);
        double robotStrafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        applyMotorPowers(robotForward, robotStrafe, rotate);
    }

    private void applyMotorPowers(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double backLeftPower = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower)),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        long now = System.nanoTime();
        double dt = lastDriveNanos == 0 ? 0.0 : Math.min(0.1, (now - lastDriveNanos) / 1e9);
        lastDriveNanos = now;
        updateBatteryProtection(now, dt);

        double[] targets = {frontLeftPower / maxPower, backLeftPower / maxPower,
                frontRightPower / maxPower, backRightPower / maxPower};
        double maxRise = MAX_POWER_RISE_PER_SECOND * dt;
        for (int i = 0; i < 4; i++) {
            double target = targets[i] * powerLimit;
            // Changing direction: let go of the old direction at once, then ramp up the new one.
            double current = target * wheelPowers[i] < 0 ? 0.0 : wheelPowers[i];
            boolean slowingDown = Math.abs(target) <= Math.abs(current);
            // Speeding up is ramped so four motors never jump to full power at once, which is the
            // biggest current spike a drivetrain makes. Slowing down is immediate.
            wheelPowers[i] = slowingDown ? target : current + Math.max(-maxRise, Math.min(maxRise, target - current));
        }
        frontLeftMotor.setPower(wheelPowers[0]);
        backLeftMotor.setPower(wheelPowers[1]);
        frontRightMotor.setPower(wheelPowers[2]);
        backRightMotor.setPower(wheelPowers[3]);
    }

    /**
     * Watches the battery. If driving pulls it under LOW_VOLTAGE the drive power is turned down
     * until it holds, so the Control Hub does not brown out; it comes back as the battery recovers.
     */
    private void updateBatteryProtection(long now, double dt) {
        readBattery(now);
        if (batteryVolts < LOW_VOLTAGE) {
            powerLimit = Math.max(MIN_POWER_LIMIT, powerLimit - LIMIT_DROP_PER_SECOND * dt);
        } else if (batteryVolts > RECOVERED_VOLTAGE) {
            powerLimit = Math.min(1.0, powerLimit + LIMIT_RISE_PER_SECOND * dt);
        }
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
            lastCurrentNanos = now;
            wheelAmps[0] = frontLeftMotor.getCurrent(CurrentUnit.AMPS);
            wheelAmps[1] = backLeftMotor.getCurrent(CurrentUnit.AMPS);
            wheelAmps[2] = frontRightMotor.getCurrent(CurrentUnit.AMPS);
            wheelAmps[3] = backRightMotor.getCurrent(CurrentUnit.AMPS);
        }
        telemetry.addData("Battery (V)", "%.1f now, %.1f lowest", batteryVolts, lowestVolts);
        telemetry.addData("Drive power limit", powerLimit < 0.999
                ? String.format("%.0f%% - BATTERY SAGGING, power cut back", powerLimit * 100) : "100%");
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
        headingReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && Double.isFinite(heading);
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
        // The motors are null if init() failed (for example a wrong name in the configuration).
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);
    }
}
