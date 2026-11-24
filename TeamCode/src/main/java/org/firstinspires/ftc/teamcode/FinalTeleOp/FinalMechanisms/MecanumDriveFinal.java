package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MecanumDriveFinal {
    private static final double DRIVE_DEADZONE = 0.05;
    private static final double STICK_LINEAR_PART = 0.4;
    private static final double POWER_WRITE_THRESHOLD = 0.01;
    private static final long POWER_RESEND_NANOS = 250_000_000L;

    private static final double MAX_POWER_RISE_PER_SECOND = 4.0;
    private static final double FULL_POWER_VOLTAGE = 10.5;
    private static final double LOW_VOLTAGE = 7.5;
    private static final double MIN_POWER_LIMIT = 0.35;
    private static final double VOLTAGE_SMOOTHING_SECONDS = 0.2;
    private static final double LIMIT_DROP_PER_SECOND = 5.0;
    private static final double LIMIT_RISE_PER_SECOND = 1.0;
    private static final long VOLTAGE_READ_NANOS = 100_000_000L;
    private static final long CURRENT_READ_NANOS = 100_000_000L;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private VoltageSensor battery;

    private final double[] wheelPowers = new double[4];
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

    private static final double HEADING_HOLD_SECONDS = 0.25;
    private static final double HEADING_WAIT_SECONDS = 2.0;
    private boolean hadHeading;
    private long lastGoodHeadingNanos;
    private long initNanos;
    private boolean needsRezero;
    private boolean wasNearStart = true;

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl_motor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br_motor");
        battery = hardwareMap.voltageSensor.iterator().hasNext() ? hardwareMap.voltageSensor.iterator().next() : null;

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        configureMotors();

        stop();
        powerLimit = 1.0;
        lowestVolts = Double.NaN;
        smoothedVolts = Double.NaN;

        headingOffsetRadians = 0;
        headingRadians = Double.NaN;
        headingReady = false;
        hadHeading = false;
        needsRezero = false;
        wasNearStart = true;
        initNanos = System.nanoTime();
        try {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.resetPosAndIMU();
        } catch (RuntimeException noPinpoint) {
            pinpoint = null;
        }
    }

    private void configureMotors() {
        for (DcMotorEx motor : new DcMotorEx[] {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void recoverFromHubReset() {
        HubReset.forget(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        try {
            configureMotors();
        } catch (TargetPositionNotSetException stillDown) {
        }
        java.util.Arrays.fill(lastWriteNanos, 0L);
    }

    private void writePower(DcMotorEx motor, double power) {
        try {
            motor.setPower(power);
        } catch (TargetPositionNotSetException hubReset) {
            recoverFromHubReset();
            try {
                motor.setPower(power);
            } catch (TargetPositionNotSetException stillDown) {
            }
        }
    }

    public void drive(double forward, double strafe, double rotate) {
        driveCubic(forward, strafe, rotate, 0, true);
    }

    public void driveWithRotatePower(double forward, double strafe, double rotatePower) {
        driveCubic(forward, strafe, rotatePower, 0, false);
    }

    private static double stickCurve(double amount) {
        return STICK_LINEAR_PART * amount + (1.0 - STICK_LINEAR_PART) * amount * amount * amount;
    }

    private void driveCubic(double forward, double strafe, double rotate, double heading, boolean rotateIsStick) {
        if (!Double.isFinite(forward) || !Double.isFinite(strafe) || !Double.isFinite(rotate)) {
            stop();
            return;
        }

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
        double dt = lastDriveNanos == 0 ? 0.0 : Math.min(gap, 0.05);
        lastDriveNanos = now;
        updateBatteryProtection(now, dt);

        double maxRise = MAX_POWER_RISE_PER_SECOND * dt;
        rampedForward = ramp(rampedForward, forward, maxRise);
        rampedStrafe = ramp(rampedStrafe, strafe, maxRise);
        rampedRotate = rotateIsStick ? ramp(rampedRotate, rotate, maxRise) : rotate;

        double robotForward = rampedForward * Math.cos(heading) - rampedStrafe * Math.sin(heading);
        double robotStrafe = rampedForward * Math.sin(heading) + rampedStrafe * Math.cos(heading);
        applyMotorPowers(robotForward, robotStrafe, rampedRotate, now);
    }

    private static double ramp(double current, double target, double maxRise) {
        if (target * current < 0) current = 0.0;
        if (Math.abs(target) <= Math.abs(current)) return target;
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

        double strongest = 0.0;
        for (double target : targets) strongest = Math.max(strongest, Math.abs(target));
        double cap = strongest > powerLimit ? powerLimit / strongest : 1.0;

        DcMotorEx[] motors = {frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor};
        for (int i = 0; i < 4; i++) {
            double power = targets[i] * cap;
            boolean changed = power != wheelPowers[i]
                    && (power == 0.0 || Math.abs(power - wheelPowers[i]) >= POWER_WRITE_THRESHOLD);
            if (changed || now - lastWriteNanos[i] >= POWER_RESEND_NANOS) {
                writePower(motors[i], power);
                wheelPowers[i] = power;
                lastWriteNanos[i] = now;
            }
        }
    }

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
        if (Double.isFinite(volts) && volts > 1.0) {
            batteryVolts = volts;
            if (!(lowestVolts <= volts)) lowestVolts = volts;
        }
    }

    public void addPowerTelemetry(Telemetry telemetry) {
        if (frontLeftMotor == null) return;
        long now = System.nanoTime();
        readBattery(now);
        if (now - lastCurrentNanos >= CURRENT_READ_NANOS) {
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
            stop();
            return;
        }
        driveCubic(forward, strafe, rotate, isFieldRelative()
                ? AngleUnit.normalizeRadians(headingRadians - headingOffsetRadians) : 0, true);
    }

    public void updateHeading() {
        if (pinpoint == null) {
            headingReady = false;
            return;
        }
        pinpoint.update();
        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        boolean gyroUsable = status != GoBildaPinpointDriver.DeviceStatus.NOT_READY
                && status != GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                && status != GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                && status != GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY;
        headingReady = gyroUsable && Double.isFinite(heading);
        if (headingReady) {
            long now = System.nanoTime();
            boolean nearStart = Math.abs(heading) < Math.toRadians(3)
                    && Math.hypot(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH)) < 1.0;
            if (hadHeading && nearStart && !wasNearStart
                    && now - lastGoodHeadingNanos > HEADING_HOLD_SECONDS * 1e9) {
                needsRezero = true;
            }
            wasNearStart = nearStart;
            headingRadians = heading;
            lastGoodHeadingNanos = now;
            hadHeading = true;
        }
    }

    public boolean isFieldRelative() {
        return hadHeading && !needsRezero && (headingReady
                || System.nanoTime() - lastGoodHeadingNanos <= HEADING_HOLD_SECONDS * 1e9);
    }

    public boolean isWaitingForHeading() {
        return pinpoint != null && !hadHeading && System.nanoTime() - initNanos <= HEADING_WAIT_SECONDS * 1e9;
    }

    public String getDriveStatus() {
        if (isFieldRelative()) return "Ready (field-centric)";
        if (isWaitingForHeading()) return "STOPPED: waiting for Pinpoint";
        if (pinpoint == null) return "ROBOT-CENTRIC: no Pinpoint in the robot config";
        if (needsRezero) return "ROBOT-CENTRIC: Pinpoint restarted - face forward, press OPTIONS";
        return "ROBOT-CENTRIC: Pinpoint heading unavailable";
    }

    public void resetFieldHeading() {
        updateHeading();
        if (headingReady) {
            headingOffsetRadians = headingRadians;
            needsRezero = false;
        }
    }

    public boolean isHeadingReady() {
        return headingReady;
    }

    public GoBildaPinpointDriver.DeviceStatus getHeadingStatus() {
        return pinpoint == null ? GoBildaPinpointDriver.DeviceStatus.NOT_READY : pinpoint.getDeviceStatus();
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
        if (frontLeftMotor != null) writePower(frontLeftMotor, 0);
        if (backLeftMotor != null) writePower(backLeftMotor, 0);
        if (frontRightMotor != null) writePower(frontRightMotor, 0);
        if (backRightMotor != null) writePower(backRightMotor, 0);
    }
}
