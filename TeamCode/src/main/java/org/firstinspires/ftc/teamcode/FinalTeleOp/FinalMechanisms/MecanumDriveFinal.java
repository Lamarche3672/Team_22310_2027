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
        // movement, even a small turn, which pulls the battery down hard.
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
        // Keep the robot stationary during INIT while the IMU calibrates.
        pinpoint.resetPosAndIMU();
    }

    public void drive(double forward, double strafe, double rotate) {
        driveCubic(forward, strafe, rotate, 0);
    }

    private void driveCubic(double forward, double strafe, double rotate, double heading) {
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
        rotate = Math.abs(rotate) > DRIVE_DEADZONE
                ? Math.pow(Math.max(-1.0, Math.min(1.0, rotate)), 3) : 0;

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
            double current = wheelPowers[i];
            boolean slowingDown = target * current >= 0 && Math.abs(target) <= Math.abs(current);
            // Speeding up (or reversing) is ramped so four motors never jump to full power at once,
            // which is the biggest current spike a drivetrain makes. Slowing down is immediate.
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
        if (battery == null) return;
        if (now - lastVoltageNanos >= VOLTAGE_READ_NANOS) {
            lastVoltageNanos = now;
            double volts = battery.getVoltage();
            if (volts > 1.0) {   // 0 means the read failed
                batteryVolts = volts;
                if (!(lowestVolts <= volts)) lowestVolts = volts;
            }
        }
        if (batteryVolts < LOW_VOLTAGE) {
            powerLimit = Math.max(MIN_POWER_LIMIT, powerLimit - LIMIT_DROP_PER_SECOND * dt);
        } else if (batteryVolts > RECOVERED_VOLTAGE) {
            powerLimit = Math.min(1.0, powerLimit + LIMIT_RISE_PER_SECOND * dt);
        }
    }

    /** Battery and per-motor current on the Driver Station, to find what is pulling the battery down. */
    public void addPowerTelemetry(Telemetry telemetry) {
        long now = System.nanoTime();
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
        if (!headingReady) {
            stop();
            return;
        }
        driveCubic(forward, strafe, rotate,
                AngleUnit.normalizeRadians(headingRadians - headingOffsetRadians));
    }

    public void updateHeading() {
        // Heading-only reads do not refresh calibration/fault status.
        pinpoint.update();
        headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);
        headingReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                && Double.isFinite(headingRadians);
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
        return headingReady ? Math.toDegrees(AngleUnit.normalizeRadians(
                headingRadians - headingOffsetRadians)) : Double.NaN;
    }

    public void stop() {
        java.util.Arrays.fill(wheelPowers, 0.0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
