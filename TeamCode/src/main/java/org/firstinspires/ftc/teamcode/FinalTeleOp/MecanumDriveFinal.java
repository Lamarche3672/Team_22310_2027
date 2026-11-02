package org.firstinspires.ftc.teamcode.FinalTeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.function.LongSupplier;

public class MecanumDriveFinal {
    private static final double DRIVE_DEADZONE = 0.05;
    // Power units per second: about 0.6 s to full power, 0.33 s to release it.
    private static final double POWER_RAMP_UP = 1.6;
    private static final double POWER_RAMP_DOWN = 3.0;
    // A delayed loop must not cause a large power jump when it resumes.
    private static final double MAX_RAMP_TIMESTEP_SECONDS = 0.05;

    private final LongSupplier nanoTime;
    private final double[] appliedPowers = new double[4];
    private final double[] targetPowers = new double[4];
    private long lastPowerUpdateNanos;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private GoBildaPinpointDriver pinpoint;
    private double headingOffsetRadians;
    private double headingRadians = Double.NaN;
    private boolean headingReady;

    public MecanumDriveFinal() {
        this(System::nanoTime);
    }

    // Injectable monotonic clock for deterministic ramp tests.
    MecanumDriveFinal(LongSupplier nanoTime) {
        this.nanoTime = nanoTime;
    }

    public void init(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotor.class, "fl_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "fr_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "bl_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "br_motor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stop();

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
        // Rescale outside the deadzone so power starts continuously at zero.
        double limitedMagnitude = Math.max(0,
                (Math.min(1.0, magnitude) - DRIVE_DEADZONE) / (1.0 - DRIVE_DEADZONE));
        double scale = magnitude > DRIVE_DEADZONE
                ? Math.pow(limitedMagnitude, 3) / magnitude : 0;
        forward *= scale;
        strafe *= scale;
        double turnMagnitude = Math.max(0,
                (Math.min(1.0, Math.abs(rotate)) - DRIVE_DEADZONE) / (1.0 - DRIVE_DEADZONE));
        rotate = Math.copySign(Math.pow(turnMagnitude, 3), rotate);

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

        targetPowers[0] = frontLeftPower / maxPower;
        targetPowers[1] = backLeftPower / maxPower;
        targetPowers[2] = frontRightPower / maxPower;
        targetPowers[3] = backRightPower / maxPower;

        long now = nanoTime.getAsLong();
        double dt = Math.max(0, Math.min(MAX_RAMP_TIMESTEP_SECONDS,
                (now - lastPowerUpdateNanos) * 1e-9));
        lastPowerUpdateNanos = now;

        // Use one interpolation fraction for all wheels. Independent wheel clamps
        // can distort the requested translation/turn mix, especially on diagonals.
        double fraction = 1.0;
        for (int i = 0; i < appliedPowers.length; i++) {
            double change = Math.abs(targetPowers[i] - appliedPowers[i]);
            if (change > 0) {
                boolean reversing = targetPowers[i] * appliedPowers[i] < 0;
                boolean speedingUp = Math.abs(targetPowers[i]) > Math.abs(appliedPowers[i]);
                double rate = reversing || speedingUp ? POWER_RAMP_UP : POWER_RAMP_DOWN;
                fraction = Math.min(fraction, rate * dt / change);
            }
        }
        for (int i = 0; i < appliedPowers.length; i++) {
            appliedPowers[i] = fraction >= 1.0 ? targetPowers[i]
                    : appliedPowers[i] + fraction * (targetPowers[i] - appliedPowers[i]);
        }

        frontLeftMotor.setPower(appliedPowers[0]);
        backLeftMotor.setPower(appliedPowers[1]);
        frontRightMotor.setPower(appliedPowers[2]);
        backRightMotor.setPower(appliedPowers[3]);
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
        // Explicit STOP, invalid inputs and heading faults bypass the normal ramp.
        Arrays.fill(appliedPowers, 0);
        Arrays.fill(targetPowers, 0);
        lastPowerUpdateNanos = nanoTime.getAsLong();
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
