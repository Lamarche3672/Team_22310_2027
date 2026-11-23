package org.firstinspires.ftc.teamcode.FinalTeleOp.FinalMechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDriveFinal {
    private static final double DRIVE_DEADZONE = 0.05;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
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

        frontLeftMotor.setPower(frontLeftPower / maxPower);
        backLeftMotor.setPower(backLeftPower / maxPower);
        frontRightMotor.setPower(frontRightPower / maxPower);
        backRightMotor.setPower(backRightPower / maxPower);
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
        // The motors are null if init() failed (for example a wrong name in the configuration).
        if (frontLeftMotor != null) frontLeftMotor.setPower(0);
        if (backLeftMotor != null) backLeftMotor.setPower(0);
        if (frontRightMotor != null) frontRightMotor.setPower(0);
        if (backRightMotor != null) backRightMotor.setPower(0);
    }
}
