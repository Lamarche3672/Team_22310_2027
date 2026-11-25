package org.firstinspires.ftc.teamcode.Shared;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * Finds our hive with the Limelight and works out which way the robot has to face to shoot
 * at it, and how far away it is. Teleop and autonomous both use it.
 *
 * The idea that makes it fast and steady: the camera is only used to find WHERE to point.
 * Actually getting there is done on the gyro.
 *
 *   camera frame:  "the hive is 12 degrees to the right"
 *   so:            aim heading = (heading the robot had when that picture was taken) - 12
 *   every loop:    turn until the gyro heading equals the aim heading
 *
 * The camera only gives a new picture every 30 to 50 ms and each one is already ~50 ms old
 * when we get it. Steering straight off the camera means steering on old news, which is what
 * makes camera-only aiming slow and wobbly. The gyro is read every loop with almost no delay.
 * A second win: if the tag drops out of view for a moment, the aim heading is still valid, so
 * the robot stays locked on.
 *
 * "Heading" can be any gyro heading as long as it is always the same one: teleop passes the
 * raw Pinpoint heading, autonomous passes the Pedro pose heading.
 *
 * About the hive (competition manual sections 9.6, 9.9 and 10.5): each alliance's hive is a
 * seesaw with a CELL at each end. Only the cell that is tilted UP can be scored in, and every
 * TIP swaps which one that is. Under each cell is a sticker with four AprilTags in a row. So
 * "aim at our hive" really means: find our two stickers, work out which cell is up, and point
 * at the middle of that sticker.
 */
public class HiveAim {
    // ---- camera setup
    /** The Limelight pipeline that detects AprilTags. Change this if ours is in another slot. */
    public static final int PIPELINE = 0;

    // ---- which tags belong to which hive (competition manual, section 9.9). Each hive has
    // two cells with four tags each, numbered in a row: red 30-37, blue 38-45.
    public static final int FIRST_RED_TAG = 30;
    public static final int FIRST_BLUE_TAG = 38;
    public static final int TAGS_PER_HIVE = 8;
    public static final int TAGS_PER_CELL = 4;
    /**
     * The cell that is up when a match starts: the second group of four for both alliances
     * (red 34-37, blue 42-45). After that the camera works it out, because it flips on every tip.
     */
    public static final int START_UP_CELL = 1;
    /**
     * Where the four tags sit across their sticker, in inches from its middle, in ID order
     * (from the SDK's AprilTagGameDatabase). Knowing this lets us find the middle of the
     * sticker even when only some of the tags are in view, and gives us the distance from
     * how far apart the tags look.
     */
    private static final double[] TAG_OFFSET_INCHES = {-6.5, -2.75, 2.75, 6.5};
    /** In the camera image y grows downward, so the higher cell has the SMALLER y. */
    private static final boolean IMAGE_Y_GROWS_DOWN = true;
    /** How long a camera sighting of "this cell is up" is trusted before a lone sticker is taken at face value. */
    private static final double UP_CELL_MEMORY_SECONDS = 10.0;

    // ---- things to flip if the real robot disagrees
    /** Make this -1 if the robot turns AWAY from the hive (camera mounted upside down). */
    public static final double TX_SIGN = 1.0;
    /** Shots land a little left or right every time? Positive aims further right, in degrees. */
    public static final double AIM_OFFSET_DEGREES = 0;

    // ---- the turn controller: power = KP x error - KD x turn rate (error in radians, rate in
    // radians per second). These came out of a simulation sweep over fast and slow robots with
    // 0 to 50 ms of delay. The ratio matters most: KD about 0.12 x KP is what stops overshoot
    // when there is delay in the loop. Raising KP alone makes it wobble.
    public static final double KP = 2.5;
    public static final double KD = 0.30;
    public static final double MAX_TURN_POWER = 0.7;
    /** A small extra shove in the right direction so the last few degrees are not left hanging. */
    public static final double MIN_TURN_POWER = 0.05;
    // A mecanum drive needs a certain power before it turns at all, and that depends on the
    // carpet. If we are off target and not moving, add power slowly until it breaks free.
    public static final double STALL_PUSH_PER_SECOND = 0.4;
    public static final double MAX_STALL_PUSH = 0.25;
    public static final double STALLED_TURN_RATE = 0.1;
    /** This close and this still: stop correcting, or the motors would buzz forever. */
    public static final double REST_DEGREES = 0.4;

    // ---- "aimed" (for the buzz and for autonomous). Two thresholds so it does not flicker:
    // tight to become aimed, a bit looser to stay aimed. The cell opening is 20 inches wide,
    // which is about 5 degrees either side at 8 feet, so 2 degrees is aimed with room to spare.
    public static final double AIMED_DEGREES = 2.0;
    public static final double STILL_AIMED_DEGREES = 3.0;
    public static final double AIMED_TURN_RATE = 0.3;
    public static final double STILL_AIMED_TURN_RATE = 0.8;

    // ---- how long old information is trusted
    /** Keep holding the last aim heading this long after the tag was last seen. */
    public static final double HOLD_SECONDS = 1.5;
    public static final double SEEN_SECONDS = 0.3;
    public static final double DISTANCE_SECONDS = 1.0;

    // The SDK asks the camera 100 times a second but the camera only has a new picture every
    // 20 to 50 ms, so a "new" picture has already waited a while. Half a frame on average.
    private static final double EXTRA_LATENCY_SECONDS = 0.015;
    // Tags further apart than MAX or closer together than MIN (in degrees per inch of sticker)
    // would put the hive under a foot or over 90 feet away. That is a misread, not a distance.
    private static final double MIN_DEGREES_PER_INCH = 0.05;
    private static final double MAX_DEGREES_PER_INCH = 5.0;
    private static final int HISTORY = 64;

    /**
     * Which alliance we are on. Static on purpose: autonomous sets it, and it is still set
     * when MAIN TELEOP starts a few seconds later. Teleop can change it with X / B at INIT.
     */
    public static boolean blueAlliance;

    private Limelight3A limelight;
    private volatile boolean pipelineSet;
    private volatile boolean stopped;
    private double lastStamp = Double.NaN;

    // Recent gyro headings, so we can look up where the robot was pointing when a picture
    // was taken instead of where it points now.
    private final long[] historyNanos = new long[HISTORY];
    private final double[] historyHeading = new double[HISTORY];
    private int historyCount;
    private int historyNext;

    private boolean hasAim;
    private double aimHeading;
    private long lastSeenNanos;
    private double errorDegrees = Double.NaN;
    private double distanceInches = Double.NaN;
    private String tagsInView = "";
    private int pipelineInUse = -1;

    // Which cell is up, and when the camera last confirmed it. Static like the alliance: a tip
    // in autonomous is still true when teleop starts.
    private static int upCell = START_UP_CELL;
    private static long upCellSeenNanos;
    private boolean aimingAtDownCell;
    private int lastTarget = -1;
    /** How many degrees one inch of sticker covers, with its sign (which way round the sticker looks). */
    private double degreesPerInch = Double.NaN;

    private boolean aimed;
    private boolean wasAimed;
    private double stallPush;
    private long lastControlNanos;

    /** Call at the start of a match (autonomous does): the hive is back in its starting position. */
    public static void matchStart() {
        upCell = START_UP_CELL;
        upCellSeenNanos = 0;
    }

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        stopped = false;
        pipelineSet = false;

        // Switching pipeline talks to the camera over the network and can block for seconds if
        // the Limelight is still booting. Do it on the side so INIT never hangs, and keep
        // trying until it answers.
        Limelight3A camera = limelight;
        Thread setup = new Thread(() -> {
            for (int attempt = 0; attempt < 20 && !stopped && !pipelineSet; attempt++) {
                try {
                    pipelineSet = camera.pipelineSwitch(PIPELINE);
                    if (!pipelineSet) Thread.sleep(1500);
                } catch (InterruptedException end) {
                    return;
                } catch (RuntimeException retry) {
                    // Camera not reachable yet. Try again.
                }
            }
        });
        setup.setDaemon(true);
        setup.start();
        limelight.start();
    }

    public void stop() {
        stopped = true;
        if (limelight != null) limelight.stop();
    }

    /**
     * Call every loop with the current gyro heading (radians). headingOk = false when the
     * gyro cannot be trusted right now; the camera is still read so distance keeps working.
     */
    public void update(double heading, boolean headingOk) {
        long now = System.nanoTime();
        if (headingOk) noteHeading(now, heading);
        if (limelight == null) return;

        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (RuntimeException noResult) {
            return;
        }
        if (result == null || !result.isValid()) return;

        // The SDK polls the camera faster than the camera makes pictures, so most loops hand
        // us the same picture again. The camera's own timestamp tells us when it is new.
        double stamp = result.getTimestamp();
        if (stamp == 0) stamp = result.getControlHubTimeStamp();
        if (stamp == lastStamp) return;
        lastStamp = stamp;

        pipelineInUse = result.getPipelineIndex();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        int count = tags == null ? 0 : tags.size();
        int[] ids = new int[count];
        double[] tx = new double[count];
        double[] yPixels = new double[count];
        for (int i = 0; i < count; i++) {
            LLResultTypes.FiducialResult tag = tags.get(i);
            ids[i] = tag.getFiducialId();
            // "No crosshair" = measured from the centre of the lens, so it cannot be thrown
            // off by somebody moving the crosshair in the Limelight web page.
            tx[i] = tag.getTargetXDegreesNoCrosshair();
            yPixels[i] = tag.getTargetYPixels();
        }

        // How old the picture is by the time we look at it.
        double age = (result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0
                + EXTRA_LATENCY_SECONDS;
        if (!Double.isFinite(age)) age = EXTRA_LATENCY_SECONDS;
        onTags(ids, tx, yPixels, Math.max(0, Math.min(0.3, age)), now, headingOk);
    }

    /**
     * One camera picture, as plain numbers: for every tag in view its ID, how many degrees to
     * the right of centre it is, and how far down the image it is.
     */
    void onTags(int[] ids, double[] tx, double[] yPixels, double ageSeconds, long now, boolean headingOk) {
        StringBuilder seen = new StringBuilder();
        int firstTag = blueAlliance ? FIRST_BLUE_TAG : FIRST_RED_TAG;
        int cells = TAGS_PER_HIVE / TAGS_PER_CELL;
        int[] count = new int[cells];
        double[] sumTx = new double[cells], sumOffset = new double[cells], sumY = new double[cells];
        double[] sumOffsetSquared = new double[cells], sumOffsetTx = new double[cells];
        for (int i = 0; i < ids.length; i++) {
            if (seen.length() > 0) seen.append(' ');
            seen.append(ids[i]);
            int index = ids[i] - firstTag;
            if (index < 0 || index >= TAGS_PER_HIVE || !Double.isFinite(tx[i])) continue;
            int c = index / TAGS_PER_CELL;
            double offset = TAG_OFFSET_INCHES[index % TAGS_PER_CELL];
            count[c]++;
            sumTx[c] += tx[i];
            sumOffset[c] += offset;
            sumOffsetSquared[c] += offset * offset;
            sumOffsetTx[c] += offset * tx[i];
            sumY[c] += yPixels[i];
        }
        tagsInView = seen.toString();
        if (count[0] == 0 && count[1] == 0) return;

        // Both stickers in view: the one higher up in the picture is the cell that is up.
        if (count[0] > 0 && count[1] > 0) {
            boolean firstIsHigher = (sumY[0] / count[0] < sumY[1] / count[1]) == IMAGE_Y_GROWS_DOWN;
            upCell = firstIsHigher ? 0 : 1;
            upCellSeenNanos = now;
        }
        // Aim at the up cell if we can see it. If only the other sticker is in view it still
        // points us at the hive (both cells are in a line), which is better than nothing.
        int target = count[upCell] > 0 ? upCell : 1 - upCell;
        boolean upCellIsFresh = upCellSeenNanos != 0 && now - upCellSeenNanos <= UP_CELL_MEMORY_SECONDS * 1e9;
        aimingAtDownCell = target != upCell && upCellIsFresh;

        // The middle of the sticker. With two or more tags, fit a straight line through
        // (position on the sticker, angle seen): where that line crosses position 0 is the
        // middle, whichever tags happened to be visible. Its slope is how many degrees one
        // inch of sticker covers, and that shrinks with distance.
        int n = count[target];
        double meanTx = sumTx[target] / n;
        double meanOffset = sumOffset[target] / n;
        double spread = sumOffsetSquared[target] - n * meanOffset * meanOffset;
        double centre = meanTx;
        double distance = Double.NaN;
        if (n >= 2 && spread > 1e-6) {
            double slope = (sumOffsetTx[target] - n * meanOffset * meanTx) / spread;
            if (Math.abs(slope) >= MIN_DEGREES_PER_INCH && Math.abs(slope) <= MAX_DEGREES_PER_INCH) {
                degreesPerInch = slope;
                centre = meanTx - slope * meanOffset;
                distance = 1.0 / Math.tan(Math.toRadians(Math.abs(slope)));
            }
        } else if (Double.isFinite(degreesPerInch)) {
            // Only one tag: use the last known scale to step from that tag to the middle.
            centre = meanTx - degreesPerInch * meanOffset;
        }
        // A distance to the DOWN cell would pick the wrong shot speed (it is about two feet
        // further away), so only the up cell's distance is passed on.
        if (aimingAtDownCell) distance = Double.NaN;
        // Switched to the other cell (a tip): its distance is a different number, so do not
        // blend it with the old one.
        if (target != lastTarget) distanceInches = Double.NaN;
        lastTarget = target;
        onFrame(centre, distance, ageSeconds, now, headingOk);
    }

    /** One new camera picture: the hive is txDegrees to the right, and the picture is ageSeconds old. */
    void onFrame(double txDegrees, double distance, double ageSeconds, long now, boolean headingOk) {
        if (!Double.isFinite(txDegrees)) return;
        boolean wasRecent = lastSeenNanos != 0 && now - lastSeenNanos <= DISTANCE_SECONDS * 1e9;
        errorDegrees = TX_SIGN * txDegrees - AIM_OFFSET_DEGREES;
        lastSeenNanos = now;

        if (headingOk && historyCount > 0) {
            // Gyro heading is counter-clockwise positive, camera tx is positive to the right,
            // hence the minus.
            double then = headingAt(now - (long) (ageSeconds * 1e9));
            aimHeading = AngleUnit.normalizeRadians(then - Math.toRadians(errorDegrees));
            hasAim = true;
        } else {
            hasAim = false;
        }

        if (Double.isFinite(distance)) {
            // Light smoothing while we keep seeing it; a fresh sighting starts clean.
            distanceInches = wasRecent && Double.isFinite(distanceInches)
                    ? distanceInches + 0.3 * (distance - distanceInches) : distance;
        } else if (!wasRecent) {
            distanceInches = Double.NaN;
        }
    }

    /**
     * Turn power that points the robot at the hive: positive = turn right, same as the right
     * stick. Returns NaN when there is nothing to aim at (never saw the hive, saw it too long
     * ago, or no gyro), and then the driver's stick should be used instead.
     */
    public double rotatePower(double heading, double turnRate, boolean headingOk) {
        return rotatePower(heading, turnRate, headingOk, System.nanoTime());
    }

    double rotatePower(double heading, double turnRate, boolean headingOk, long now) {
        aimed = false;
        double dt = lastControlNanos == 0 ? 0 : Math.min(0.1, (now - lastControlNanos) / 1e9);
        lastControlNanos = now;
        if (lastSeenNanos == 0 || !headingOk || !hasAim) {
            wasAimed = false;
            stallPush = 0;
            return Double.NaN;
        }
        if ((now - lastSeenNanos) / 1e9 > HOLD_SECONDS) {
            hasAim = false;
            wasAimed = false;
            stallPush = 0;
            return Double.NaN;
        }

        double error = AngleUnit.normalizeRadians(aimHeading - heading);
        double errorDegreesNow = Math.toDegrees(Math.abs(error));
        double rate = Math.abs(turnRate);
        aimed = errorDegreesNow <= (wasAimed ? STILL_AIMED_DEGREES : AIMED_DEGREES)
                && rate <= (wasAimed ? STILL_AIMED_TURN_RATE : AIMED_TURN_RATE);
        wasAimed = aimed;

        if (errorDegreesNow <= REST_DEGREES && rate <= STALLED_TURN_RATE) {
            stallPush = 0;
            return 0;
        }
        if (stallPush * error < 0) stallPush = 0;
        if (rate < STALLED_TURN_RATE) {
            stallPush += Math.signum(error) * STALL_PUSH_PER_SECOND * dt;
            stallPush = Math.max(-MAX_STALL_PUSH, Math.min(MAX_STALL_PUSH, stallPush));
        }
        double push = Math.signum(error) * MIN_TURN_POWER * Math.min(1.0, errorDegreesNow / 3.0);
        double turnLeft = KP * error - KD * turnRate + push + stallPush;
        return -Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnLeft));
    }

    /** Call when the driver lets go of the aim button. */
    public void release() {
        aimed = false;
        wasAimed = false;
        stallPush = 0;
        lastControlNanos = 0;
    }

    /** True while rotatePower() has the robot pointing at the hive and nearly still. */
    public boolean isAimed() {
        return aimed;
    }

    /** True when there is a fresh enough aim heading to turn to (used by autonomous). */
    public boolean hasAim() {
        return hasAim && lastSeenNanos != 0 && System.nanoTime() - lastSeenNanos <= HOLD_SECONDS * 1e9;
    }

    /** The heading to face, in the same gyro frame that was passed to update(). */
    public double aimHeading() {
        return aimHeading;
    }

    public boolean seesHive() {
        return lastSeenNanos != 0 && System.nanoTime() - lastSeenNanos <= SEEN_SECONDS * 1e9;
    }

    /**
     * Roughly how far the up cell is, in inches, or NaN if unknown. It is worked out from how
     * far apart the tags look, so it needs at least two tags in view and it does NOT depend
     * on any Limelight 3D setting. It is not tape-measure exact (it reads long from the
     * side), but it is repeatable, which is all the saved shots need.
     */
    public double distanceInches() {
        return distanceInches(System.nanoTime());
    }

    double distanceInches(long now) {
        boolean recent = lastSeenNanos != 0 && now - lastSeenNanos <= DISTANCE_SECONDS * 1e9;
        return recent ? distanceInches : Double.NaN;
    }

    /** What the camera sees right now, for the autonomous telemetry. */
    public String view() {
        if (limelight == null) return "OFF";
        if (!limelight.isConnected()) return "not answering (still starting, or unplugged)";
        if (!seesHive()) return "hive not in view (" + (tagsInView.isEmpty() ? "no tags, pipeline " + pipelineInUse : "tags " + tagsInView) + ")";
        double distance = distanceInches();
        String cellNote = aimingAtDownCell ? ", only the DOWN cell" : "";
        return (Double.isFinite(distance) ? String.format("sees the hive, %.0f in away", distance) : "sees the hive") + cellNote;
    }

    /** One line for the Driver Station saying what the aim is doing, in words a driver can act on. */
    public String status(boolean aiming) {
        if (limelight == null) return "OFF - no limelight in the robot config";
        if (!limelight.isConnected()) return "camera not answering (still starting, or unplugged)";
        if (!seesHive() && !(aiming && aimed)) {
            String others = tagsInView.isEmpty() ? "no tags in view, pipeline " + pipelineInUse : "only sees tags " + tagsInView;
            return aiming ? "can't see the " + allianceName() + " hive (" + others + ") - turn toward it"
                    : "hive not in view (" + others + ")";
        }
        if (aiming && !hasAim) return "needs the Pinpoint heading - aim with the stick";
        String cellNote = aimingAtDownCell ? " (only the DOWN cell in view)" : "";
        if (aiming) return (aimed ? "AIMED" : "aiming...") + cellNote;
        return String.format("sees the hive, %.0f deg off%s (hold LEFT TRIGGER to aim)", errorDegrees, cellNote);
    }

    boolean aimingAtDownCell() {
        return aimingAtDownCell;
    }

    String tagsInView() {
        return tagsInView;
    }

    /** How far off the hive was in the last picture, degrees, positive = to the right. */
    double offDegrees() {
        return errorDegrees;
    }

    public static String allianceName() {
        return blueAlliance ? "BLUE" : "RED";
    }

    void noteHeading(long now, double heading) {
        historyNanos[historyNext] = now;
        historyHeading[historyNext] = heading;
        historyNext = (historyNext + 1) % HISTORY;
        if (historyCount < HISTORY) historyCount++;
    }

    /** The newest remembered heading that is not newer than the given time. */
    private double headingAt(long nanos) {
        int newest = (historyNext - 1 + HISTORY) % HISTORY;
        int index = newest;
        for (int i = 0; i < historyCount; i++) {
            index = (newest - i + HISTORY) % HISTORY;
            if (historyNanos[index] <= nanos) break;
        }
        return historyHeading[index];
    }
}
