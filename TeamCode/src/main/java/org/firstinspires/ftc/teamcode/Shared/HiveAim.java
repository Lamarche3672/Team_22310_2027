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
    private static final double UP_CELL_MEMORY_SECONDS = 30.0;
    /** A one-tag picture is stepped to the sticker's middle using a scale no older than this. */
    private static final double SCALE_MEMORY_SECONDS = 2.0;

    // ---- things to flip if the real robot disagrees
    /** Make this -1 if the robot turns AWAY from the hive (camera mounted upside down). */
    public static final double TX_SIGN = 1.0;
    /** Shots land a little left or right every time? Positive aims further right, in degrees. */
    public static final double AIM_OFFSET_DEGREES = 0;

    // ---- the turn controller: power = KP x error - KD x turn rate (error in radians, rate in
    // radians per second), up to full power. KD is the brake: it is what stops the robot
    // sailing past the hive. Want it snappier? Raise both together (4.5 and 0.45). Wobbles?
    // Lower both (2.8 and 0.35).
    public static final double KP = 3.5;
    public static final double KD = 0.40;
    public static final double MAX_TURN_POWER = 1.0;

    // ---- the friction push. KP x error gets tiny as the error gets small, and a mecanum
    // drive needs a fair bit of power before it turns at all (more on a heavy robot, more on
    // sticky tiles). On its own, the controller above gets most of the way round and then
    // crawls the last several degrees. That crawl is what made the aim feel slow.
    // So: the robot should always be closing in at a rate that fits how far off it still is.
    // Whenever it is slower than that, extra power builds up until it is not. Nothing here
    // needs to know how much friction the robot has; it finds out by pushing.
    /** Further out than PUSH_FAR_DEGREES the robot should close at least this fast: 3 x error per second. */
    public static final double CLOSING_RATE_PER_RADIAN = 3.0;
    public static final double PUSH_PER_SECOND = 1.5;
    /** Closing at twice the wanted rate: the push has done its job, take it off again (per second). */
    public static final double PUSH_EASE_PER_SECOND = 3.0;
    public static final double MAX_PUSH = 0.45;
    /** Closer than this the push only builds if the robot is properly stuck, or it would hunt. */
    public static final double PUSH_FAR_DEGREES = 2.0;
    /** The push fades out over the last degree and a half so it cannot kick the robot past the target. */
    public static final double PUSH_FADE_DEGREES = 1.5;
    // How long the robot has to be too slow before the push starts to build. Long enough that
    // the first moment of every turn (wheels not up to speed yet) does not count.
    public static final double SLOW_SECONDS = 0.04;
    public static final double STUCK_SECONDS = 0.08;
    public static final double STALLED_TURN_RATE = 0.1;
    /** This close and this still: stop correcting. Chasing the last fraction of a degree only makes it hunt. */
    public static final double REST_DEGREES = 0.7;

    // ---- the safety net. If the robot swings through the target and out the other side by
    // more than SWING_DEGREES, the gains are too hot for this robot (or the loop is running
    // slow). Each swing like that turns KP and KD down a step until it settles. The Driver
    // Station says so when it happens: that is the cue to lower KP and KD for good.
    public static final double SWING_DEGREES = 2.5;
    public static final double SWING_STEP = 0.7;
    public static final double SWING_FLOOR = 0.4;
    public static final double SWING_FORGET_SECONDS = 1.5;

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
    // A picture taken while the robot turned slower than TRUSTED_SPIN (radians per second) is
    // believed completely. From there the belief drops, down to DOUBTED_TRUST at DOUBTED_SPIN.
    private static final double TRUSTED_SPIN = 0.5;
    private static final double DOUBTED_SPIN = 2.5;
    private static final double DOUBTED_TRUST = 0.2;
    /** The turn rate at the moment of a picture is measured over this long before it. */
    private static final long SPIN_SAMPLE_NANOS = 40_000_000L;
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
    private boolean newTarget;
    // How many degrees one inch of sticker covers, with its sign (which way round the sticker
    // looks), kept per cell: the two stickers are at different distances and can be seen from
    // opposite sides.
    private final double[] degreesPerInch = {Double.NaN, Double.NaN};
    private final long[] degreesPerInchNanos = new long[2];

    private boolean aimed;
    private boolean wasAimed;
    private long lastControlNanos;
    // The friction push: how much extra power is on right now (always toward the target), and
    // how long the robot has been closing too slowly.
    private double push;
    private double slowSeconds;
    // Which side of the target we are on (+1 = target is to the left), and how far out we got
    // on this side. Used to spot the robot swinging back and forth through the target.
    private int side;
    private boolean passedTarget;
    private double furthestThisSide;
    private int swings;
    private long lastSwingNanos;
    // How long the last lock took, for the Driver Station.
    private long aimStartNanos;
    private double lockSeconds = Double.NaN;

    /** Call at the start of a match (autonomous does): the hive is back in its starting position. */
    public static void matchStart() {
        upCell = START_UP_CELL;
        // Counts as a sighting: at the start of a match we KNOW which cell is up.
        upCellSeenNanos = System.nanoTime();
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
        if (result == null) return;
        if (!result.isValid()) {
            // The camera is answering but sees no tags at all. Say so on the screen instead
            // of leaving the last list of tags up.
            tagsInView = "";
            pipelineInUse = result.getPipelineIndex();
            return;
        }

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
            // Older Limelight firmware does not send that number, and the SDK then hands back
            // exactly 0 for every tag. A real measurement is never exactly 0, so fall back to
            // the ordinary one rather than believe the hive is dead ahead.
            if (tx[i] == 0.0) tx[i] = tag.getTargetXDegrees();
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
                degreesPerInch[target] = slope;
                degreesPerInchNanos[target] = now;
                centre = meanTx - slope * meanOffset;
                distance = 1.0 / Math.tan(Math.toRadians(Math.abs(slope)));
            }
        } else if (Double.isFinite(degreesPerInch[target])
                && now - degreesPerInchNanos[target] <= SCALE_MEMORY_SECONDS * 1e9) {
            // Only one tag: use this sticker's last known scale to step from that tag to the
            // middle. If the scale is old (we may have moved a long way) aim at the tag itself.
            centre = meanTx - degreesPerInch[target] * meanOffset;
        }
        // A distance to the DOWN cell would pick the wrong shot speed (it is about two feet
        // further away), so only the up cell's distance is passed on.
        if (aimingAtDownCell) distance = Double.NaN;
        // Switched to the other cell (a tip): its distance is a different number, so do not
        // blend it with the old one.
        if (target != lastTarget) {
            distanceInches = Double.NaN;
            newTarget = true;
        }
        lastTarget = target;
        onFrame(centre, distance, ageSeconds, now, headingOk);
    }

    /** One new camera picture: the hive is txDegrees to the right, and the picture is ageSeconds old. */
    void onFrame(double txDegrees, double distance, double ageSeconds, long now, boolean headingOk) {
        if (!Double.isFinite(txDegrees)) return;
        boolean wasRecent = lastSeenNanos != 0 && now - lastSeenNanos <= DISTANCE_SECONDS * 1e9;
        long previousSeenNanos = lastSeenNanos;
        // Adding the offset makes the hive look further right than it is, so the robot turns
        // further right.
        errorDegrees = TX_SIGN * txDegrees + AIM_OFFSET_DEGREES;
        lastSeenNanos = now;

        if (headingOk && historyCount > 0) {
            // Gyro heading is counter-clockwise positive, camera tx is positive to the right,
            // hence the minus.
            long taken = now - (long) (ageSeconds * 1e9);
            double then = headingAt(taken);
            double fresh = AngleUnit.normalizeRadians(then - Math.toRadians(errorDegrees));

            // How much to believe this picture. "Where was the robot pointing when it was
            // taken" is only as good as our guess of the picture's age, and in the middle of
            // a fast turn a guess that is 30 ms out is 5 degrees out. So a picture taken
            // while spinning only nudges the aim; one taken while nearly still replaces it.
            // (The hive does not move, so the aim heading only really changes when the robot
            // drives somewhere else, and that is slow next to a spin.)
            double spin = Math.abs(AngleUnit.normalizeRadians(then - headingAt(taken - SPIN_SAMPLE_NANOS)))
                    / (SPIN_SAMPLE_NANOS / 1e9);
            double trust = 1.0 - (spin - TRUSTED_SPIN) / (DOUBTED_SPIN - TRUSTED_SPIN) * (1.0 - DOUBTED_TRUST);
            trust = Math.max(DOUBTED_TRUST, Math.min(1.0, trust));
            // Nothing recent to nudge, or a different cell than last time: start clean.
            boolean startClean = !hasAim || newTarget || now - previousSeenNanos > SEEN_SECONDS * 1e9;
            aimHeading = startClean ? fresh
                    : AngleUnit.normalizeRadians(aimHeading + trust * AngleUnit.normalizeRadians(fresh - aimHeading));
            newTarget = false;
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
        if (lastControlNanos == 0) {
            // First loop of this press of the aim button.
            aimStartNanos = now;
            lockSeconds = Double.NaN;
        }
        double dt = lastControlNanos == 0 ? 0 : Math.min(0.1, (now - lastControlNanos) / 1e9);
        lastControlNanos = now;
        if (lastSeenNanos == 0 || !headingOk || !hasAim) {
            forgetTurn();
            return Double.NaN;
        }
        if ((now - lastSeenNanos) / 1e9 > HOLD_SECONDS) {
            hasAim = false;
            forgetTurn();
            return Double.NaN;
        }

        // error is how far we still have to turn, positive = to the left (counter-clockwise).
        double error = AngleUnit.normalizeRadians(aimHeading - heading);
        double errorDegreesNow = Math.toDegrees(Math.abs(error));
        double rate = Math.abs(turnRate);
        aimed = errorDegreesNow <= (wasAimed ? STILL_AIMED_DEGREES : AIMED_DEGREES)
                && rate <= (wasAimed ? STILL_AIMED_TURN_RATE : AIMED_TURN_RATE);
        if (aimed && !wasAimed && Double.isNaN(lockSeconds)) lockSeconds = (now - aimStartNanos) / 1e9;
        wasAimed = aimed;

        int sideNow = error >= 0 ? 1 : -1;
        if (side != 0 && sideNow != side) {
            // Went through the target. Whatever push was on belonged to the other direction.
            push = 0;
            // The first time through, "how far out we got" is just where the turn started
            // from. After that it is how far the robot overshot, and a big one is a swing.
            if (passedTarget && furthestThisSide >= SWING_DEGREES) {
                swings++;
                lastSwingNanos = now;
            }
            passedTarget = true;
            furthestThisSide = 0;
        }
        side = sideNow;
        furthestThisSide = Math.max(furthestThisSide, errorDegreesNow);
        if (swings > 0 && now - lastSwingNanos > SWING_FORGET_SECONDS * 1e9) swings = 0;
        double calm = Math.max(SWING_FLOOR, Math.pow(SWING_STEP, swings));

        if (errorDegreesNow <= REST_DEGREES && rate <= STALLED_TURN_RATE) {
            push = 0;
            slowSeconds = 0;
            return 0;
        }

        // How fast we are closing on the target, and how fast we ought to be.
        double closing = turnRate * sideNow;
        boolean far = errorDegreesNow > PUSH_FAR_DEGREES;
        double wanted = Math.max(STALLED_TURN_RATE, far ? CLOSING_RATE_PER_RADIAN * Math.abs(error) : 0);
        if (closing < wanted) {
            slowSeconds += dt;
            if (slowSeconds >= (far ? SLOW_SECONDS : STUCK_SECONDS)) push += PUSH_PER_SECOND * dt;
        } else {
            slowSeconds = 0;
            if (closing > 2 * wanted) push -= PUSH_EASE_PER_SECOND * dt;
        }
        push = Math.max(0, Math.min(MAX_PUSH, push));

        double turnLeft = calm * (KP * error - KD * turnRate)
                + sideNow * push * Math.min(1.0, errorDegreesNow / PUSH_FADE_DEGREES);
        return -Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnLeft));
    }

    /** Clears everything the turn controller remembers from one loop to the next. */
    private void forgetTurn() {
        wasAimed = false;
        push = 0;
        slowSeconds = 0;
        side = 0;
        passedTarget = false;
        furthestThisSide = 0;
        swings = 0;
    }

    /** Call when the driver lets go of the aim button. */
    public void release() {
        aimed = false;
        forgetTurn();
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
        if (aiming) {
            // The time it took is there so a change to KP / KD can be judged by a number.
            String took = Double.isNaN(lockSeconds) ? "" : String.format(" (locked in %.2f s)", lockSeconds);
            String wobble = swings > 0 ? " - WOBBLING, lower KP and KD in HiveAim" : "";
            return (aimed ? "AIMED" + took : "aiming...") + cellNote + wobble;
        }
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

    /**
     * Where the robot was pointing at the given time, worked out from the two remembered
     * headings either side of it. Taking the nearest older one instead would be up to a whole
     * loop out of date, which is a couple of degrees in the middle of a fast turn.
     */
    private double headingAt(long nanos) {
        int newest = (historyNext - 1 + HISTORY) % HISTORY;
        int older = newest;
        int newer = -1;
        for (int i = 0; i < historyCount; i++) {
            older = (newest - i + HISTORY) % HISTORY;
            if (historyNanos[older] <= nanos) break;
            newer = older;
        }
        // Older than everything we remember, or newer than the newest: use the closest one.
        if (newer < 0 || historyNanos[older] > nanos) return historyHeading[older];
        long gap = historyNanos[newer] - historyNanos[older];
        if (gap <= 0) return historyHeading[older];
        double part = (double) (nanos - historyNanos[older]) / gap;
        // Go the short way round, so 179 to -179 degrees is a step of 2, not 358.
        double step = AngleUnit.normalizeRadians(historyHeading[newer] - historyHeading[older]);
        return AngleUnit.normalizeRadians(historyHeading[older] + part * step);
    }
}
