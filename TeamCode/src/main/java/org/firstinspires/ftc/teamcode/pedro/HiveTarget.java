package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/*
 * HiveTarget - finds the UPWARD-FACING CELL of a HIVE with the Limelight.
 *
 * BIOBUZZ rules (Competition Manual V1, sections 9.6, 9.9, 10.5.1, G417):
 *   - Each alliance has ONE HIVE with TWO CELLS. Only the upward-facing CELL can be scored
 *     (and deliberately launching at the downward CELL can be a G417 foul).
 *   - The HIVE flips after every TIP, so the target CELL changes during the match.
 *   - Each CELL has a 4-tag cluster (36h11, 3.25 in) on its BOTTOM, facing down:
 *       RED HIVE : 30-33 (far side), 34-37 (audience side)
 *       BLUE HIVE: 38-41 (audience side), 42-45 (far side)
 *     Tag centers sit at -6.50, -2.75, +2.75, +6.50 in from the cluster center.
 *   - The HIVE tilts 30 deg on a pivot 43.95 in above the tiles, so the upward CELL's
 *     tags are roughly 15 in HIGHER than the downward CELL's tags.
 *
 * HOW IT PICKS: for each HIVE it can see, it works out how high each cluster is.
 *   Both clusters visible -> the HIGHER one is the upward CELL.
 *   Only one visible      -> it counts as the upward CELL only if it is above UP_CELL_MIN_HEIGHT_IN.
 * Height uses CAMERA_PITCH_DEG and CAMERA_HEIGHT_IN from zack code SETTINGS - measure them.
 *
 * Every loop call update(). Then read:
 *   hasTarget()   - true if an upward CELL was seen recently
 *   aimErrorDeg() - degrees the target is RIGHT (+) or LEFT (-) of the camera
 *   distanceIn()  - inches from the camera to the target cluster center
 *   frameAgeMs()  - how old the camera picture is (AimController uses it to cancel camera delay)
 *
 * Not a TeleOp - it's a helper used by "zack code", "zack blue", "zack red" and the tuners.
 */
public class HiveTarget {

    public enum Alliance { ANY, RED, BLUE }

    static final int APRILTAG_PIPELINE = 0;       // pipeline slot set to AprilTag in the Limelight web page
    static final double METERS_TO_INCHES = 39.3701;
    static final double[] TAG_OFFSETS_IN = {-6.50, -2.75, 2.75, 6.50};
    static final long MAX_STALENESS_MS = 150;      // ignore frames older than this
    static final double HOLD_LAST_SECONDS = 0.5;   // keep the last target this long if it flickers out

    // Cluster bases: each HIVE is a pair of clusters.
    static final int[] RED_HIVE  = {30, 34};
    static final int[] BLUE_HIVE = {38, 42};

    private final Limelight3A limelight;
    private Alliance alliance = Alliance.ANY;

    private boolean seenThisFrame = false;
    private double aimErrorDeg = 0;
    private double distanceIn = 0;
    private int clusterBase = -1;
    private int tagsSeen = 0;
    private long lastSeenNanos = 0;
    private String debug = "not started";
    private String cells = "-";
    private double frameAgeMs = 0;

    /** One visible cluster, worked out from the camera picture. */
    private static class Cluster {
        int base;
        int tags;
        double aimDeg;
        double distIn = -1;        // -1 = no 3D pose
        double heightIn = Double.NaN;  // above the floor; NaN = unknown
    }

    public HiveTarget(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
    }

    public void setAlliance(Alliance a) { alliance = a; }
    public Alliance getAlliance() { return alliance; }

    public void stop() { limelight.stop(); }

    public void update() {
        seenThisFrame = false;
        LLResult result = limelight.getLatestResult();
        if (result == null) { debug = "no data from Limelight (not connected?)"; return; }
        if (result.getPipelineIndex() != APRILTAG_PIPELINE) {
            debug = "Limelight on pipeline " + result.getPipelineIndex() + ", need " + APRILTAG_PIPELINE;
            return;
        }
        if (result.getStaleness() > MAX_STALENESS_MS) {
            debug = "old data (" + result.getStaleness() + " ms) - camera frozen?";
            return;
        }
        if (!result.isValid()) { debug = "connected, no tag in view"; cells = "-"; return; }

        // How old this picture is: camera capture + processing + time since it arrived.
        double age = result.getCaptureLatency() + result.getTargetingLatency() + result.getStaleness();

        List<LLResultTypes.FiducialResult> all = result.getFiducialResults();
        StringBuilder ids = new StringBuilder();
        for (LLResultTypes.FiducialResult t : all) ids.append(t.getFiducialId()).append(' ');
        debug = "tags in view: " + (ids.length() == 0 ? "none" : ids.toString().trim());

        // Find the upward CELL of each allowed HIVE, then take the nearest one.
        StringBuilder cellText = new StringBuilder();
        Cluster best = null;
        if (alliance != Alliance.BLUE) best = closer(best, upwardCell(RED_HIVE, all, cellText));
        if (alliance != Alliance.RED)  best = closer(best, upwardCell(BLUE_HIVE, all, cellText));
        cells = cellText.length() == 0 ? "none of ours" : cellText.toString().trim();

        if (best != null) {
            aimErrorDeg = best.aimDeg;
            distanceIn = best.distIn;
            clusterBase = best.base;
            tagsSeen = best.tags;
            seenThisFrame = true;
            lastSeenNanos = System.nanoTime();
            frameAgeMs = age;
        }
    }

    /** Of one HIVE's two clusters, return the upward-facing CELL (or null if it can't be seen). */
    private Cluster upwardCell(int[] hive, List<LLResultTypes.FiducialResult> all, StringBuilder text) {
        Cluster a = cluster(hive[0], all);
        Cluster b = cluster(hive[1], all);
        Cluster up;
        if (a != null && b != null) {
            if (!Double.isNaN(a.heightIn) && !Double.isNaN(b.heightIn)) {
                up = a.heightIn >= b.heightIn ? a : b;   // both visible: the higher one is UP
            } else {
                up = null;                                // can't tell which is up without 3D
            }
        } else {
            Cluster only = (a != null) ? a : b;
            if (only == null) return null;
            // Only one visible: trust it only if it is clearly at "upward CELL" height.
            up = (!Double.isNaN(only.heightIn) && only.heightIn >= AxonTeleOp.UP_CELL_MIN_HEIGHT_IN) ? only : null;
        }
        for (Cluster c : new Cluster[]{a, b}) {
            if (c == null) continue;
            text.append(shortName(c.base))
                    .append(Double.isNaN(c.heightIn) ? " h=?" : String.format(" h=%.0fin", c.heightIn))
                    .append(c == up ? " UP  " : " down  ");
        }
        return up;
    }

    private static Cluster closer(Cluster x, Cluster y) {
        if (x == null) return y;
        if (y == null) return x;
        double dx = x.distIn > 0 ? x.distIn : 1e6;
        double dy = y.distIn > 0 ? y.distIn : 1e6;
        return dy < dx ? y : x;
    }

    /** Build one cluster from the tags in view (null if none of its tags are visible). */
    private Cluster cluster(int base, List<LLResultTypes.FiducialResult> all) {
        List<LLResultTypes.FiducialResult> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult t : all) {
            int id = t.getFiducialId();
            if (id >= base && id < base + 4) tags.add(t);
        }
        if (tags.isEmpty()) return null;

        Cluster c = new Cluster();
        c.base = base;
        c.tags = tags.size();

        // Camera space: x = right, y = down, z = forward (inches).
        List<double[]> pts = new ArrayList<>();
        List<Double> offs = new ArrayList<>();
        double txSum = 0;
        for (LLResultTypes.FiducialResult t : tags) {
            txSum += t.getTargetXDegrees();
            Pose3D pose = t.getTargetPoseCameraSpace();
            if (pose == null) continue;
            Position p = pose.getPosition();
            if (p.x == 0 && p.y == 0 && p.z == 0) continue;   // all-zero = no 3D for this tag
            pts.add(new double[]{p.x * METERS_TO_INCHES, p.y * METERS_TO_INCHES, p.z * METERS_TO_INCHES});
            offs.add(TAG_OFFSETS_IN[t.getFiducialId() - base]);
        }
        double txAvg = txSum / tags.size();
        if (pts.isEmpty()) {           // no 3D pose: can still aim, distance + height unknown
            c.aimDeg = txAvg;
            return c;
        }

        // Estimate the cluster CENTER (not just one tag), so aim doesn't shift when only some tags show.
        double[] center;
        if (pts.size() >= 2) {
            int ia = 0, ib = 0;
            for (int i = 0; i < offs.size(); i++) {
                if (offs.get(i) < offs.get(ia)) ia = i;
                if (offs.get(i) > offs.get(ib)) ib = i;
            }
            double span = offs.get(ib) - offs.get(ia);
            double[] pa = pts.get(ia), pb = pts.get(ib);
            double[] u = {(pb[0] - pa[0]) / span, (pb[1] - pa[1]) / span, (pb[2] - pa[2]) / span};
            double oa = offs.get(ia);
            center = new double[]{pa[0] - oa * u[0], pa[1] - oa * u[1], pa[2] - oa * u[2]};
            c.aimDeg = Math.toDegrees(Math.atan2(center[0], center[2]));
        } else {
            center = pts.get(0);       // only one tag: aim at that tag
            c.aimDeg = txAvg;
        }
        c.distIn = Math.sqrt(center[0] * center[0] + center[1] * center[1] + center[2] * center[2]);

        // Height above the floor: undo the camera's upward tilt.
        double pitch = Math.toRadians(AxonTeleOp.CAMERA_PITCH_DEG);
        double aboveCamera = -center[1] * Math.cos(pitch) + center[2] * Math.sin(pitch);
        c.heightIn = AxonTeleOp.CAMERA_HEIGHT_IN + aboveCamera;
        return c;
    }

    public boolean hasTarget() {
        return seenThisFrame || (System.nanoTime() - lastSeenNanos) / 1e9 < HOLD_LAST_SECONDS;
    }
    public boolean seenThisFrame() { return seenThisFrame; }
    public double aimErrorDeg() { return aimErrorDeg; }
    public double distanceIn() { return distanceIn; }
    public String debug() { return debug; }
    /** Which of our CELLS are visible, their heights, and which one counts as UP. */
    public String cells() { return cells; }
    /** How old the last camera picture was, in milliseconds (for delay-corrected aiming). */
    public double frameAgeMs() { return frameAgeMs; }

    /** Camera health line, e.g. "pipeline 0 (pipe_fiducial), 40 fps, 45C". */
    public String status() {
        LLStatus s = limelight.getStatus();
        if (s == null) return "no status";
        return String.format("pipeline %d (%s), %d fps, %.0fC",
                s.getPipelineIndex(), s.getPipelineType(), (int) s.getFps(), s.getTemp());
    }

    public int tagsSeen() { return seenThisFrame ? tagsSeen : 0; }

    public String clusterName() {
        switch (clusterBase) {
            case 30: return "RED far CELL (30-33)";
            case 34: return "RED audience CELL (34-37)";
            case 38: return "BLUE audience CELL (38-41)";
            case 42: return "BLUE far CELL (42-45)";
            default: return "none";
        }
    }

    private static String shortName(int base) {
        switch (base) {
            case 30: return "RED far";
            case 34: return "RED aud";
            case 38: return "BLUE aud";
            case 42: return "BLUE far";
            default: return "?";
        }
    }
}
