package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * AimController - smooth auto-aim using the Limelight + the Control Hub's IMU (gyro).
 *
 * WHY: the camera picture is ~20-30 ms old and only updates ~30-90 times a second, so steering
 * straight off the camera chases where the tag USED to be -> wobble.
 * FIX: the camera says "the HIVE is at heading X". The gyro (fast, no delay) turns the robot to X
 * and holds it, even if the tag flickers out for a moment.
 *
 * Turn power = PDF on heading error, battery-compensated:
 *   P = AIM_KP x degrees off      D = AIM_KD x turn speed (brakes, stops overshoot)
 *   F = AIM_KS (just enough to beat friction), only when not locked
 *
 * Also gives heading() for field-centric driving.
 *
 * CAMERA-ONLY MODE (USE_GYRO = false, or the IMU failed to start): it steers straight off the
 * camera. With no gyro, the turn speed for the D (braking) part is worked out from how fast the
 * tag moves across the picture from one camera frame to the next. Two safeguards:
 *   - if the tag has been out of view for AIM_LOST_SEC it stops steering (the stick works again)
 *     instead of carrying on turning with an old reading;
 *   - it only reports LOCKED once it has stayed on target for AIM_SETTLE_SEC, so swinging
 *     THROUGH the target on the way past doesn't count as aimed.
 */
public class AimController {

    private static final int HISTORY = 128;

    private IMU imu;
    private boolean imuOk = false;
    private String imuError = "";

    // Live-tunable gains (start from zack code SETTINGS; Aim Tuner changes them).
    // Set in the constructor: the gyro numbers with a gyro, the gentler AIM_CAM_ numbers without.
    public double kP = AxonTeleOp.AIM_KP;
    public double kD = AxonTeleOp.AIM_KD;
    public double kS = AxonTeleOp.AIM_KS;
    public double maxPower = AxonTeleOp.AIM_MAX_POWER;

    private static final double MAX_CAMERA_RATE = 200;   // deg/s: ignore anything faster from the camera

    private final double[] histTime = new double[HISTORY];
    private final double[] histHeading = new double[HISTORY];
    private int histIndex = 0, histCount = 0;

    private double heading = 0;      // degrees, + = robot turned LEFT
    private double yawRate = 0;      // degrees per second, + = turning LEFT
    private boolean hasAim = false;
    private double aimHeading = 0;
    private double cameraError = 0;  // used only in camera-only fallback
    private boolean locked = false;
    private final ElapsedTime sinceCamera = new ElapsedTime();   // time since the last camera reading
    private final ElapsedTime sinceFrame = new ElapsedTime();    // time since the last NEW camera frame
    private final ElapsedTime lockedFor = new ElapsedTime();     // how long it has been locked

    public AimController(HardwareMap hardwareMap) {
        if (!AxonTeleOp.USE_GYRO) {          // gyro switched off in SETTINGS: camera-only aiming
            imuOk = false;
            imuError = "turned off (USE_GYRO = false)";
        } else {
            try {
                imu = hardwareMap.get(IMU.class, AxonTeleOp.IMU_NAME);
                imu.initialize(new IMU.Parameters(
                        new RevHubOrientationOnRobot(AxonTeleOp.HUB_LOGO, AxonTeleOp.HUB_USB)));
                imu.resetYaw();
                imuOk = true;
            } catch (Exception e) {
                imuOk = false;
                imuError = e.getMessage();
            }
        }
        if (!imuOk) {                        // camera-only: use the gentler camera numbers
            kP = AxonTeleOp.AIM_CAM_KP;
            kD = AxonTeleOp.AIM_CAM_KD;
            maxPower = AxonTeleOp.AIM_CAM_MAX_POWER;
        }
    }

    public boolean imuOk() { return imuOk; }
    public String imuError() { return imuError; }

    /** Make the direction the robot faces right now = heading 0. */
    public void resetHeading() {
        if (imuOk) imu.resetYaw();
        histCount = 0;
        clearTarget();
    }

    /** Call FIRST every loop. */
    public void update() {
        if (!imuOk) return;
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        histTime[histIndex] = System.nanoTime() / 1e9;
        histHeading[histIndex] = heading;
        histIndex = (histIndex + 1) % HISTORY;
        if (histCount < HISTORY) histCount++;
    }

    /**
     * Give it a fresh camera reading.
     * @param errorDeg  how far RIGHT (+) or LEFT (-) of center the HIVE is in the picture
     * @param frameAgeMs how old that picture is
     */
    public void cameraUpdate(double errorDeg, double frameAgeMs) {
        if (!imuOk) {
            // No gyro: get the turn speed from the camera. A different number = a NEW frame (the
            // same frame is handed over every loop until the next one arrives). Turning LEFT makes
            // the tag slide to the RIGHT in the picture, so turn speed = how fast the error grows.
            if (!hasAim) {
                yawRate = 0;                       // first reading of this aim: no speed known yet
                sinceFrame.reset();
            } else if (errorDeg != cameraError) {
                double dt = sinceFrame.seconds();
                sinceFrame.reset();
                if (dt > 0.004 && dt < 0.15) {
                    double seen = Math.max(-MAX_CAMERA_RATE, Math.min(MAX_CAMERA_RATE, (errorDeg - cameraError) / dt));
                    yawRate += 0.6 * (seen - yawRate);                            // lightly smoothed
                } else {
                    yawRate = 0;                   // frames too far apart to trust
                }
            } else if (sinceFrame.seconds() > 0.15) {
                yawRate = 0;                       // camera has stopped sending new frames
            }
        }
        cameraError = errorDeg;
        sinceCamera.reset();
        if (imuOk) {
            // Where was the robot pointing when the picture was TAKEN? Aim from there.
            double headingThen = headingAt(System.nanoTime() / 1e9 - frameAgeMs / 1000.0);
            aimHeading = AngleUnit.normalizeDegrees(headingThen - errorDeg);
        }
        hasAim = true;
    }

    /** Forget the target (call when the aim button is released). */
    public void clearTarget() {
        hasAim = false;
        locked = false;
        if (!imuOk) yawRate = 0;
    }

    public boolean hasAim() {
        // Camera-only: an old reading is worthless without a gyro to hold the heading. If the tag has
        // been out of view for a moment, stop steering on it.
        if (hasAim && !imuOk && sinceCamera.seconds() > AxonTeleOp.AIM_LOST_SEC) clearTarget();
        return hasAim;
    }

    /** Degrees to turn to face the HIVE. + = turn LEFT. */
    public double error() {
        if (!hasAim) return 0;
        return imuOk ? AngleUnit.normalizeDegrees(aimHeading - heading) : -cameraError;
    }

    /** Turn power for the drivetrain (+ = turn LEFT), already battery-compensated. */
    public double turnPower(Battery battery) {
        if (!hasAim()) return 0;
        double err = error();
        double tol = AxonTeleOp.AIM_TOLERANCE;

        // Lock when close and nearly stopped; unlock only if it drifts well off (no chatter).
        if (locked && Math.abs(err) > 2 * tol) locked = false;
        if (!locked && Math.abs(err) < tol && Math.abs(yawRate) < 20) {
            locked = true;
            lockedFor.reset();
        }

        double p = kP * err - kD * yawRate;
        if (!locked) p += Math.copySign(kS, err);
        p *= battery.scale();
        return Math.max(-maxPower, Math.min(maxPower, p));
    }

    /** True when aimed. Camera-only: it must also have STAYED on target for AIM_SETTLE_SEC. */
    public boolean isLocked() {
        if (!hasAim() || !locked) return false;
        return imuOk || lockedFor.seconds() >= AxonTeleOp.AIM_SETTLE_SEC;
    }
    public double heading() { return heading; }
    public double yawRate() { return yawRate; }
    public double aimHeading() { return aimHeading; }

    private double headingAt(double time) {
        if (histCount == 0) return heading;
        double best = heading, bestDiff = Double.MAX_VALUE;
        for (int i = 0; i < histCount; i++) {
            double d = Math.abs(histTime[i] - time);
            if (d < bestDiff) { bestDiff = d; best = histHeading[i]; }
        }
        return best;
    }
}
