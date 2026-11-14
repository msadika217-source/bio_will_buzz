package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * "North Red Preload" - shoot the 4 preloaded Pollen, then park in the LOADING ZONE.
 *
 * WHAT IT DOES
 *   1. Flywheel starts spinning at SHOOT_RPM the moment you press START.
 *   2. Drives from START to SHOOT (a short move off the wall = LEAVE points).
 *   3. Aims at the RED HIVE's upward CELL with the Limelight. It turns by HOLDING a new heading with
 *      the Pinpoint (fast, no camera delay), the same idea as the gyro aim in the TeleOp.
 *   4. When it is aimed AND the flywheel is at speed: stopper OPEN (lights green) + intake 100%,
 *      exactly like D-pad UP + Square in the TeleOp. Stops after 4 balls, or after FEED_MAX_SEC.
 *   5. Stopper closed, intake and flywheel off, drives to PARK and holds there.
 *   Every step has a time limit, so it can never get stuck waiting.
 *
 * BEFORE EVERY MATCH (during INIT, on the Driver Hub):
 *   - "Limelight sees" must show the RED cell. "nothing" = the robot is not facing the HIVE: fix the
 *     start position, or the shot will be blind.
 *   - "Battery" 12.5 V or more.
 *
 * Uses the same Flywheel, Stopper, HiveTarget and SETTINGS as the TeleOp (AxonTeleOp.java).
 * No buttons: robots can't be driven during AUTO.
 */
@Autonomous(name = "North Red Preload", group = "Autonomous")
public class NorthRedPreloadAuto extends LinearOpMode {

    // ======================= SETTINGS =======================
    // Field positions in inches and degrees, in the same field coordinates as your friend's
    // NorthRedAutoPaths. Pedro: heading 90 = facing up the field, 270 = facing down it.
    // START: her start point, squared up (her 260 deg -> 270 deg) and centred on the tile (60.5 -> 60)
    // so the robot can be placed the same way every match: BACK flat against the wall, centred on the
    // 3rd tile. The Limelight aim takes care of the last few degrees.
    static final double START_X = 60.0, START_Y = 134.6362, START_HEADING = 270;
    // SHOOT: her shoot point (8.6 in off the wall). It should be the same distance from the HIVE you
    // shoot from in TeleOp, because SHOOT_RPM is one fixed speed - check "Target ... in" on the screen.
    static final double SHOOT_X = 60.0, SHOOT_Y = 126.0225, SHOOT_HEADING = 270;
    // PARK: the middle of the RED LOADING ZONE as worked out from your friend's four park points
    // (against the red wall, in the 5th row of tiles counting from the audience: y 96 to 120). Her
    // own North Red point (9.9, 119.8) only caught the edge of it. CHECK against the tape on the
    // real field - the robot only has to be PARTLY inside. Heading stays the same as when shooting.
    static final double PARK_X = 10.5, PARK_Y = 108.0;

    static final boolean PARK = true;             // false = stay at SHOOT after the volley

    // Time limits (seconds). Each step moves on when it is done OR when its time runs out.
    static final double DRIVE_TIMEOUT     = 3.0;  // START -> SHOOT
    static final double SETTLE_SPEED_IN_S = 2.0;  // "stopped" = moving slower than this
    static final double SETTLE_DIST_IN    = 1.5;  // "arrived" = this close to SHOOT
    static final double AIM_TIMEOUT       = 2.0;  // aiming: shoot anyway after this long
    static final double NO_TAG_SEC        = 1.0;  // no tag at all for this long -> shoot without aiming
    static final double AIM_WRONG_WAY_DEG = 10;   // aim error grows this much past the first reading = turning the wrong way -> stop aiming
    static final double SPINUP_TIMEOUT    = 4.0;  // from START: shoot even if the flywheel isn't "at speed" yet
    static final double FEED_MAX_SEC      = 3.0;  // most time the intake feeds for
    static final double FEED_AFTER_4_SEC  = 0.4;  // after the 4th ball is seen, keep feeding this long (in case one is stuck)
    static final double PARK_TIMEOUT      = 5.0;
    // ========================================================

    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(START_X, START_Y, START_HEADING);
    private final Pose shoot = poseFactory.of(SHOOT_X, SHOOT_Y, SHOOT_HEADING);

    private Follower follower;
    private Flywheel flywheel;
    private Battery battery;
    private Stopper stopper;
    private HiveTarget hive;
    private DcMotorEx intake;
    private ServoImplEx flowerArm;

    private enum Step { DRIVE_TO_SHOOT, AIM, SHOOT, PARK, DONE }
    private Step step = Step.DRIVE_TO_SHOOT;
    private final ElapsedTime stepTimer = new ElapsedTime();
    private final ElapsedTime autoTimer = new ElapsedTime();

    // Aim
    private double holdHeading;                 // radians: the heading it is holding at SHOOT
    private boolean aimEnabled = true;
    private boolean everSawTag = false;
    private final ElapsedTime onTarget = new ElapsedTime();
    private boolean wasOnTarget = false;
    private double lastAimError = 0;            // degrees, + = HIVE is to the RIGHT
    private double firstAimError = 0;           // degrees, the first reading at SHOOT
    private String aimText = "-";

    // Shooting
    private boolean flywheelOn = false;
    private int lastBalls = 0;
    private final ElapsedTime sinceBall = new ElapsedTime();
    private String shootReason = "";

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        follower = Constants.create(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);
        stopper = new Stopper(hardwareMap);          // lights RED
        hive = new HiveTarget(hardwareMap, AxonTeleOp.LIMELIGHT_NAME);
        hive.setAlliance(HiveTarget.Alliance.RED);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flowerArm = hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flowerArm.setPosition(AxonTeleOp.ARM_UP);   // stowed, inside 18 in

        follower.setPose(start);
        telemetry.setMsTransmissionInterval(50);

        // ---- INIT: show the checks, don't move ----
        while (!isStarted() && !isStopRequested()) {
            follower.update();                       // reads the Pinpoint (motors stay off)
            hive.update();
            battery.update();
            telemetry.addLine("North Red Preload: shoot 4 preloads, then park");
            telemetry.addData("Limelight sees", hive.seenThisFrame()
                    ? String.format("%s at %.0f in, %.1f deg off", hive.clusterName(), hive.distanceIn(), hive.aimErrorDeg())
                    : "NOTHING <- robot not facing the RED HIVE? check the start position");
            telemetry.addData("Limelight", hive.debug());
            telemetry.addData("Pose", poseText());
            telemetry.addData("Battery", "%.2f V %s", battery.volts(), battery.volts() < 12.5 ? " <- CHARGE ME" : "");
            telemetry.update();
        }
        if (isStopRequested()) { hive.stop(); return; }

        // ---- START ----
        autoTimer.reset();
        follower.setPose(start);
        stopper.setOpen(false);
        flywheelOn = true;                           // spin up while driving
        holdHeading = shoot.heading();
        follower.follow(Paths.line(start, shoot).linear(start, shoot));
        beginStep(Step.DRIVE_TO_SHOOT);

        while (opModeIsActive()) {
            battery.update();
            hive.update();

            switch (step) {
                case DRIVE_TO_SHOOT: {
                    boolean arrived = follower.atParametricEnd()
                            && follower.pose().distance(shoot) < SETTLE_DIST_IN
                            && speed() < SETTLE_SPEED_IN_S;
                    if (arrived || stepTimer.seconds() > DRIVE_TIMEOUT) {
                        follower.hold(shoot);            // stay here while aiming and shooting
                        beginStep(Step.AIM);
                    }
                    break;
                }
                case AIM: {
                    boolean aimed = updateAim();
                    boolean atSpeed = flywheel.atSpeed();
                    boolean spinupOver = autoTimer.seconds() > SPINUP_TIMEOUT;
                    boolean noTag = !everSawTag && stepTimer.seconds() > NO_TAG_SEC;
                    boolean aimOver = stepTimer.seconds() > AIM_TIMEOUT;

                    boolean aimOk = aimed || noTag || aimOver || !aimEnabled;
                    if (aimOk && (atSpeed || spinupOver)) {
                        shootReason = (aimed ? "aimed" : noTag ? "NO TAG - not aimed" : !aimEnabled ? "aim turned off (wrong way)" : "aim timed out")
                                + (atSpeed ? ", at speed" : ", flywheel NOT at speed (timed out)");
                        flywheel.resetBallLog();
                        lastBalls = 0;
                        sinceBall.reset();
                        stopper.setOpen(true);           // lights GREEN
                        intake.setPower(1.0);            // same as holding Square
                        beginStep(Step.SHOOT);
                    }
                    break;
                }
                case SHOOT: {
                    updateAim();                         // keep facing the HIVE between balls
                    intake.setPower(1.0);
                    int balls = flywheel.ballsLogged();
                    if (balls != lastBalls) { lastBalls = balls; sinceBall.reset(); }
                    boolean allFour = balls >= 4 && sinceBall.seconds() > FEED_AFTER_4_SEC;
                    if (allFour || stepTimer.seconds() > FEED_MAX_SEC) {
                        intake.setPower(0.0);
                        stopper.setOpen(false);          // lights RED
                        flywheelOn = false;
                        if (PARK) {
                            Pose here = follower.pose();
                            Pose park = new Pose(PARK_X, PARK_Y, here.heading());
                            follower.follow(Paths.line(here, park).linear(here, park));
                            beginStep(Step.PARK);
                        } else {
                            beginStep(Step.DONE);
                        }
                    }
                    break;
                }
                case PARK: {
                    if (follower.atParametricEnd() || stepTimer.seconds() > PARK_TIMEOUT) {
                        follower.hold(new Pose(PARK_X, PARK_Y, follower.pose().heading()));
                        beginStep(Step.DONE);
                    }
                    break;
                }
                case DONE:
                    break;
            }

            flywheel.update(flywheelOn, AxonTeleOp.SHOOT_RPM, battery);
            follower.update();

            telemetry.addData("Step", "%s  (%.1f s in step, %.1f s total)", step, stepTimer.seconds(), autoTimer.seconds());
            telemetry.addData("Pose", poseText());
            telemetry.addData("Aim", aimText);
            telemetry.addData("Target", hive.seenThisFrame()
                    ? String.format("%s at %.0f in", hive.clusterName(), hive.distanceIn()) : "no tag");
            telemetry.addData("Flywheel RPM", "%.0f / %.0f  (power %.2f) %s", flywheel.rpm(), AxonTeleOp.SHOOT_RPM,
                    flywheel.power(), flywheel.atSpeed() ? "AT SPEED" : "");
            telemetry.addData("Shot", shootReason.isEmpty() ? "-" : shootReason);
            telemetry.addData("Ball speeds", flywheel.ballLog());
            telemetry.addData("Stopper", stopper.status());
            telemetry.addData("Battery", "%.2f V", battery.volts());
            telemetry.update();
        }

        flywheel.stop();
        intake.setPower(0.0);
        hive.stop();
    }

    private void beginStep(Step s) {
        step = s;
        stepTimer.reset();
    }

    /** Robot speed in inches per second. */
    private double speed() {
        return Math.hypot(follower.velocity().vx, follower.velocity().vy);
    }

    /**
     * Turns the robot to the HIVE by changing the heading it HOLDS at SHOOT.
     * The camera says how far off the HIVE is; the Pinpoint does the turning (no camera delay).
     * New target headings are only taken while the robot is nearly still, so an old camera picture
     * taken mid-turn is never used.
     * Returns true when it has been on target for AxonTeleOp.AIM_SETTLE_SEC.
     */
    private boolean updateAim() {
        if (!aimEnabled) { aimText = "OFF - it turned the wrong way (heading sign?)"; return false; }
        if (!hive.seenThisFrame()) {
            aimText = everSawTag ? String.format("tag lost (last %.1f deg off)", lastAimError) : "no tag yet";
            wasOnTarget = false;
            return false;
        }
        double errorDeg = hive.aimErrorDeg() + AxonTeleOp.AIM_OFFSET_DEG;   // + = HIVE is to the RIGHT
        if (!everSawTag) firstAimError = errorDeg;
        everSawTag = true;
        lastAimError = errorDeg;

        if (Math.abs(errorDeg) > Math.abs(firstAimError) + AIM_WRONG_WAY_DEG) {   // getting WORSE: stop and shoot as-is
            aimEnabled = false;
            follower.hold(new Pose(SHOOT_X, SHOOT_Y, shoot.heading()));
            aimText = "OFF - it turned the wrong way (heading sign?)";
            return false;
        }

        boolean still = Math.abs(Math.toDegrees(follower.velocity().omega)) < 20 && speed() < SETTLE_SPEED_IN_S;
        if (still && Math.abs(errorDeg) > 0.3) {
            // Heading + = turned LEFT. HIVE to the RIGHT (+ error) -> turn right -> smaller heading.
            double newHeading = Angle.normalizeSigned(follower.pose().heading() - Math.toRadians(errorDeg));
            if (Math.abs(Angle.normalizeSigned(newHeading - holdHeading)) > Math.toRadians(0.3)) {
                holdHeading = newHeading;
                follower.hold(new Pose(SHOOT_X, SHOOT_Y, holdHeading));
            }
        }

        boolean onNow = Math.abs(errorDeg) < AxonTeleOp.AIM_TOLERANCE;
        if (onNow && !wasOnTarget) onTarget.reset();
        wasOnTarget = onNow;
        boolean locked = onNow && onTarget.seconds() >= AxonTeleOp.AIM_SETTLE_SEC;
        aimText = String.format("%.1f deg off %s", errorDeg, locked ? "(LOCKED)" : still ? "" : "(turning)");
        return locked;
    }

    private String poseText() {
        Pose p = follower.pose();
        return String.format("x %.1f  y %.1f  heading %.1f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}
