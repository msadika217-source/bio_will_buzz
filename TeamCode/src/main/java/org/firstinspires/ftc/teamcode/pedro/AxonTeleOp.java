package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * "zack code" TeleOp (one player, gamepad 1)
 *
 * BEFORE START (during INIT):
 *   Cross = aim at BLUE HIVE only   Circle = aim at RED HIVE only   (default: ANY HIVE)
 *   ("zack blue" / "zack red" are this same TeleOp, locked to one alliance - no button needed.)
 *   (The gyro is switched OFF - USE_GYRO = false - so aiming uses the camera only.)
 *
 * CONTROLS
 *   Left stick                : drive (robot-centric: up = the way the robot faces)
 *   Right stick               : turn
 *   Square / Circle           : intake in / out (100% power, always - also feeds the balls when shooting)
 *   HOLD Cross                : AUTO-AIM - turns the robot to the HIVE (left stick still drives)
 *   R1 (right bumper)         : flywheel ON  -> runs at SHOOT_RPM
 *   D-pad DOWN                : flywheel OFF -> coasts to a stop
 *   D-pad UP                  : stopper TOGGLE - each press flips it:
 *                                 CLOSED (balls held)   = light RED   <- starts here at Start
 *                                 OPEN   (balls go out) = light GREEN
 *   R2 (right trigger)        : flower arm DOWN
 *   L2 (left trigger)         : flower arm UP (stowed)
 *
 * TO SHOOT: R1 (flywheel) -> hold Cross (aim) -> D-pad UP (stopper open, green) -> hold Square (feed).
 * Controller rumbles when the robot is AIMED and the flywheel is AT SPEED = shoot.
 * The flywheel does NOT start on its own. It only runs after R1.
 * Drive, aim and flywheel are battery-compensated: they act the same on a full or
 * tired battery (down to about 12 V - below that there is no spare power left to make up with).
 *
 * ONLY EDIT THE "SETTINGS" BLOCK BELOW.
 */
@TeleOp(name = "zack code", group = "Drive")
public class AxonTeleOp extends OpMode {

    // ======================= SETTINGS =======================
    // Names must match the Driver Station "Configure Robot" screen exactly.
    static final String FLOWER_ARM_NAME = "flowerArm";    // Axon MAX
    static final String LEADER_NAME     = "leftouttake";  // encoder cable plugged in
    static final String FOLLOWER_NAME   = "rightouttake"; // encoder cable plugged in (speed = average of both)
    static final String LIMELIGHT_NAME  = "limelight";
    static final String IMU_NAME        = "imu";
    static final boolean USE_GYRO       = false;     // false = never touch the gyro (aim uses the camera only)

    // Which way the Control Hub is mounted (for the gyro). Look at the hub ON the robot:
    //   HUB_LOGO = the way the REV logo faces:  UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
    //   HUB_USB  = the way the USB-C port faces: UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD
    //   (they can't both point the same way or opposite ways)
    // This robot: hubs stand UPRIGHT (label facing out sideways, ports along the top edge, USB end
    // pointing sideways). Only the heading (yaw) is used, so any upright pair works - LEFT/FORWARD is fine.
    static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // ---- Limelight mounting (used to tell the UPWARD CELL from the downward one) ----
    // MEASURE THESE. Pitch = how far the camera is tilted UP from level (phone level app on the
    // Limelight's face: 90 deg face-vertical = 0 pitch). Height = floor to camera lens, inches.
    // Check: during INIT the "Cells" line should show the UP cell at about 45-52 in, the down one ~30-38 in.
    // From Fusion (Deck_V2 v97, "V2R6 Turret Limelight mount 35"): lens glass faces 35.0 deg up,
    // centre ~16.0 in above the floor. Re-check with a tape measure once the new mount is printed.
    // (Old 45 deg mount: 45.0 / 15.7 - switch back if you're still running the old mount.)
    static final double CAMERA_PITCH_DEG      = 35.0;
    static final double CAMERA_HEIGHT_IN      = 16.0;
    static final double UP_CELL_MIN_HEIGHT_IN = 41.0;   // a lone cluster above this = upward CELL

    // Flower arm positions (0.0 to 1.0), found with "Flower Arm Finder".
    // Nov 13: re-measured after the arm stopped going all the way down. Both moved by the same
    // 0.120 (was UP 0.810 / DOWN 0.240, measured Nov 5), which is what a skipped gear tooth looks like.
    static final double ARM_UP   = 0.690;  // stowed, inside 18 in - the arm always STARTS here
    static final double ARM_DOWN = 0.120;  // ramp under the Flower

    // ---- Stopper: goBILDA 2000 Series torque servo (25-2) in standard (position) mode ----
    // It goes to a position and holds it by itself. Find both numbers with "Axon Position Finder"
    // (press X = stopper). Set them just SHORT of the printed hard stops so it never strains.
    static final String STOPPER_NAME        = "stopper";   // Expansion Hub 2 servo port 5, type: Servo
    static final double STOPPER_CLOSED_POS  = 0.100; // measured (goBILDA): blocks the balls
    static final double STOPPER_OPEN_POS    = 0.450; // measured (goBILDA): lets the balls through
    static final double STOPPER_TOGGLE_LOCKOUT = 0.40; // seconds after a flip when D-pad UP is ignored (stops double-flips)

    // ---- RGB indicator (goBILDA 3118-0808-0002): colour is set like a servo position ----
    static final String RGB_NAME  = "rgbLight";      // Control Hub servo port 0, type: Servo
    static final String RGB2_NAME = "rgblight2";     // Expansion Hub servo port 0, type: Servo. Same colours as rgbLight.
    static final double RGB_RED   = 0.309;           // measured with Position Finder (A) on this light
    static final double RGB_GREEN = 0.489;           // measured with Position Finder (A) on this light

    // ---- Flywheel speed: ONE fixed speed (no distance table) ----
    // Nov 5: distance-based speed removed. Change this one number to make every shot go farther / shorter.
    // History: 3620 RPM (Oct 28 test) -> 3420 RPM (Nov 5, -200) -> 3150 RPM (Nov 6).
    static final double SHOOT_RPM = 3150;

    // ---- Battery compensation ----
    static final double NOMINAL_VOLTAGE    = 12.0;   // everything is tuned "as if" the battery were this
    static final boolean DRIVE_VOLTAGE_COMP = true;  // false = sticks feel stronger on a full battery

    // ---- Auto-aim - tune with "Aim Tuner" ----
    // AIM_KP, AIM_KD and AIM_MAX_POWER are the GYRO numbers (tuned Oct 17): only used when
    // USE_GYRO = true. AIM_KS, AIM_TOLERANCE and AIM_OFFSET_DEG are used with or without the gyro.
    static final double AIM_KP         = 0.021;   // turn power per degree off target
    static final double AIM_KD         = 0.0016;  // braking per degree/second of turning (stops wobble)
    static final double AIM_KS         = 0.05;    // minimum push to overcome friction (the "F")
    static final double AIM_MAX_POWER  = 0.60;    // fastest it will turn while aiming
    static final double AIM_TOLERANCE  = 1.5;     // "aimed" if within this many degrees
    static final double AIM_OFFSET_DEG = 0.0;     // + if shots land LEFT, - if they land RIGHT
    // Camera-only aiming (USE_GYRO = false). The camera picture is about 50 ms old, so steering off it
    // needs gentler numbers than the gyro did, or the robot rocks back and forth across the target.
    // "Aim Tuner" changes these live and shows which names to copy the result into.
    static final double AIM_CAM_KP        = 0.008;   // (the gyro value 0.021 is too strong without a gyro)
    static final double AIM_CAM_KD        = 0.0016;  // braking, from how fast the tag moves across the picture
    static final double AIM_CAM_MAX_POWER = 0.50;
    static final double AIM_SETTLE_SEC = 0.12;    // must stay on target this long before it says LOCKED / SHOOT
    static final double AIM_LOST_SEC   = 0.30;    // no tag for this long -> stop turning and give the stick back

    // ---- Flywheel PIDF - all at 12 V, battery-compensated ----
    // Flywheel direction: leader FORWARD, follower REVERSE (always opposite so they never fight).
    static final DcMotorSimple.Direction LEADER_DIR   = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction FOLLOWER_DIR = DcMotorSimple.Direction.REVERSE;
    static final double TICKS_PER_REV = 28;            // goBILDA 6000 RPM Yellow Jacket
    static final double MAX_RPM       = 6000;
    static final double FLY_kV = 1.0 / 2800.0;         // F: power per tick/sec (6000 RPM = 2800 ticks/sec at 12 V)
    static final double FLY_kS = 0.0;                  // F: extra power to overcome friction
    static final double FLY_kP = 0.004;                // P: push back when a ball slows the wheel. Speed hunts up and down -> lower it.
    static final double FLY_kI = 0.012;                // I: learns the leftover power the wheel needs (was 0.0005 = took ~10 s)
    static final double FLY_kD = 0.0;                  // D: leave 0 (encoder speed is noisy)
    static final double FLY_I_ZONE_RPM = 300;          // only use I within this many RPM of target
    static final double FLY_I_RATE_RPM = 40;           // I treats any bigger error as this much
    static final double FLY_I_WAIT_SEC = 0.8;          // I pauses this long after the wheel is knocked off speed (a ball), so shots don't upset it
    static final double FLY_I_MAX      = 0.30;         // I can add at most this much power (was 0.15)
    static final double READY_RPM_TOLERANCE = 60;      // "at speed" for the SHOOT light = within this many RPM (was 150)
    static final double BALL_DETECT_DROP_RPM = 60;     // ball log: a sudden drop this big = a ball was shot
    static final double BALL_DETECT_RISE_RPM = 30;     // ball log: wheel climbing back this much = ready for the next

    // (Nov 13: the paced feed was taken out. The intake is plain 100% power, always.)
    static final double FEED_READY_RPM    = 50;        // not used by the intake any more (Flywheel.readyToFeed() still reads it)
    // ========================================================

    private Mecanum drivetrain;
    private DcMotorEx intake;

    // Intake motor: goBILDA 5203 1620 RPM, 103.8 encoder counts per output shaft turn.
    private static final double INTAKE_COUNTS_PER_REV = 103.8;
    private ServoImplEx flowerArm;
    private Stopper stopper;   // stopper servo + red/green light
    private HiveTarget hive;
    private AimController aim;
    private Flywheel flywheel;
    private Battery battery;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean flywheelOn = false;
    private double armTarget = ARM_UP;
    private boolean wasReady = false;
    private boolean wasStopperOpen = false;
    private final ElapsedTime sinceRumble = new ElapsedTime();
    private String feedText = "";
    private int loopCount = 0;
    private double intakeAmps = 0;

    @Override
    public void init() {
        // Read all hub sensors in one go each loop = faster loop = smoother aim.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // power control as before; encoder still readable
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // stop instantly on release (no coasting)
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);

        flowerArm = hardwareMap.get(ServoImplEx.class, FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        stopper = new Stopper(hardwareMap);   // light shows RED while waiting

        telemetry.setMsTransmissionInterval(50);   // update the screen 20x a second (was 4x)
        hive = new HiveTarget(hardwareMap, LIMELIGHT_NAME);
        if (lockedAlliance() != null) hive.setAlliance(lockedAlliance());
        aim = new AimController(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (lockedAlliance() == null) {
            if (gamepad1.cross)  hive.setAlliance(HiveTarget.Alliance.BLUE);
            if (gamepad1.circle) hive.setAlliance(HiveTarget.Alliance.RED);
        }
        hive.update();
        aim.update();
        battery.update();
        telemetry.addData("Target HIVE", hive.getAlliance() + (lockedAlliance() != null
                ? "   (locked by this OpMode)" : "   (Cross = BLUE, Circle = RED)"));
        telemetry.addData("Limelight sees", hive.seenThisFrame()
                ? hive.clusterName() + (hive.distanceIn() > 0 ? String.format(" at %.0f in", hive.distanceIn()) : " (no distance)")
                : "nothing");
        telemetry.addData("Cells", hive.cells());
        telemetry.addData("Limelight", hive.debug());
        telemetry.addData("Camera", hive.status());
        telemetry.addData("Gyro", aim.imuOk()
                ? String.format("Heading %.0f deg  (turn robot LEFT by hand -> should go UP)", aim.heading())
                : !USE_GYRO ? "OFF (USE_GYRO = false) - aim uses the camera only"
                : "NOT WORKING - check HUB_LOGO / HUB_USB: " + aim.imuError());
        telemetry.addData("Battery", "%.2f V %s", battery.volts(), battery.volts() < 12.5 ? " <- CHARGE ME" : "");
        telemetry.update();
    }

    @Override
    public void start() {
        aim.resetHeading();
        flowerArm.setPosition(armTarget);
        stopper.startClosed();        // start CLOSED (red): drive to the closed hard stop
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        aim.update();
        battery.update();
        hive.update();

        // ---- Drive (robot-centric) ----
        // Signs tested on this robot (Nov 13, after the drive motor names were swapped for Pedro):
        // forward = -left_stick_y, strafe = -left_stick_x. (Before Nov 13 both were + .)
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        if (DRIVE_VOLTAGE_COMP) {
            forward *= battery.scale();
            strafe  *= battery.scale();
            turn    *= battery.scale();
        }

        // ---- Auto-aim (hold Cross) ----
        if (gamepad1.cross) {
            if (hive.seenThisFrame()) {
                aim.cameraUpdate(hive.aimErrorDeg() + AIM_OFFSET_DEG, hive.frameAgeMs());
            }
            if (aim.hasAim()) turn = aim.turnPower(battery);   // no tag for AIM_LOST_SEC -> the stick turns again
        } else {
            aim.clearTarget();
        }
        boolean aimed = aim.isLocked();

        // Keep the wheel powers in range without changing the direction of travel.
        double biggest = Math.abs(forward) + Math.abs(strafe) + Math.abs(turn);
        if (biggest > 1.0) {
            forward /= biggest;
            strafe  /= biggest;
            turn    /= biggest;
        }
        drivetrain.drive(new DrivePowers(forward, strafe, turn), true);

        // ---- Flywheel: R1 on, D-pad DOWN off, always SHOOT_RPM ----
        if (gamepad1.right_bumper) flywheelOn = true;
        if (gamepad1.dpad_down)    flywheelOn = false;
        flywheel.update(flywheelOn, SHOOT_RPM, battery);

        // Rumble when it's time to shoot (at most once every 1.5 s, so it doesn't buzz after every
        // ball of a volley).
        boolean ready = flywheel.atSpeed() && aimed;
        if (ready && !wasReady && sinceRumble.seconds() > 1.5) {
            gamepad1.rumble(250);
            sinceRumble.reset();
        }
        wasReady = ready;

        // ---- Stopper: D-pad UP toggles (red = closed/holding, green = open/shooting) ----
        stopper.update(gamepad1.dpad_up);
        if (stopper.isOpen() && !wasStopperOpen) flywheel.resetBallLog();   // new volley -> fresh ball log
        wasStopperOpen = stopper.isOpen();

        // ---- Intake: Square in, Circle out (100% power, always runs, even with the flywheel on) ----
        if (gamepad1.square) {
            intake.setPower(1.0);
            feedText = "Square: in (100% power)";
        } else if (gamepad1.circle) {
            intake.setPower(-1.0);
            feedText = "Circle: out (100% power)";
        } else {
            intake.setPower(0.0);
            feedText = "off";
        }

        // ---- Flower arm: R2 down, L2 up ----
        if (gamepad1.right_trigger > 0.5) armTarget = ARM_DOWN;
        if (gamepad1.left_trigger  > 0.5) armTarget = ARM_UP;
        flowerArm.setPosition(armTarget);

        // ---- Screen ----
        telemetry.addData("SHOOT?", ready ? ">>> YES <<<" : "not yet");
        telemetry.addData("Target", hive.hasTarget()
                ? String.format("%s | %d tags | %.0f in", hive.clusterName(), hive.tagsSeen(), hive.distanceIn())
                : "NO UPWARD CELL in view");
        telemetry.addData("Cells", hive.cells());
        telemetry.addData("Aim", gamepad1.cross
                ? (aim.hasAim() ? String.format("%s  %.1f deg off", aimed ? "LOCKED" : "turning...", aim.error()) : "no tag in view")
                : "hold Cross to aim");
        telemetry.addData("Flywheel", flywheelOn ? "ON  (D-pad DOWN to stop)" : "OFF (R1 to start)");
        telemetry.addData("Flywheel RPM", "%.0f / %.0f  (power %.2f)%s", flywheel.rpm(), flywheel.targetRpm(), flywheel.power(),
                flywheel.isRecovering() ? "  FULL POWER" : "");
        telemetry.addData("Each motor", "left %.0f | right %.0f RPM", flywheel.leftRpm(), flywheel.rightRpm());
        if (!flywheel.encoderWarning().isEmpty()) telemetry.addData("ENCODER", flywheel.encoderWarning());
        telemetry.addData("Shot record", "biggest dip %.0f RPM | full-power catch-ups %d   (resets on R1)",
                flywheel.biggestDipRpm(), flywheel.recoveries());
        telemetry.addData("Ball speeds", flywheel.ballLog());
        telemetry.addData("Intake", feedText);
        telemetry.addData("Code version", "Nov 13: flower arm 0.690 / 0.120");
        if (!aim.imuOk()) telemetry.addData("Gyro", USE_GYRO ? "NOT WORKING - aim is camera-only" : "off - aim is camera-only");
        telemetry.addData("Flower arm", armTarget == ARM_UP ? "UP" : "DOWN");
        telemetry.addData("Stopper", stopper.status() + "   (D-pad UP)");
        double intakeCps = intake.getVelocity();                          // counts per second
        double intakeRpm = intakeCps / INTAKE_COUNTS_PER_REV * 60.0;
        telemetry.addData("Intake speed", "%.0f counts/s  |  %.0f rpm", intakeCps, intakeRpm);
        // Reading the current costs a separate trip to the hub, so only do it every 10th loop.
        if (loopCount++ % 10 == 0) intakeAmps = intake.getCurrent(CurrentUnit.AMPS);
        telemetry.addData("Intake current", "%.2f A", intakeAmps);
        telemetry.addData("Battery", "%.2f V %s", battery.volts(), battery.volts() < 12.0 ? " <- LOW, CHARGE" : "");
        telemetry.addData("Loop", "%.0f ms", loopMs);
        telemetry.addData("Limelight", hive.debug());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
        stopper.stop();
        hive.stop();
    }

    /**
     * Which HIVE tags the Limelight is allowed to use. null = pick in INIT with Cross/Circle.
     * "zack blue" and "zack red" override this to lock it to one alliance.
     */
    protected HiveTarget.Alliance lockedAlliance() {
        return null;
    }
}