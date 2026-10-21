package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * "zack code" TeleOp (one player, gamepad 1)
 *
 * BEFORE START (during INIT):
 *   Cross = aim at BLUE HIVE only   Circle = aim at RED HIVE only   (default: ANY HIVE)
 *   Turn the robot LEFT by hand - "Heading" should go UP (+). If it doesn't, fix HUB_LOGO / HUB_USB.
 *
 * CONTROLS
 *   Left stick                : drive (FIELD-CENTRIC: up = away from you, no matter where the robot faces)
 *   Right stick               : turn
 *   Options (Start)           : reset field-centric "forward" - face the robot AWAY from you, then press
 *   Share (Back)              : switch field-centric ON / OFF
 *   Square / Circle           : intake in / out
 *   HOLD Cross                : AUTO-AIM - gyro turns the robot to the HIVE (left stick still drives)
 *   R1 (right bumper)         : flywheel ON  -> speed set from the Limelight distance
 *   L1 (left bumper)          : flywheel OFF -> stops completely
 *   (R2 / L2 reserved for the stopper - added later)
 *   D-pad Up / D-pad Down     : flower arm up (stowed) / down
 *
 * Controller rumbles when the robot is AIMED and the flywheel is AT SPEED = shoot.
 * The flywheel does NOT start on its own. It only runs after R1.
 * Drive, aim and flywheel are battery-compensated: they act the same on a full or tired battery.
 *
 * ONLY EDIT THE "SETTINGS" BLOCK BELOW.
 */
@TeleOp(name = "zack code", group = "Drive")
public class AxonTeleOp extends OpMode {

    // ======================= SETTINGS =======================
    // Names must match the Driver Station "Configure Robot" screen exactly.
    static final String FLOWER_ARM_NAME = "flowerArm";    // Axon MAX
    static final String LEADER_NAME     = "leftouttake";  // MUST have its encoder cable plugged in
    static final String FOLLOWER_NAME   = "rightouttake";
    static final String LIMELIGHT_NAME  = "limelight";
    static final String IMU_NAME        = "imu";

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

    // Flower arm positions (0.0 to 1.0), found with "Axon Position Finder".
    static final double ARM_UP   = 0.79;   // stowed, inside 18 in (measured)
    static final double ARM_DOWN = 0.10;   // measured

    // ---- Distance -> flywheel speed table (fill in with "Shooter Tuner") ----
    // Distance from the Limelight to the HIVE cluster, in inches, smallest first.
    // Speeds in between are blended automatically. Right now every distance = 57% (3420 RPM).
    static final double[] TABLE_DISTANCE_IN = {  30,   50,   70 };
    static final double[] TABLE_RPM         = { 3420, 3420, 3420 };
    static final double NO_TARGET_RPM       = 3420;  // used when no tag is seen (57%)

    // ---- Battery compensation ----
    static final double NOMINAL_VOLTAGE    = 12.0;   // everything is tuned "as if" the battery were this
    static final boolean DRIVE_VOLTAGE_COMP = true;  // false = sticks feel stronger on a full battery

    // ---- Drive ----
    static final boolean FIELD_CENTRIC_AT_START = true;

    // ---- Auto-aim (gyro PDF) - tune with "Aim Tuner" ----
    static final double AIM_KP         = 0.015;   // turn power per degree off target
    static final double AIM_KD         = 0.0010;  // braking per degree/second of turning (stops wobble)
    static final double AIM_KS         = 0.05;    // minimum push to overcome friction (the "F")
    static final double AIM_MAX_POWER  = 0.50;    // fastest it will turn while aiming
    static final double AIM_TOLERANCE  = 1.5;     // "aimed" if within this many degrees
    static final double AIM_OFFSET_DEG = 0.0;     // + if shots land LEFT, - if they land RIGHT

    // ---- Flywheel PIDF - all at 12 V, battery-compensated ----
    // Flywheel direction: leader FORWARD, follower REVERSE (always opposite so they never fight).
    static final DcMotorSimple.Direction LEADER_DIR   = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction FOLLOWER_DIR = DcMotorSimple.Direction.REVERSE;
    static final double TICKS_PER_REV = 28;            // goBILDA 6000 RPM Yellow Jacket
    static final double MAX_RPM       = 6000;
    static final double FLY_kV = 1.0 / 2800.0;         // F: power per tick/sec (6000 RPM = 2800 ticks/sec at 12 V)
    static final double FLY_kS = 0.0;                  // F: extra power to overcome friction
    static final double FLY_kP = 0.0015;               // P: push back when a ball slows the wheel
    static final double FLY_kI = 0.0005;               // I: removes leftover steady error
    static final double FLY_kD = 0.0;                  // D: leave 0 (encoder speed is noisy)
    static final double FLY_I_ZONE_RPM = 300;          // only use I within this many RPM of target
    static final double FLY_I_MAX      = 0.15;         // I can add at most this much power
    static final double READY_RPM_TOLERANCE = 150;
    // ========================================================

    private Mecanum drivetrain;
    private DcMotor intake;
    private ServoImplEx flowerArm;
    private HiveTarget hive;
    private AimController aim;
    private Flywheel flywheel;
    private Battery battery;
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean flywheelOn = false;
    private boolean fieldCentric = FIELD_CENTRIC_AT_START;
    private double armTarget = ARM_UP;
    private boolean wasReady = false;
    private boolean lastShare = false, lastOptions = false;

    @Override
    public void init() {
        // Read all hub sensors in one go each loop = faster loop = smoother aim.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);

        flowerArm = hardwareMap.get(ServoImplEx.class, FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        hive = new HiveTarget(hardwareMap, LIMELIGHT_NAME);
        aim = new AimController(hardwareMap);
    }

    @Override
    public void init_loop() {
        if (gamepad1.cross)  hive.setAlliance(HiveTarget.Alliance.BLUE);
        if (gamepad1.circle) hive.setAlliance(HiveTarget.Alliance.RED);
        hive.update();
        aim.update();
        battery.update();
        telemetry.addData("Target HIVE", hive.getAlliance() + "   (Cross = BLUE, Circle = RED)");
        telemetry.addData("Limelight sees", hive.seenThisFrame()
                ? hive.clusterName() + (hive.distanceIn() > 0 ? String.format(" at %.0f in", hive.distanceIn()) : " (no distance)")
                : "nothing");
        telemetry.addData("Limelight", hive.debug());
        telemetry.addData("Camera", hive.status());
        telemetry.addData("Gyro", aim.imuOk()
                ? String.format("Heading %.0f deg  (turn robot LEFT by hand -> should go UP)", aim.heading())
                : "NOT WORKING - check HUB_LOGO / HUB_USB: " + aim.imuError());
        telemetry.addData("Battery", "%.2f V %s", battery.volts(), battery.volts() < 12.5 ? " <- CHARGE ME" : "");
        telemetry.update();
    }

    @Override
    public void start() {
        aim.resetHeading();   // the way the robot faces at Start = "forward" for field-centric
        flowerArm.setPosition(armTarget);
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        aim.update();
        battery.update();
        hive.update();

        // ---- Field-centric controls ----
        if (gamepad1.share && !lastShare) fieldCentric = !fieldCentric;
        if (gamepad1.options && !lastOptions) aim.resetHeading();
        lastShare = gamepad1.share;
        lastOptions = gamepad1.options;
        boolean useFieldCentric = fieldCentric && aim.imuOk();

        // ---- Drive ----
        // Stick -> "field" directions: F = away from driver, L = to the driver's left.
        double F = -gamepad1.left_stick_y;
        double L = -gamepad1.left_stick_x;
        double f = F, l = L;
        if (useFieldCentric) {
            double h = Math.toRadians(aim.heading());
            f =  F * Math.cos(h) + L * Math.sin(h);
            l = -F * Math.sin(h) + L * Math.cos(h);
        }
        // This robot's DrivePowers signs (tested: forward = left_stick_y, strafe = left_stick_x).
        double forward = -f;
        double strafe  = -l;
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
            if (aim.hasAim()) turn = aim.turnPower(battery);   // gyro holds aim even if the tag flickers
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

        // ---- Intake (same as One Teleop) ----
        if (gamepad1.square) {
            intake.setPower(1.0);
        } else if (gamepad1.circle) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        // ---- Flywheel: R1 on, L1 off, speed from distance ----
        if (gamepad1.right_bumper) flywheelOn = true;
        if (gamepad1.left_bumper)  flywheelOn = false;
        double targetRpm = (hive.hasTarget() && hive.distanceIn() > 0)
                ? rpmForDistance(hive.distanceIn()) : NO_TARGET_RPM;
        flywheel.update(flywheelOn, targetRpm, battery);

        // Rumble once when it's time to shoot.
        boolean ready = flywheel.atSpeed() && aimed;
        if (ready && !wasReady) gamepad1.rumble(250);
        wasReady = ready;

        // ---- Flower arm: D-pad up / down ----
        if (gamepad1.dpad_up)   armTarget = ARM_UP;
        if (gamepad1.dpad_down) armTarget = ARM_DOWN;
        flowerArm.setPosition(armTarget);

        // ---- Screen ----
        telemetry.addData("SHOOT?", ready ? ">>> YES <<<" : "not yet");
        telemetry.addData("Target", hive.hasTarget()
                ? String.format("%s | %d tags | %.0f in", hive.clusterName(), hive.tagsSeen(), hive.distanceIn())
                : "NO TAG - using " + (int) NO_TARGET_RPM + " RPM");
        telemetry.addData("Aim", gamepad1.cross
                ? (aim.hasAim() ? String.format("%s  %.1f deg off", aimed ? "LOCKED" : "turning...", aim.error()) : "no tag yet")
                : "hold Cross to aim");
        telemetry.addData("Flywheel", flywheelOn ? "ON  (L1 to stop)" : "OFF (R1 to start)");
        telemetry.addData("Flywheel RPM", "%.0f / %.0f  (power %.2f)", flywheel.rpm(), flywheel.targetRpm(), flywheel.power());
        telemetry.addData("Drive", useFieldCentric
                ? String.format("FIELD-CENTRIC (heading %.0f)  Share = off, Options = reset", aim.heading())
                : (aim.imuOk() ? "robot-centric  (Share = field-centric)" : "robot-centric (gyro not working)"));
        telemetry.addData("Flower arm", armTarget == ARM_UP ? "UP" : "DOWN");
        telemetry.addData("Battery", "%.2f V %s", battery.volts(), battery.volts() < 12.0 ? " <- LOW, CHARGE" : "");
        telemetry.addData("Loop", "%.0f ms", loopMs);
        telemetry.addData("Limelight", hive.debug());
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
        hive.stop();
    }

    /** Look up flywheel RPM for a distance, blending between table rows. */
    static double rpmForDistance(double distIn) {
        int n = TABLE_DISTANCE_IN.length;
        if (distIn <= TABLE_DISTANCE_IN[0]) return TABLE_RPM[0];
        if (distIn >= TABLE_DISTANCE_IN[n - 1]) return TABLE_RPM[n - 1];
        for (int i = 0; i < n - 1; i++) {
            double d0 = TABLE_DISTANCE_IN[i], d1 = TABLE_DISTANCE_IN[i + 1];
            if (distIn <= d1) {
                double f = (distIn - d0) / (d1 - d0);
                return TABLE_RPM[i] + f * (TABLE_RPM[i + 1] - TABLE_RPM[i]);
            }
        }
        return TABLE_RPM[n - 1];
    }
}