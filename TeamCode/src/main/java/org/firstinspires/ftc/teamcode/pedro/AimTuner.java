package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Aim Tuner - tune the gyro auto-aim LIVE with the controller (no reinstalling).
 *
 * SETUP: tape a printed tag page (all 4 tags) flat on a wall where the Limelight can see it.
 *        During INIT, turn the robot LEFT by hand: "Heading" must go UP. If not, fix HUB_LOGO / HUB_USB.
 *
 * CONTROLS
 *   Sticks            : drive (robot-centric) - turn so the tag is off to one side
 *   HOLD Cross        : auto-aim
 *   D-pad Up / Down   : kP  (strength)      +/- 0.002
 *   D-pad Right / Left: kD  (braking)       +/- 0.0002
 *   R1 / L1           : kS  (friction push) +/- 0.01
 *   Triangle          : max turn power, cycles 0.3 / 0.4 / 0.5 / 0.6 / 0.7
 *
 * GOAL: from 20-30 deg off -> "Time to lock" under ~0.5 s and "Wobbles" 0.
 * Then copy the numbers at the bottom into zack code SETTINGS (AIM_KP, AIM_KD, AIM_KS, AIM_MAX_POWER).
 * The robot only turns while you hold Cross. No flywheel.
 */
@TeleOp(name = "Aim Tuner", group = "Test")
public class AimTuner extends OpMode {

    private static final double[] MAX_POWER_STEPS = {0.3, 0.4, 0.5, 0.6, 0.7};

    private Mecanum drivetrain;
    private HiveTarget hive;
    private AimController aim;
    private Battery battery;

    private boolean lastUp, lastDown, lastRight, lastLeft, lastRB, lastLB, lastTri, lastCross;

    private final ElapsedTime aimTimer = new ElapsedTime();
    private boolean lockedThisPress = false;
    private double lockTime = -1;
    private double startError = 0;
    private boolean haveStartError = false;
    private int wobbles = 0;
    private double lastOutsideSign = 0;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        hive = new HiveTarget(hardwareMap, AxonTeleOp.LIMELIGHT_NAME);
        aim = new AimController(hardwareMap);
        battery = new Battery(hardwareMap);
    }

    @Override
    public void init_loop() {
        hive.update();
        aim.update();
        battery.update();
        telemetry.addData("Limelight", hive.debug());
        telemetry.addData("Gyro", aim.imuOk()
                ? String.format("Heading %.0f deg  <- turn robot LEFT by hand, must go UP", aim.heading())
                : "NOT WORKING - check HUB_LOGO / HUB_USB: " + aim.imuError());
        telemetry.addData("Battery", "%.2f V", battery.volts());
        telemetry.update();
    }

    @Override
    public void start() {
        aim.resetHeading();
    }

    @Override
    public void loop() {
        aim.update();
        battery.update();
        hive.update();

        // ---- Live adjustments (one step per press) ----
        if (gamepad1.dpad_up && !lastUp)       aim.kP += 0.002;
        if (gamepad1.dpad_down && !lastDown)   aim.kP -= 0.002;
        if (gamepad1.dpad_right && !lastRight) aim.kD += 0.0002;
        if (gamepad1.dpad_left && !lastLeft)   aim.kD -= 0.0002;
        if (gamepad1.right_bumper && !lastRB)  aim.kS += 0.01;
        if (gamepad1.left_bumper && !lastLB)   aim.kS -= 0.01;
        if (gamepad1.triangle && !lastTri)     aim.maxPower = nextMaxPower(aim.maxPower);
        aim.kP = clamp(aim.kP, 0.0, 0.1);
        aim.kD = clamp(aim.kD, 0.0, 0.01);
        aim.kS = clamp(aim.kS, 0.0, 0.3);
        lastUp = gamepad1.dpad_up;  lastDown = gamepad1.dpad_down;
        lastRight = gamepad1.dpad_right;  lastLeft = gamepad1.dpad_left;
        lastRB = gamepad1.right_bumper;  lastLB = gamepad1.left_bumper;
        lastTri = gamepad1.triangle;

        // ---- New Cross press = new measurement ----
        if (gamepad1.cross && !lastCross) {
            aimTimer.reset();
            lockedThisPress = false;
            lockTime = -1;
            wobbles = 0;
            lastOutsideSign = 0;
            haveStartError = false;
        }
        lastCross = gamepad1.cross;

        double turn = -gamepad1.right_stick_x;
        double aimPower = 0;
        if (gamepad1.cross) {
            if (hive.seenThisFrame()) {
                aim.cameraUpdate(hive.aimErrorDeg() + AxonTeleOp.AIM_OFFSET_DEG, hive.frameAgeMs());
            }
            if (aim.hasAim()) {
                aimPower = aim.turnPower(battery);
                turn = aimPower;
                double err = aim.error();
                if (!haveStartError) { startError = err; haveStartError = true; }

                // Count overshoots: error flips from one side of the target to the other.
                if (Math.abs(err) >= AxonTeleOp.AIM_TOLERANCE) {
                    double s = Math.signum(err);
                    if (lastOutsideSign != 0 && s != lastOutsideSign) wobbles++;
                    lastOutsideSign = s;
                }
                if (aim.isLocked() && !lockedThisPress) {
                    lockedThisPress = true;
                    lockTime = aimTimer.seconds();
                    gamepad1.rumble(200);
                }
            }
        } else {
            aim.clearTarget();
        }
        drivetrain.drive(new DrivePowers(gamepad1.left_stick_y, gamepad1.left_stick_x, turn), true);

        // Turning away = gyro direction is wrong (HUB_LOGO / HUB_USB).
        boolean wrongWay = gamepad1.cross && aim.hasAim() && haveStartError && aimTimer.seconds() > 0.4
                && Math.abs(aim.error()) > Math.abs(startError) + 5 && !lockedThisPress;

        // ---- Screen ----
        if (wrongWay) telemetry.addLine(">>> TURNING AWAY - gyro direction wrong, check HUB_LOGO / HUB_USB <<<");
        if (!aim.imuOk()) telemetry.addLine("Gyro NOT working - using camera-only aim");
        telemetry.addData("Tag", hive.seenThisFrame()
                ? String.format("%s, %d tags (picture %.0f ms old)", hive.clusterName(), hive.tagsSeen(), hive.frameAgeMs())
                : "NO TAG");
        telemetry.addData("Error", aim.hasAim() ? String.format("%.1f deg %s", aim.error(), aim.isLocked() ? "(LOCKED)" : "") : "-");
        telemetry.addData("Heading / turn speed", "%.1f deg / %.0f deg/s", aim.heading(), aim.yawRate());
        telemetry.addData("Aim power", "%.2f", aimPower);
        telemetry.addData("Battery", "%.2f V", battery.volts());
        telemetry.addLine("--- Last Cross press ---");
        telemetry.addData("Started at", haveStartError ? String.format("%.1f deg off", startError) : "-");
        telemetry.addData("Time to lock", lockTime < 0 ? "not locked yet" : String.format("%.2f s", lockTime));
        telemetry.addData("Wobbles", wobbles);
        telemetry.addLine("--- Settings (copy into zack code) ---");
        telemetry.addData("AIM_KP", "%.3f   (D-pad Up/Down)", aim.kP);
        telemetry.addData("AIM_KD", "%.4f   (D-pad Right/Left)", aim.kD);
        telemetry.addData("AIM_KS", "%.2f   (R1/L1)", aim.kS);
        telemetry.addData("AIM_MAX_POWER", "%.2f   (Triangle)", aim.maxPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        hive.stop();
    }

    private static double nextMaxPower(double current) {
        for (double step : MAX_POWER_STEPS) {
            if (step > current + 0.001) return step;
        }
        return MAX_POWER_STEPS[0];
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}