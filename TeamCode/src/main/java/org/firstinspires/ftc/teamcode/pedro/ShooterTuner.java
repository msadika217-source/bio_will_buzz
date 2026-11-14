package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/*
 * Shooter Tuner - finds the flywheel speed for a shooting spot, and checks the flywheel itself.
 *
 * HOW TO USE
 *   1. Park at a distance from the HIVE. Hold Cross to aim.
 *   2. R1 = flywheel on, D-pad DOWN = off. D-pad UP = stopper toggle (red = closed, green = open).
 *      Square / Circle = intake in / out (full power, always). Same buttons as zack code.
 *   3. D-pad RIGHT / LEFT = +100 / -100 RPM.   Right stick click / Left stick click = +25 / -25 RPM.
 *   4. When shots go in reliably, press TRIANGLE to save "distance -> RPM".
 *   5. Repeat at each spot you want to shoot from. Send the saved list.
 *
 * NUMBERS WORTH SENDING BACK
 *   "Ball speeds"   - the wheel speed when each ball hit, how far it knocked the wheel down, and the
 *                     time since the ball before. This shows whether every ball got the same wheel.
 *   "FLY_kV check"  - the feedforward the wheel really needs (shows after it holds speed ~1 s).
 *   "Spin-up"       - seconds from R1 to holding speed.
 *
 * Uses the same aim and flywheel control as "zack code", so the numbers carry over exactly.
 */
@TeleOp(name = "Shooter Tuner", group = "Test")
public class ShooterTuner extends OpMode {

    private Mecanum drivetrain;
    private DcMotor intake;
    private ServoImplEx flowerArm;
    private HiveTarget hive;
    private AimController aim;
    private Flywheel flywheel;
    private Battery battery;
    private Stopper stopper;

    private boolean flywheelOn = false;
    private double targetRpm = AxonTeleOp.SHOOT_RPM;
    private final List<String> saved = new ArrayList<>();

    private boolean lastRight, lastLeft, lastRStick, lastLStick, lastTriangle;
    private boolean wasStopperOpen = false;
    private String feedText = "";
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean wasOn = false, timingSpinUp = false;
    private double spinUpSec = -1;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // stop instantly on release
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);
        stopper = new Stopper(hardwareMap);

        flowerArm = hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry.setMsTransmissionInterval(50);   // update the screen 20x a second
        hive = new HiveTarget(hardwareMap, AxonTeleOp.LIMELIGHT_NAME);
        aim = new AimController(hardwareMap);
    }

    @Override
    public void start() {
        aim.resetHeading();
        stopper.startClosed();
        flowerArm.setPosition(AxonTeleOp.ARM_UP);   // keep the arm stowed while tuning
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        aim.update();
        battery.update();
        hive.update();

        // Drive, with Cross to auto-aim (same aim as zack code).
        double turn = -gamepad1.right_stick_x;
        if (gamepad1.cross) {
            if (hive.seenThisFrame()) {
                aim.cameraUpdate(hive.aimErrorDeg() + AxonTeleOp.AIM_OFFSET_DEG, hive.frameAgeMs());
            }
            if (aim.hasAim()) turn = aim.turnPower(battery);
        } else {
            aim.clearTarget();
        }
        drivetrain.drive(new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, turn), true);   // same signs as zack code

        // Adjust target RPM.
        if (gamepad1.dpad_right && !lastRight) targetRpm += 100;
        if (gamepad1.dpad_left && !lastLeft) targetRpm -= 100;
        if (gamepad1.right_stick_button && !lastRStick) targetRpm += 25;
        if (gamepad1.left_stick_button && !lastLStick) targetRpm -= 25;
        targetRpm = Math.max(0, Math.min(AxonTeleOp.MAX_RPM, targetRpm));
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;
        lastRStick = gamepad1.right_stick_button;
        lastLStick = gamepad1.left_stick_button;

        // Flywheel (same PIDF + battery compensation as zack code).
        if (gamepad1.right_bumper) flywheelOn = true;
        if (gamepad1.dpad_down) flywheelOn = false;
        flywheel.update(flywheelOn, targetRpm, battery);

        // Spin-up time: from switching ON to first being at speed.
        if (flywheelOn && !wasOn) { spinUpTimer.reset(); timingSpinUp = true; }
        if (timingSpinUp && flywheel.atSpeed()) { spinUpSec = spinUpTimer.seconds(); timingSpinUp = false; }
        if (!flywheelOn) timingSpinUp = false;
        wasOn = flywheelOn;

        // Stopper (same as zack code).
        stopper.update(gamepad1.dpad_up);
        if (stopper.isOpen() && !wasStopperOpen) flywheel.resetBallLog();   // new volley -> fresh ball log
        wasStopperOpen = stopper.isOpen();

        // Intake (same as zack code): Square in, Circle out, 100% power, always.
        if (gamepad1.square) { intake.setPower(1.0); feedText = "Square: in (100% power)"; }
        else if (gamepad1.circle) { intake.setPower(-1.0); feedText = "Circle: out (100% power)"; }
        else { intake.setPower(0.0); feedText = "off"; }

        // Save a point.
        if (gamepad1.triangle && !lastTriangle) {
            if (hive.seenThisFrame() && hive.distanceIn() > 0) {
                saved.add(String.format("%.0f in -> %.0f RPM", hive.distanceIn(), targetRpm));
            } else {
                gamepad1.rumble(500);   // can't save without a tag (and distance) in view
            }
        }
        lastTriangle = gamepad1.triangle;

        telemetry.addData("Distance", hive.seenThisFrame()
                ? String.format("%.0f in  (%s, %d tags)", hive.distanceIn(), hive.clusterName(), hive.tagsSeen())
                : "NO TAG");
        telemetry.addData("Aim", aim.hasAim()
                ? String.format("%.1f deg off %s", aim.error(), aim.isLocked() ? "(LOCKED)" : "") : "-");
        telemetry.addData("Target RPM", "%.0f   (D-pad L/R +-100, stick clicks +-25)", targetRpm);
        telemetry.addData("Actual RPM", "%.0f   %s", flywheel.rpm(), flywheelOn ? (flywheel.atSpeed() ? "AT SPEED" : "") : "(flywheel OFF - R1)");
        telemetry.addData("Each motor", "left %.0f | right %.0f RPM", flywheel.leftRpm(), flywheel.rightRpm());
        if (!flywheel.encoderWarning().isEmpty()) telemetry.addData("ENCODER", flywheel.encoderWarning());
        telemetry.addData("Stopper", stopper.status() + "   (D-pad UP)");
        telemetry.addData("Intake", feedText);
        telemetry.addData("Shot record", "biggest dip %.0f RPM | full-power catch-ups %d", flywheel.biggestDipRpm(), flywheel.recoveries());
        telemetry.addData("Ball speeds", flywheel.ballLog());
        telemetry.addData("FLY_kV check", flywheel.suggestedKv().isEmpty()
                ? "hold speed for a second..." : flywheel.suggestedKv() + String.format("  now 1.0 / %.0f", 1.0 / AxonTeleOp.FLY_kV));
        telemetry.addData("Spin-up", spinUpSec < 0 ? "-" : String.format("%.2f s", spinUpSec));
        telemetry.addData("Battery", "%.2f V", battery.volts());
        telemetry.addData("Loop", "%.0f ms", loopMs);
        telemetry.addLine("--- Saved (Triangle) ---");
        for (String s : saved) telemetry.addLine(s);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
        stopper.stop();
        hive.stop();
    }
}