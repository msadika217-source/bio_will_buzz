package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import java.util.ArrayList;
import java.util.List;

/*
 * Shooter Tuner - builds the distance -> flywheel speed table for "zack code".
 *
 * HOW TO USE
 *   1. Park at a distance from the HIVE. Hold Cross to aim.
 *   2. R1 = flywheel on, L1 = off. Square = intake/feed a ball.
 *   3. D-pad RIGHT / LEFT = +100 / -100 RPM.   Right stick click / Left stick click = +25 / -25 RPM.
 *   4. When shots go in reliably, press TRIANGLE to save "distance -> RPM".
 *   5. Repeat at 3-5 distances (close, middle, far). Send the saved list.
 *
 * Uses the same aim (gyro) and flywheel PIDF as "zack code", so the numbers carry over exactly.
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

    private boolean flywheelOn = false;
    private double targetRpm = AxonTeleOp.NO_TARGET_RPM;
    private final List<String> saved = new ArrayList<>();

    private boolean lastRight, lastLeft, lastRStick, lastLStick, lastTriangle;

    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);

        flowerArm = hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));

        hive = new HiveTarget(hardwareMap, AxonTeleOp.LIMELIGHT_NAME);
        aim = new AimController(hardwareMap);
    }

    @Override
    public void start() {
        aim.resetHeading();
        flowerArm.setPosition(AxonTeleOp.ARM_UP);   // keep the arm stowed while tuning
    }

    @Override
    public void loop() {
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
        drivetrain.drive(new DrivePowers(gamepad1.left_stick_y, gamepad1.left_stick_x, turn), true);

        // Intake / feed.
        if (gamepad1.square) intake.setPower(1.0);
        else if (gamepad1.circle) intake.setPower(-1.0);
        else intake.setPower(0.0);

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
        if (gamepad1.left_bumper) flywheelOn = false;
        flywheel.update(flywheelOn, targetRpm, battery);

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
        telemetry.addData("Battery", "%.2f V", battery.volts());
        telemetry.addLine("--- Saved (Triangle) ---");
        for (String s : saved) telemetry.addLine(s);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheel.stop();
        hive.stop();
    }
}