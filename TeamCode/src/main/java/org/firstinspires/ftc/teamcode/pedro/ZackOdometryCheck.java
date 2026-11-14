package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/*
 * "zack odometry check" - checks that the Pinpoint counts the right way, in plain words.
 *
 * THE WHEELS ARE NEVER POWERED. You move the robot by hand. Run it from the Driver Hub like any
 * TeleOp (no web page needed).
 *
 * Pedro can only drive an auto if all three of these are true:
 *   pushing the robot FORWARD  makes "forward" go UP
 *   pushing the robot LEFT     makes "left"    go UP
 *   turning the robot LEFT     makes "turn"    go UP
 * If any is wrong, the path follower corrects the wrong way and the robot shakes or runs off.
 *
 * HOW TO USE
 *   INIT with the robot sitting STILL (the Pinpoint calibrates). Press START.
 *   Step 1: push the robot straight FORWARD about one tile (24 in). Don't turn it.  Press CROSS.
 *   Step 2: push it straight to ITS LEFT about one tile. Don't turn it.            Press CROSS.
 *   Step 3: turn it LEFT (counter-clockwise from above) a quarter turn, in place.  (ends by itself)
 *   The screen then shows the 3 results. Take ONE photo of it.
 *   CIRCLE = start over.
 *
 * "FORWARD" = the way the robot drives when you push the left stick forward in zack code.
 * "LEFT"    = the robot's own left when you stand behind it looking the way it drives forward.
 */
@TeleOp(name = "zack odometry check", group = "Test")
public class ZackOdometryCheck extends OpMode {

    private static final double MIN_PUSH_IN = 6;     // a push shorter than this is not judged
    private static final double MIN_TURN_DEG = 30;   // a turn smaller than this is not judged
    private static final double AUTO_TURN_DEG = 60;  // step 3 ends by itself once it has turned this far

    private PinpointLocalizer localizer;
    private Pose last = Pose.zero();

    // Movement since the current step began, as the ROBOT feels it (not field x/y), so it does not
    // matter which way the robot is pointing.
    private double forwardIn = 0, leftIn = 0, turnDeg = 0;

    private int step = 0;                            // 0, 1, 2 = the three moves; 3 = finished
    private final String[] results = {"-", "-", "-"};
    private boolean lastCross, lastCircle;
    private String note = "";

    @Override
    public void init() {
        // Let the drive wheels spin freely so the robot is easy to push. Power stays at zero.
        String[] names = {
                Constants.drivetrainConfig.frontLeftName.get(), Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.backLeftName.get(), Constants.drivetrainConfig.backRightName.get()};
        for (String n : names) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, n);
            if (m != null) {
                m.setPower(0);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
        }
        telemetry.addLine("Keep the robot STILL - Pinpoint is calibrating...");
        telemetry.update();
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);   // calibrates (0.5 s)
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        localizer.update();
        telemetry.addLine("zack odometry check - the wheels are NEVER powered");
        telemetry.addLine("Robot still on the floor? Press START.");
        telemetry.addData("Pinpoint", poseText(localizer.pose()));
        telemetry.addData("Constants.java", configText());
        telemetry.update();
    }

    @Override
    public void start() {
        restart();
    }

    private void restart() {
        // Nothing is reset on the Pinpoint itself: every number here is movement SINCE this moment.
        localizer.update();
        last = localizer.pose();
        forwardIn = leftIn = turnDeg = 0;
        step = 0;
        results[0] = results[1] = results[2] = "-";
        note = "";
    }

    @Override
    public void loop() {
        localizer.update();
        Pose now = localizer.pose();
        double dx = now.x() - last.x(), dy = now.y() - last.y();
        double dh = Angle.normalizeSigned(now.heading() - last.heading());
        double h = last.heading() + dh / 2;          // heading halfway through this small move
        if (Math.hypot(dx, dy) < 20) {               // a jump this big in one loop is a glitch, not a push
            forwardIn += dx * Math.cos(h) + dy * Math.sin(h);
            leftIn += -dx * Math.sin(h) + dy * Math.cos(h);
            turnDeg += Math.toDegrees(dh);
        }
        last = now;

        if (gamepad1.circle && !lastCircle) restart();
        if (gamepad1.cross && !lastCross && step < 3) {
            String r = judge(step);
            if (r == null) {
                note = step == 2 ? "Not enough turn yet - keep turning it LEFT, it finishes by itself."
                        : "Not enough movement yet - push it about one tile, then CROSS.";
            } else {
                results[step] = r;
                step++;
                forwardIn = leftIn = turnDeg = 0;
                note = "";
            }
        }
        lastCross = gamepad1.cross;
        lastCircle = gamepad1.circle;
        if (step == 2 && Math.abs(turnDeg) >= AUTO_TURN_DEG) {   // the turn finishes by itself
            results[2] = judge(2);
            step = 3;
            note = "";
        }

        telemetry.addLine("zack odometry check - wheels are NOT powered");
        if (step == 0) telemetry.addLine(">> STEP 1: push the robot FORWARD ~1 tile, straight. Then CROSS.");
        if (step == 1) telemetry.addLine(">> STEP 2: push the robot to ITS LEFT ~1 tile, no turning. Then CROSS.");
        if (step == 2) telemetry.addLine(">> STEP 3: turn the robot LEFT a quarter turn, in place. (It finishes by itself.)");
        if (step == 3) telemetry.addLine(">> DONE. Take a photo of this screen.   (CIRCLE = start over)");
        if (!note.isEmpty()) telemetry.addLine(note);
        if (step < 3) {
            telemetry.addData("Robot thinks it moved", "%s %.1f in,  %s %.1f in",
                    forwardIn >= 0 ? "FORWARD" : "BACKWARD", Math.abs(forwardIn),
                    leftIn >= 0 ? "LEFT" : "RIGHT", Math.abs(leftIn));
            telemetry.addData("Robot thinks it turned", "%s %.0f deg", turnDeg >= 0 ? "LEFT" : "RIGHT", Math.abs(turnDeg));
        }
        telemetry.addData("1 FORWARD push", results[0]);
        telemetry.addData("2 LEFT push", results[1]);
        telemetry.addData("3 LEFT turn", results[2]);
        telemetry.addData("Pinpoint", poseText(now));
        telemetry.addData("Constants.java", configText());
        telemetry.update();
    }

    /** Which pod directions are installed right now (so a photo shows what was being tested). */
    private static String configText() {
        return "X pod " + Constants.localizerConfig.xPodDirection.get()
                + ", Y pod " + Constants.localizerConfig.yPodDirection.get();
    }

    /** What the Pinpoint saw during this step, in words. null = too small to judge yet. */
    private String judge(int which) {
        String raw = String.format("  [fwd %+.1f in, left %+.1f in, turn %+.0f deg]", forwardIn, leftIn, turnDeg);
        if (which == 2) {
            if (Math.abs(turnDeg) < MIN_TURN_DEG) return null;
            return (turnDeg > 0 ? "OK" : "WRONG - heading counts BACKWARDS") + raw;
        }
        double main = which == 0 ? forwardIn : leftIn;       // the number that should have gone UP
        double other = which == 0 ? leftIn : forwardIn;      // the number that should have stayed near 0
        if (Math.abs(main) < MIN_PUSH_IN && Math.abs(other) < MIN_PUSH_IN) return null;
        String mainName = which == 0 ? "forward" : "left";
        if (Math.abs(other) > Math.abs(main)) {
            return "WRONG - this push showed up as " + (which == 0 ? "SIDEWAYS" : "FORWARD/BACK")
                    + " (pods swapped, or pushed the wrong way)" + raw;
        }
        if (Math.abs(other) > 0.5 * Math.abs(main)) {
            return "NOT CLEAR - not a straight push, do it again (CIRCLE)" + raw;
        }
        return (main > 0 ? "OK" : "WRONG - " + mainName + " counts BACKWARDS") + raw;
    }

    private static String poseText(Pose p) {
        return String.format("x %.1f  y %.1f  heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}