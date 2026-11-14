package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/*
 * Limelight Test
 * Shows every AprilTag the Limelight sees: its ID, how far left/right it is
 * (tx, for aiming the turret later) and how far away it is (for flywheel speed).
 *
 * No motors or servos move in this OpMode.
 */
@TeleOp(name = "Limelight Test", group = "Test")
public class LimelightTest extends OpMode {

    // ======================= SETTINGS =======================
    static final String LIMELIGHT_NAME = "limelight";   // must match Driver Station config
    static final int APRILTAG_PIPELINE = 0;             // pipeline slot set to AprilTag in the Limelight web page
    static final double METERS_TO_INCHES = 39.3701;
    // ========================================================

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        telemetry.addData("Status", "Limelight started. Press Start.");
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Tags", "none seen");
            telemetry.update();
            return;
        }

        if (result.getPipelineIndex() != APRILTAG_PIPELINE) {
            telemetry.addData("Warning", "Limelight is on pipeline %d, expected %d",
                    result.getPipelineIndex(), APRILTAG_PIPELINE);
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        telemetry.addData("Tags seen", tags.size());

        for (LLResultTypes.FiducialResult tag : tags) {
            double tx = tag.getTargetXDegrees();   // + = tag is to the right, - = left
            double ty = tag.getTargetYDegrees();   // + = tag is above center

            String distanceText = "n/a";
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose != null) {
                Position p = pose.getPosition();   // meters, from the camera to the tag
                double meters = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                distanceText = String.format("%.1f in", meters * METERS_TO_INCHES);
            }

            telemetry.addData("Tag " + tag.getFiducialId(),
                    "tx %.1f deg | ty %.1f deg | dist %s", tx, ty, distanceText);
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
