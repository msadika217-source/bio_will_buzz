package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("backRight");
        c.frontRightName.set("backLeft");
        c.backLeftName.set("frontRight");
        c.backRightName.set("frontLeft");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    //* reverse the motors if they don't work *//
    //*RFRF was the old one*//

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("odoPods");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Nov 13 (Zack): "zack odometry check" showed the two pods were in each other's ports
        // (a FORWARD push counted as sideways, a LEFT push counted as forward), so the pod CABLES
        // were swapped at the Pinpoint. Now:
        //   - X pod = the pod that rolls when the robot drives FORWARD (in the X port).
        //   - Y pod = the pod that rolls when the robot moves SIDEWAYS (in the Y port).
        // Directions: with X REVERSED / Y FORWARD the check said BOTH counted backwards, so both
        // were flipped to X FORWARD / Y REVERSED.
        // If "zack odometry check" says a direction is BACKWARDS, flip that pod's FORWARD/REVERSED.
        // Nov 13 (Zack): the Pinpoint itself was also bolted on its EDGE (sticker facing out the back).
        // It must lie FLAT, sticker UP. On its edge the heading was wrong (straight pushes read as
        // 20-80 deg turns). It was re-mounted flat; pushes now read 1-6 deg.
        // Offsets (Pinpoint Tuner run AFTER the re-mount: printed x -2.2213, y +4.0961):
        //   xPodOffset = -2.22 -> the FORWARD pod sits about 2.2 in to the RIGHT of the robot's centre.
        //   yPodOffset = -4.10 -> the SIDEWAYS pod sits about 4.1 in BEHIND the robot's centre.
        // The tuner printed y as +4.10, but it measured with the Y pod set to FORWARD (it keeps
        // getting the Y direction wrong), and that flips the sign of the y offset.
        // CHECK: run "zack odometry check" and turn the robot in place on the floor. In the
        // "3 LEFT turn" line, fwd and left should both be under about 1 in. If "left" is around
        // 8 in instead, change yPodOffset to +4.10.
        // (History: before the cable swap the file had xPodOffset 3.2914, yPodOffset 1.6156.)
        c.xPodOffset.set(-2.22);
        c.yPodOffset.set(-4.10);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    //reverse and forward = FORWARD GOOD, STRAFING BAD  //

    // Foresight numbers MEASURED ON THIS ROBOT by the Foresight Tuner (AutoTune) on Nov 13, 2026,
    // after the odometry was fixed (pod cables swapped, Pinpoint re-mounted flat, offsets measured).
    // Before this, every number here was the Pedro example robot's, not ours.
    // Re-run the Foresight Tuner if the robot's weight, wheels, drive motors or gearing change.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3736172233916813);
                Controller secondaryTranslationalForward = Controller.proportional(0.13804158080715334);
                Controller primaryTranslationalLateral = Controller.proportional(0.5614035819125254);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20742362253669247);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.013108017587775142));
                c.brake.set(Controller.proportionalFeedforward(0.01114181494960887));

                c.headingFeedback.set(Controller.proportional(5.1258249369253965));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04937324068355044, 0.004651939302078673));

                c.linearBrakeCoefficients.set(Matrix.diag(0.1047090778701528, 0.08730084899176153));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0016833690488841932, 0.0013298429280909803));

                c.maxAchievableForwardVelocity.set(73.39053669062407);
                c.maxAchievableStrafeVelocity.set(57.38816004666141);
                c.naturalForwardDeceleration.set(29.3927548190953);
                c.naturalStrafeDeceleration.set(65.70732264060476);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }




//im gonna jump off a cliff it ts don't flipping work//


}