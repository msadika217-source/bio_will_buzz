package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name = "SouthBlueAuto", group = "Autonomous")
public class SouthBlueAutoPaths extends LinearOpMode {

    private Follower follower;
    private Flywheel flywheel;
    private Stopper stopper;
    private Battery battery;
    private DcMotorEx intake;
    private ServoImplEx flowerArm;
    private boolean flywheelRunning = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose tip2 = poseFactory.of(82.0327, 8.8398, 90);
    private final Pose shoottointake = poseFactory.of(130.5154, 96.4571, 0);
    private final Pose shoottointakeControl1 = poseFactory.of(126.2033, 59.4505, 0);
    private final Pose point2 = poseFactory.of(85.7246, 134.0072, -129.5008);
    private final Pose point2Control1 = poseFactory.of(61.6288, 104.7774, 0);
    private final Pose intakegarden = poseFactory.of(141.5, 135.0743, -178.904);
    private final Pose shootPose = poseFactory.of(82.8685, 116.7863, -105);
    private final Pose intakeflower = poseFactory.of(97.6227, 7.3798, 270);
    private final Pose intakeflowerControl1 = poseFactory.of(57.6564, 59.34, 0);
    private final Pose shoot_2 = poseFactory.of(93.0658, 23.0728, 90);
    private final Pose park = poseFactory.of(138.6815, 24.3254, -178.427);

    public Command autoRoutine() {
        return sequential(
                follow(follower, shoottointakePath()),
                follow(follower, path2()),

                // Intake garden
                parallel(
                        follow(follower, intakegardenPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // Shoot 1
                parallel(
                        follow(follower, shootPath()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                // Intake flower
                parallel(
                        follow(follower, intakeflowerPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                // Shoot 2
                parallel(
                        follow(follower, shoot2Path()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                follow(follower, parkPath())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        battery = new Battery(hardwareMap);
        stopper = new Stopper(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flowerArm = hardwareMap.get(ServoImplEx.class, AxonTeleOp.FLOWER_ARM_NAME);
        flowerArm.setPwmRange(new PwmControl.PwmRange(500, 2500));
        flowerArm.setPosition(0.810);

        stopper.startClosed();
        follower.setPose(tip2);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            battery.update();
            flywheel.update(flywheelRunning, AxonTeleOp.SHOOT_RPM, battery);
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.update();
        }
    }

    public Path shoottointakePath() { return Paths.curve(tip2, shoottointakeControl1, shoottointake).linear(tip2, shoottointake); }
    public Path path2() { return Paths.curve(shoottointake, point2Control1, point2).reverseTangent(); }
    public Path intakegardenPath() { return Paths.line(point2, intakegarden).reverseTangent(); }
    public Path shootPath() { return Paths.line(intakegarden, shootPose).linear(intakegarden, shootPose); }
    public Path intakeflowerPath() { return Paths.curve(shootPose, intakeflowerControl1, intakeflower).linear(shootPose, intakeflower); }
    public Path shoot2Path() { return Paths.line(intakeflower, shoot_2).linear(intakeflower, shoot_2); }
    public Path parkPath() { return Paths.line(shoot_2, park).reverseTangent(); }
}