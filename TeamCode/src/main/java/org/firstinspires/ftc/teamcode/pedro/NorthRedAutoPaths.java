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

@Autonomous(name = "NorthRedAuto Paths", group = "Autonomous")
public class NorthRedAutoPaths extends LinearOpMode {

    private Follower follower;
    private Flywheel flywheel;
    private Stopper stopper;
    private Battery battery;
    private DcMotorEx intake;
    private ServoImplEx flowerArm;
    private boolean flywheelRunning = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.5347, 134.6362, 260);
    private final Pose shootPose = poseFactory.of(59.8629, 126.0225, 260);
    private final Pose intakePose = poseFactory.of(46.6975, 131.5454, 90);
    private final Pose intakeControl1 = poseFactory.of(41.1547, 86.5795, 0);
    private final Pose shoot_2 = poseFactory.of(49.1694, 8.7949, 90);
    private final Pose shoot_2Control1 = poseFactory.of(0, 56.1613, 0);
    private final Pose intakegarden = poseFactory.of(4.0682, 9.7386, 270);
    private final Pose intakegardenControl1 = poseFactory.of(31.0704, 45.5843, 0);
    private final Pose shoot_3 = poseFactory.of(56.3119, 9.2474, 90);
    private final Pose intakeflower = poseFactory.of(11.3325, 46.6388, 180);
    private final Pose intakeflowerControl1 = poseFactory.of(49.7671, 31.0819, 0);
    private final Pose shoot_4 = poseFactory.of(59.8208, 16.3943, 90);
    private final Pose park = poseFactory.of(9.9346, 119.7682, 0);
    private final Pose parkControl1 = poseFactory.of(83.1545, 141.5, 0);

    public Command autoRoutine() {
        return sequential(
                instant(() -> flywheelRunning = true),
                instant(() -> flowerArm.setPosition(0.240)),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                parallel(
                        follow(follower, intakePath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

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

                parallel(
                        follow(follower, intakegardenPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                parallel(
                        follow(follower, shoot3Path()),
                        instant(() -> flywheelRunning = true)
                ),
                instant(() -> sleep(800)),
                instant(() -> stopper.update(true)),
                instant(() -> sleep(500)),
                instant(() -> stopper.startClosed()),
                instant(() -> flywheelRunning = false),
                instant(() -> flowerArm.setPosition(0.810)),

                parallel(
                        follow(follower, intakeflowerPath()),
                        sequential(
                                instant(() -> flowerArm.setPosition(0.240)),
                                instant(() -> intake.setPower(1.0))
                        )
                ),
                instant(() -> intake.setPower(0.0)),

                parallel(
                        follow(follower, shoot4Path()),
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
        follower.setPose(start);
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

    public Path shootPath() { return Paths.line(start, shootPose).linear(start, shootPose); }
    public Path intakePath() { return Paths.curve(shootPose, intakeControl1, intakePose).linear(shootPose, intakePose); }
    public Path shoot2Path() { return Paths.curve(intakePose, shoot_2Control1, shoot_2).linear(intakePose, shoot_2); }
    public Path intakegardenPath() { return Paths.curve(shoot_2, intakegardenControl1, intakegarden).linear(shoot_2, intakegarden); }
    public Path shoot3Path() { return Paths.line(intakegarden, shoot_3).linear(intakegarden, shoot_3); }
    public Path intakeflowerPath() { return Paths.curve(shoot_3, intakeflowerControl1, intakeflower).linear(shoot_3, intakeflower); }
    public Path shoot4Path() { return Paths.line(intakeflower, shoot_4).linear(intakeflower, shoot_4); }
    public Path parkPath() { return Paths.curve(shoot_4, parkControl1, park).linear(shoot_4, park); }
}