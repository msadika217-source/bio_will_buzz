package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.pedro.Constants.drivetrainConfig;
import static org.firstinspires.ftc.teamcode.pedro.Constants.foresightConfig;
import static org.firstinspires.ftc.teamcode.pedro.Constants.localizerConfig;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Tuning {

    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, drivetrainConfig));
    }



    @Tuner
    public static Procedure tests() {
        return new Tests(
                (hardwareMap) -> new Mecanum(hardwareMap, drivetrainConfig),
                (hardwareMap) -> new PinpointLocalizer(hardwareMap, localizerConfig),
                () -> new Foresight(foresightConfig)
        );
    }

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}