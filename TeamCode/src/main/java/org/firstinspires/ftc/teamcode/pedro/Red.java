package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * "zack red" - exactly the same as "zack code", but the Limelight only uses the RED HIVE (tags 30-37),
 * and aims at whichever RED CELL is facing up (it switches automatically after each TIP).
 * All settings and controls live in AxonTeleOp.java - change them there and both colors update.
 */
@TeleOp(name = "zack red", group = "Drive")
public class Red extends AxonTeleOp {
    @Override
    protected HiveTarget.Alliance lockedAlliance() {
        return HiveTarget.Alliance.RED;
    }
}
