package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * "zack blue" - exactly the same as "zack code", but the Limelight only uses the BLUE HIVE (tags 38-45),
 * and aims at whichever BLUE CELL is facing up (it switches automatically after each TIP).
 * All settings and controls live in AxonTeleOp.java - change them there and both colors update.
 */
@TeleOp(name = "zack blue", group = "Drive")
public class Blue extends AxonTeleOp {
    @Override
    protected HiveTarget.Alliance lockedAlliance() {
        return HiveTarget.Alliance.BLUE;
    }
}
