package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * "zack blue north" - Zack's BLUE auto for the START OF A MATCH. It goes for TWO HIVE TIPS:
 *     1. shoot the 4 preloads into the REAR blue cell (the one that starts UP)        = TIP 1
 *        (an official HIVE tips when 3 POLLEN land on top of the 3 NECTAR already in the cell.
 *         The rear cell drops and the AUDIENCE-side blue cell comes up, EMPTY: it needs 8 POLLEN.)
 *     2. FLOWER on the blue wall -> drive AROUND the HIVE to the audience side -> shoot   (4 of the 8)
 *     3. FLOWER on the audience wall, right behind that spot -> shoot                    (8 of the 8) = TIP 2
 *     4. GARDEN in the rear/blue corner -> back to the same spot -> shoot   (spare pollen, only if there is time)
 *     5. park in the BLUE LOADING ZONE
 *   Whatever it is doing, it leaves for PARK when the time is nearly up. If that happens after it
 *   has collected, it parks with the pollen still on board, ready for the start of TELEOP.
 *   Everything after step 1 is shot at the AUDIENCE-side cell. If fewer than 3 preloads go in, that
 *   cell is still DOWN and those shots are wasted - the preloads have to land.
 * It is "zack red south" turned half a turn around the middle of the field:
 * every number here is 141.5 minus the red one.
 *
 * This file only holds WHERE things are on the field. What the robot does is in ZackAuto.java.
 * THE ORDER OF THE LINES IN settings() IS THE ORDER THE ROBOT DOES THEM.
 *
 * WHERE TO PUT THE ROBOT  (this is the ONLY thing the robot goes by - see ZackAuto.java)
 *   Back flat against the REAR wall (the one far from the audience), centred on the BLUE-side
 *   tile that touches the centre seam: the same gap from the robot's sides to the tile seam on
 *   each side. Front and Limelight facing the HIVE.
 *
 * WHERE THE NUMBERS COME FROM
 *   FTC Competition Manual Figures 9-2, 9-3 and 9-5 for WHAT is on which tile, and the Pedro
 *   Pathing Visualizer's BIOBUZZ field for the inches (floor = 141.5 in wall to wall).
 *   - FLOWERS sit on tile seams: blue wall 94.75 in from the audience wall; audience wall
 *     94.75 in from the red wall.
 *   - Blue GARDEN: against the REAR wall, the 23 in next to the blue wall. The 4 pollen lie in a
 *     line touching the rear wall, starting in the corner.
 *   - Blue LOADING ZONE: against the blue wall, on the 2nd tile from the audience wall, 11 in deep.
 *   - Rule G402: in AUTO the blue side is columns D, E, F. Every spot here keeps the whole robot
 *     on the blue side of the centre line (x = 70.75).
 *
 * Field numbers: x = inches from the inside of the RED wall, y = inches from the inside of the
 * AUDIENCE wall, heading 0 = facing the blue wall, 90 = facing away from the audience,
 * 270 = facing the audience.
 */
@Autonomous(name = "zack blue north", group = "Autonomous")
public class ZackBlueNorth extends ZackAuto {
    @Override
    protected void settings() {
        title = "zack blue north";
        alliance = HiveTarget.Alliance.BLUE;
        start(82.75, 132.1362, 270);    // back on the rear wall, centred on the tile

        shoot(82.75, 123.5225, 270);    // REAR-side spot, 8.6 in off the wall: ONLY the 4 preloads go from here = TIP 1
        parkVia(122.75, 111.5);         //   (if it ever has to park from this spot: bend towards the blue wall, AROUND the HIVE frame)

        shoot(81.5, 17.9775, 90);       // AUDIENCE-side spot, 8.6 in off that wall: EVERYTHING below this line is shot from here
        flower(141.5, 94.75, 0);        // 1st: FLOWER on the blue wall, on the tile seam. 0 = robot faces the blue wall.
        around(119, 60,  119, 30,  104, 22);     //   then the drive AROUND the blue end of the HIVE to the audience side
        flower(94.75, 0.0, 270);        // 2nd: FLOWER on the audience wall, right behind that spot. 270 = robot faces the audience.
        garden(131.5, 141.5, 90);       // 3rd (spare): GARDEN. Nose-first into the rear wall, 1 in clear of the blue wall
        driveVia(104, 22,  126, 25,  118, 60,  118, 95,  112, 120,  124, 112);   //   the drive to it: back AROUND the blue end of the HIVE, between the HIVE and the blue-wall FLOWER
        around(104, 119,  119, 95,  119, 70,  119, 46,  119, 20,  104, 22);   //   and the same way back
        lateParkVia(104.75, 105.5);     //   (if time runs out at the GARDEN: park by going around the blue-wall FLOWER, not through it)

        park(131.0, 34.75);             // middle of the blue LOADING ZONE (tile F2), 1.5 in off the blue wall
    }
}