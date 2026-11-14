package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * "zack red south" - Zack's RED auto for the START OF A MATCH. It goes for TWO HIVE TIPS:
 *     1. shoot the 4 preloads into the AUDIENCE-side red cell (the one that starts UP)   = TIP 1
 *        (an official HIVE tips when 3 POLLEN land on top of the 3 NECTAR already in the cell.
 *         The audience cell drops and the REAR red cell comes up, EMPTY: it needs 8 POLLEN.)
 *     2. FLOWER on the red wall -> drive AROUND the HIVE to the rear side -> shoot   (4 of the 8)
 *     3. FLOWER on the rear wall, right behind that spot -> shoot                   (8 of the 8) = TIP 2
 *     4. GARDEN in the audience/red corner -> back to the same spot -> shoot   (spare pollen, only if there is time)
 *     5. park in the RED LOADING ZONE
 *   Whatever it is doing, it leaves for PARK when the time is nearly up. If that happens after it
 *   has collected, it parks with the pollen still on board, ready for the start of TELEOP.
 *   Everything after step 1 is shot at the REAR cell. If fewer than 3 preloads go in, that cell is
 *   still DOWN and those shots are wasted - the preloads have to land.
 * Same start spot as the 16010 Astra Machina video.
 * It is "zack blue north" turned half a turn around the middle of the field: every number here
 * is 141.5 minus the blue one. NOT RUN ON THE ROBOT YET (Nov 13).
 *
 * This file only holds WHERE things are on the field. What the robot does is in ZackAuto.java.
 * THE ORDER OF THE LINES IN settings() IS THE ORDER THE ROBOT DOES THEM.
 *
 * WHERE TO PUT THE ROBOT  (this is the ONLY thing the robot goes by - see ZackAuto.java)
 *   Back flat against the AUDIENCE wall, centred on the RED-side tile that touches the centre
 *   seam: the same gap from the robot's sides to the tile seam on each side. Front and Limelight
 *   facing the HIVE.
 *
 * WHERE THE NUMBERS COME FROM
 *   FTC Competition Manual Figures 9-2, 9-3 and 9-5 for WHAT is on which tile, and the Pedro
 *   Pathing Visualizer's BIOBUZZ field for the inches (floor = 141.5 in wall to wall).
 *   - FLOWERS sit on tile seams: red wall 46.75 in from the audience wall; rear wall 46.75 in
 *     from the red wall.
 *   - Red GARDEN: against the AUDIENCE wall, the 23 in next to the red wall. The 4 pollen lie in
 *     a line touching the audience wall, starting in the corner.
 *   - Red LOADING ZONE: against the red wall, on the 2nd tile from the rear wall, 11 in deep.
 *   - Rule G402: in AUTO the red side is columns A, B, C. Every spot here keeps the whole robot
 *     on the red side of the centre line (x = 70.75).
 *
 * Field numbers: x = inches from the inside of the RED wall, y = inches from the inside of the
 * AUDIENCE wall, heading 90 = facing away from the audience, 180 = facing the red wall,
 * 270 = facing the audience.
 */
@Autonomous(name = "zack red south", group = "Autonomous")
public class ZackRedSouth extends ZackAuto {
    @Override
    protected void settings() {
        title = "zack red south";
        alliance = HiveTarget.Alliance.RED;
        start(58.75, 9.3638, 90);       // back on the audience wall, centred on the tile

        shoot(58.75, 17.9775, 90);      // AUDIENCE-side spot, 8.6 in off the wall: ONLY the 4 preloads go from here = TIP 1
        parkVia(18.75, 30.0);           //   (if it ever has to park from this spot: bend towards the red wall, AROUND the HIVE frame)

        shoot(60.0, 123.5225, 270);     // REAR-side spot, 8.6 in off that wall: EVERYTHING below this line is shot from here
        flower(0.0, 46.75, 180);        // 1st: FLOWER on the red wall, on the tile seam. 180 = robot faces the red wall.
        around(22.5, 81.5,  22.5, 111.5,  37.5, 119.5);     //   then the drive AROUND the red end of the HIVE to the rear side
        flower(46.75, 141.5, 90);       // 2nd: FLOWER on the rear wall, right behind that spot. 90 = robot faces the rear wall.
        garden(10.0, 0.0, 270);         // 3rd (spare): GARDEN. Nose-first into the audience wall, 1 in clear of the red wall
        driveVia(37.5, 119.5,  15.5, 116.5,  23.5, 81.5,  23.5, 46.5,  29.5, 21.5,  17.5, 29.5);   //   the drive to it: back AROUND the red end of the HIVE, between the HIVE and the red-wall FLOWER
        around(37.5, 22.5,  22.5, 46.5,  22.5, 71.5,  22.5, 95.5,  22.5, 121.5,  37.5, 119.5);   //   and the same way back
        lateParkVia(36.75, 36.0);       //   (if time runs out at the GARDEN: park by going around the red-wall FLOWER, not through it)

        park(10.5, 106.75);             // middle of the red LOADING ZONE (tile A5), 1.5 in off the red wall
    }
}