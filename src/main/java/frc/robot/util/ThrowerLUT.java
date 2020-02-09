package frc.robot.util;
import frc.robot.Constants.ThrowerPIDs;;

/**
   * Contains the LUT for the thrower
   */
public class ThrowerLUT {
    // if the vision coprocessor can't see the RFT, thrower should be set to this speed
    public static double DEFAULT_RPM = 3000;

    // Known distance to rpm values, determined from our testing
    // first column is inches from the target; second column is RPM that scores from that distance
    // keep the table organized from closest to farthest
    private static final double[][] LUT = {
      // from testing 2/8 with 53 degrees hood angle = 37 degrees departure angle from horizontal
      { 84, 5400 },
      { 300, 5480}
      /* 
      // from testing 2/8 with 43 degrees hood angle = 47 degrees departure angle from horizontal
      { 60, 3580},
      { 72, 3550},
      { 84, 3600},
      { 96, 3625},
      { 108, 3650},
      { 120, 3700},
      { 132, 3900},
      { 156, 4250},
      { 180, 4450},
      { 204, 4625},
      { 228, 4750},
      { 300, 5475}
      */
    };

    /**
     * Use a LUT to map distance as given by the vision processor to RPMs. 
     * Linearly interpolate between known values in our table.
     * @param inches the distance from the thrower to the goal
     */
    public static double distanceToRPMs(double inches){
        if( inches < LUT[0][0]) { // we can't score if we're closer than first  entry in our table
          return DEFAULT_RPM;
        }

        int index = 0;
        while (index < LUT.length - 1) {
          if (inches < LUT[index + 1][0]) {
            break;
          }
          ++index;
        }
    
        if (index + 1 >= LUT.length) {
          if (ThrowerPIDs.TUNE)
            System.err.println("ThrowerLUT: Ran off the end of the ThrowerLUT");
          return LUT[index][1];  // return our fastest throw RPM
        }
        // the slope intercept formulas
        // m = y2 - y1 / x2 - x1
        // b = y - mx
        // y = mx + b
        double m = (LUT[index + 1][1] - LUT[index][1])
            / (LUT[index + 1][0] - LUT[index][0]);
        double b = LUT[index][1] - (m * LUT[index][0]);
        double y = (m * inches) + b;
        return y;
    }
}