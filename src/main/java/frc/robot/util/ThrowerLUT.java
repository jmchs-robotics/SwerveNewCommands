package frc.robot.util;
import frc.robot.Constants.ThrowerPIDs;;

/**
   * Contains the LUT for the thrower
   */
public class ThrowerLUT {
    // if the vision coprocessor can't see the RFT, thrower should be set to this speed
    public static double DEFAULT_RPM = 5050;  // 5050 should score from the Initiation Line

    // Known distance to rpm values, determined from our testing
    // first column is inches from the target; second column is RPM that scores from that distance
    // keep the table organized from closest to farthest
    private static final double[][] LUT = {
      // from testing 2/8 with 53 degrees hood angle = 37 degrees departure angle from horizontal
      {0, DEFAULT_RPM}, // Default RPM
      {24, 4600},
      {48, 4800},
      {72, 5100},
      {84, 5400 },
      {96, 5100},
      {108, 5200},
      {132, 4900},
      {156, 4700},
      {300, 5480}
      

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

        int index = LUT.length - 2; // Start at the highest meaningful index for a right-handed discrete derivative
        while(inches < LUT[index][0]){ index--; } // iterate down. Safe if the lowest index contains {0, DEFAULT_RPM}
        // No need to check if we're off the deep end, because the worst that could happen
        // is the motor gets set to full forward. This would replace the loop & following if-statement.
        
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