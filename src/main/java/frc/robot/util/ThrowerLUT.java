package frc.robot.util;
import frc.robot.Constants.ThrowerPIDs;;

/**
   * Contains the LUT for the thrower
   */
public class ThrowerLUT {
    // if the vision coprocessor can't see the RFT, thrower should be set to this speed
    public static double DEFAULT_RPM = 4550;  // from testing 3/2, and estimating, hopefully scores at autonomous dist.

    // Known distance to rpm values, determined from our testing
    // first column is inches from the target as reported by the vision processor; 
    //   second column is RPM that scores from that distance
    // keep the table organized from closest to farthest
    private static final double[][] LUT = {
      // from testing 3/2 with 53 degrees hood angle = 37 degrees departure angle from horizontal
      //And fresh Battery :)
      {0, DEFAULT_RPM}, // Default RPM
      {132, 4300},
      {149, 4350},
      {173, 4600},
      {197, 4675},
      {214, 4825},
      {239, 5065},
      {300, 5480} // from testing 2/8. Go for it!
      
      /* testing 2/29 with 53 degrees hood angle = 37 degrees departure angle from horizontal
      // from 
      {125, 4200},
      {143, 4400},
      {163, 4625},
      {192, 4650},
      {217, 4800},
      {232, 5000},
      {266, 5150}
      
      
     
     /*
     // from testing 2/8 with 53 degrees hood angle = 37 degrees departure angle from horizontal
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


    private static final double[][] llLUT = {
        // from testing 3/2 with 53 degrees hood angle = 37 degrees departure angle from horizontal
        //And fresh Battery :)
        {-15, 5500},
        {-10, 5000},
        {-5, 4000},
        {0, DEFAULT_RPM}, // Default RPM
        {5, 4000},
        {10, 5000},

    };

    public static double llAngleToRPMs( double angle) {
        int index = llLUT.length - 2; // Start at the highest meaningful index for a right-handed discrete derivative
        while(angle < llLUT[index][0]){ index--; } // iterate down. Safe if the lowest index contains {0, DEFAULT_RPM}
        // No need to check if we're off the deep end, because the worst that could happen
        // is the motor gets set to full forward. This would replace the loop & following if-statement.
        
        // the slope intercept formulas
        // m = y2 - y1 / x2 - x1
        // b = y - mx
        // y = mx + b
        double m = ( llLUT[index + 1][1] - llLUT[index][1])
            / ( llLUT[index + 1][0] - llLUT[index][0]);
        double b = llLUT[index][1] - (m * llLUT[index][0]);
        double y = (m * angle) + b;
        return y;
    }
} 
