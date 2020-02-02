package frc.robot.util;

/**
   * Contains the LUT for the thrower
   */
public class ThrowerLUT {
    // if the vision coprocessor can't see the RFT, thrower should be set to this speed
    public static double DEFAULT_RPM = 500;

    // Known distance to rpm values, determined from our testing
    private static final double[] LUT = {

    };

    /**
     * Use a LUT to map distance as given by the vision processor to RPMs. 
     * Linearly interpolate between values.
     */
    public static double distanceToRPMs(double inches){
      // TODO: implement LUT
      // initial version, just for testing the initial subsystem/command
      double rpm = 700 + 2 * inches;
      return rpm;
    }
}