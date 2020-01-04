/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Declare inner public classes to segment constants

  /**
  * Contains the CAN IDs of the Drivetrian SparkMax motors
  */ 
  public final class DrivetrainMotors {
    public static final int FRONT_LEFT_DRIVE = 25;
    public static final int FRONT_LEFT_ANGLE = 23;

    public static final int FRONT_RIGHT_ANGLE = 34;
    public static final int FRONT_RIGHT_DRIVE = 32;

    public static final int BACK_LEFT_DRIVE = 26;
    public static final int BACK_LEFT_ANGLE = 24;

    public static final int BACK_RIGHT_ANGLE = 33;
    public static final int BACK_RIGHT_DRIVE = 31;
  }

  /**
   * Contains the CAN IDs of the elevator SparkMax motors
   */
  public final class CarriageMotors {
    public static final int LEFT_MOTOR = 21;
    public static final int RIGHT_MOTOR = 22;
  }
}
