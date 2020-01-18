/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean SHOW_DEBUG_VISION = false;
  
  // Declare inner public classes to segment constants

  /**
  * Contains the CAN IDs of the Drivetrian SparkMax motors
  */ 
  public static final class DrivetrainMotors {
    public static final int FRONT_LEFT_DRIVE = 25;
    public static final int FRONT_LEFT_ANGLE = 23;

    public static final int FRONT_RIGHT_ANGLE = 34;
    public static final int FRONT_RIGHT_DRIVE = 32;

    public static final int BACK_LEFT_DRIVE = 26;
    public static final int BACK_LEFT_ANGLE = 24;

    public static final int BACK_RIGHT_ANGLE = 33;
    public static final int BACK_RIGHT_DRIVE = 31;
  }

  public static final class DrivetrainConstants {
    // set for SwerveyJr 191207
    public static final double WHEELBASE = 22;
    public static final double TRACKWIDTH = 19.5;
    public static final double WIDTH = 25.75;
    public static final double LENGTH = 28;

    // PID constants for swerve modules
    public static final double ANGLE_kP = 3.0;
    public static final double ANGLE_kI = 0.0;
    public static final double ANGLE_kD = 0.0;    

    // PID constants for whole-drivetrain strafe control
    public static final double STRAFE_kP = 0.01;
    public static final double STRAFE_kI = 0.0;
    public static final double STRAFE_kD = 0.0;

    // PID constants for whole-drivetrain strafe control
    public static final double FORWARD_kP = 0.01;
    public static final double FORWARD_kI = 0.0;
    public static final double FORWARD_kD = 0.0;

    // PID constants for whole-drivetrain strafe control
    public static final double ROTATION_kP = 0.01;
    public static final double ROTATION_kI = 0.0;
    public static final double ROTATION_kD = 0.0;
  }

  /**
   * Contains the CAN IDs of the elevator SparkMax motors
   */
  public static final class CarriageMotors {
    public static final int LEFT_MOTOR = 21;
    public static final int RIGHT_MOTOR = 22;
  }

  /**
   * Constants for the color sensor targets, in RGB arrays
   */
  public static final class ColorTargets {
    public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    public static final double cpSpinnerSpeed = 0.5;
  }

  /**
   * Constants for the control panel
   */
  public static final class ControlPanelActuators {
    public static final int motorCanID = 11;
    public static final int soleniodFirst = 12;
    public static final int soleniodSecond = 13;
  }

  public static final class ThrowerMotor {
    public static final int throwerMotorID = 14;
    public static final int throwerFollowerMotorID = 15;
    //public static final double throwerMotorSpeed = 0.7;//arbitrary number
    public static final boolean INVERT_FOLLOWER = false;
  }

  public static final class ThrowerPIDs {
    public static final double kP = 2e-4;
    public static final double kI = 1e-6;
    public static final double kD = 0.0;
    public static final double FEED_FORWARD = 0.0; // minimum useful 1/18 seems to be 0.001
    public static final double kIz = 0;
    public static final double MIN_OUTPUT = -1;
    public static final double MAX_OUTPUT = 1;
    public static final double GEAR_RATIO_MOTOR_TO_WHEEL = 35.0 / 35.0; // 40T pinion, 48T wheel gear
    public static final int UPDATE_RATE = 5; // msecs  200 Ht update rate leader -> folloer. Default 10ms

    public static final boolean TIME = false;

    public static final boolean TUNE = true;
  
  }

    public static final class HopperConstants {
      public static final int HOPPER_MOTOR_ID = 6;
  }
}
