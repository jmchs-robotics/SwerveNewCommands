/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
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
  public static final class ThrowerMotors {
    public static final int LEFT_MOTOR = 21;
    public static final int RIGHT_MOTOR = 22;
    public static final boolean INVERT_FOLLOWER = false;
  }

  /**
   * Contains the physical and PID constants of the Thrower subsystem
   */
  public static final class ThrowerConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double FEED_FORWARD = 0;
    public static final double kIz = 0;
    public static final double MIN_OUTPUT = -1;
    public static final double MAX_OUTPUT = 1;
    public static final double GEAR_RATIO_MOTOR_TO_WHEEL = 40/48; // 40T pinion, 48T wheel gear
    public static final int UPDATE_RATE = 5;  // 200 Hz update rate leader -> follower. Default 10ms

    public static final boolean TUNE = false;
  }

  /**
   * Contains the LUT for the thrower
   */
  public static final class ThrowerLUT {
    // Known distance to rpm values, where
    // index = [some mathematical formula, e.g. (int)distance/20]
    private static final double[] LUT = {

    };

    /**
     * Use a LUT to map distance to RPMs. Linearly approximate between values.
     */
    public static double distanceToRPMs(double inches){
      // TODO: implement LUT
      return inches;
    }
  }

  public static final class HopperConstants {
    public static final int MAIN_MOTOR_ID = 15;
    public static final double ENCODER_TICKS_TO_REVOLUTION = 4096.0;

    public static final Port COLOR_SENSOR_PORT = I2C.Port.kMXP;
	  public static final int BALL_LOADED_THRESHOLD = 1500;

    public static final TalonSRXConfiguration GetMainMotorConfiguration(){
      TalonSRXConfiguration config = new TalonSRXConfiguration();

      /* Talon SRX */
      config.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
      config.primaryPID.selectedFeedbackCoefficient = 0.328293;
      config.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.Analog;
      config.auxiliaryPID.selectedFeedbackCoefficient = 0.877686;
      config.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
      config.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
      config.sum0Term = FeedbackDevice.QuadEncoder;
      config.sum1Term = FeedbackDevice.RemoteSensor0;
      config.diff0Term = FeedbackDevice.RemoteSensor1;
      config.diff1Term = FeedbackDevice.PulseWidthEncodedPosition;
      config.peakCurrentLimit = 20;
      config.peakCurrentDuration = 200;
      config.continuousCurrentLimit = 30;
      config.openloopRamp = 1.023000;
      config.closedloopRamp = 1.705000;
      config.peakOutputForward = 0.939394;
      config.peakOutputReverse = -0.289345;
      config.nominalOutputForward = 0.739980;
      config.nominalOutputReverse = -0.119257;
      config.neutralDeadband = 0.199413;
      config.voltageCompSaturation = 9.296875;
      config.voltageMeasurementFilter = 16;
      config.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
      config.velocityMeasurementWindow = 8;
      config.forwardLimitSwitchDeviceID = 6;
      config.reverseLimitSwitchDeviceID = 5;
      config.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
      config.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
      config.forwardSoftLimitThreshold = 2767;
      config.reverseSoftLimitThreshold = -1219;
      config.forwardSoftLimitEnable = false;
      config.reverseSoftLimitEnable = false;

      config.slot0.kP = 504.000000;
      config.slot0.kI = 5.600000;
      config.slot0.kD = 0.200000;
      config.slot0.kF = 19.300000;
      config.slot0.integralZone = 900;
      config.slot0.allowableClosedloopError = 217;
      config.slot0.maxIntegralAccumulator = 254.000000;
      config.slot0.closedLoopPeakOutput = 0.869990;
      config.slot0.closedLoopPeriod = 33;

      config.auxPIDPolarity = true;
      config.remoteFilter0.remoteSensorDeviceID = 0;
      config.remoteFilter0.remoteSensorSource = RemoteSensorSource.Off;
      config.remoteFilter1.remoteSensorDeviceID = 0;
      config.remoteFilter1.remoteSensorSource = RemoteSensorSource.Off;
      config.motionCruiseVelocity = 37;
      config.motionAcceleration = 3;
      config.motionProfileTrajectoryPeriod = 11;
      config.feedbackNotContinuous = true;
      config.remoteSensorClosedLoopDisableNeutralOnLOS = false;
      config.clearPositionOnLimitF = true;
      config.clearPositionOnLimitR = true;
      config.clearPositionOnQuadIdx = false;
      config.limitSwitchDisableNeutralOnLOS = true;
      config.softLimitDisableNeutralOnLOS = false;
      config.pulseWidthPeriod_EdgesPerRot = 9;
      config.pulseWidthPeriod_FilterWindowSz = 32;
      config.customParam0 = 3;
      config.customParam1 = 5;

      return config;
    }
  }

  /**
   * Constants for the color sensor targets, in RGB arrays
   */
  public static final class ColorTargets {
    public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  }

  /**
   * Constants for the control panel subsystem
   */
  public static final class ControlPanelActuators {
    public static final int SOLENOID_FORWARD_CHANNEL = 0;
    public static final int SOLENOID_REVERSE_CHANNEL = 1;
    public static final int SPINNER_ID = 16;
    public static final Port COLOR_SENSOR_PORT = I2C.Port.kOnboard;
  }

  /**
   * Constants for the intake subsystem
   */
  public static final class IntakeActuators {
    public static final int SOLENOID_FORWARD_CHANNEL = 2;
    public static final int SOLENOID_REVERSE_CHANNEL = 3;
    public static final int INTAKE_MOTOR_PWM_ID = 0;
  }

  /**
   * Constants for the climber subsystem
   */
  public static final class ClimberActuators {
    public static final int SOLENOID_FORWARD_CHANNEL = 4;
    public static final int SOLENOID_REVERSE_CHANNEL = 5;
    public static final int WINCH_MOTOR = 17;
  }
}
