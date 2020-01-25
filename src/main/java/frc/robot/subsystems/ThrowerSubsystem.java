/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ThrowerMotor;
import frc.robot.Constants.ThrowerPIDs;

public class ThrowerSubsystem extends SubsystemBase {
  private CANSparkMax m_Thrower;
  private CANSparkMax m_Follower;
  private CANPIDController m_throwController;
  private CANEncoder m_throwEncoder;

  private double kP = ThrowerPIDs.kP;
  private double kI = ThrowerPIDs.kI;
  private double kD = ThrowerPIDs.kD;
  private double kIz = ThrowerPIDs.kIz;
  private double kff = ThrowerPIDs.FEED_FORWARD;
  private double kMaxOutput = ThrowerPIDs.MAX_OUTPUT;
  private double kMinOutput = ThrowerPIDs.MIN_OUTPUT;
  
  private double m_setpoint = 0;
  private double ii = 0;

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ThrowerSubsystem() {
    m_Thrower = new CANSparkMax(ThrowerMotor.throwerMaxID, MotorType.kBrushless);
    m_Follower = new CANSparkMax(ThrowerMotor.throwerFollowerMaxID, MotorType.kBrushless);

    // reset controllers
    m_Thrower.restoreFactoryDefaults();
    m_Thrower.clearFaults();
    m_Follower.restoreFactoryDefaults();
    m_Follower.clearFaults();

    m_Thrower.setIdleMode(IdleMode.kCoast);
    m_Follower.setIdleMode(IdleMode.kCoast);

    m_Follower.follow(m_Thrower, ThrowerMotor.INVERT_FOLLOWER);

    m_throwController = m_Thrower.getPIDController();
    m_throwEncoder = m_Thrower.getEncoder();

    //Set PID Constants
    m_throwController.setP(kP);
    m_throwController.setD(kD);
    m_throwController.setI(kI);
    m_throwController.setIZone(kIz);
    m_throwController.setFF(kff);
    
    m_throwController.setOutputRange(kMinOutput,kMaxOutput);

    //Everything on the smartDashboard:
    if(ThrowerPIDs.TUNE){
      SmartDashboard.putNumber("Thrower desired wheel RPM", m_setpoint);
      SmartDashboard.putNumber("Thrower P", kP);
      SmartDashboard.putNumber("Thrower I", kI);
      SmartDashboard.putNumber("Thrower D", kD);
      SmartDashboard.putNumber("Thrower Feed Forward", kff);
      SmartDashboard.putNumber("Thrower I Zone", kIz);
      SmartDashboard.putNumber("Thrower Min Output", kMinOutput);
      SmartDashboard.putNumber("Thrower Max Output", kMaxOutput);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //This is where to set the sparkmax speed so it gets set every 20 millieseconds
    // setThrowerSpeed(ThrowerMotor.throwerMotorSpeed);
    //System.out.println( "ThrowerSubsystem periodic, about to set motor speed to " + m_setpoint);
    m_throwController.setReference(m_setpoint, ControlType.kVelocity);

     // Tune the thrower's constants
     if(ThrowerPIDs.TUNE){
      SmartDashboard.putNumber( "Thrower speed from encoder ", m_throwEncoder.getVelocity());
      double speed = SmartDashboard.getNumber("Thrower desired wheel RPM", 0);
      double p = SmartDashboard.getNumber("Thrower P", 0);
      double i = SmartDashboard.getNumber("Thrower I", 0);
      double d = SmartDashboard.getNumber("Thrower D", 0);
      double iz = SmartDashboard.getNumber("Thrower I Zone", 0);
      double ff = SmartDashboard.getNumber("Thrower Feed Forward", 0);
      double max = SmartDashboard.getNumber("Thrower Max Output", 0);
      double min = SmartDashboard.getNumber("Thrower Min Output", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if(( speed != m_setpoint)) { setThrowerSpeed( speed); }
      if((p != kP)) { m_throwController.setP(p); kP = p; }
      if((i != kI)) { m_throwController.setI(i); kI = i; }
      if((d != kD)) { m_throwController.setD(d); kD = d; }
      if((iz != kIz)) { m_throwController.setIZone(iz); kIz = iz; }
      if((ff != kff)) { m_throwController.setFF(ff); kff = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_throwController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
    }
  }


  /**
   * set motor speed so the thrower wheel is this many RPMs, based on ThrowerPIDs.GEAR_RATIO_MOTOR_TO_WHEEL
   * @param wheelTargetRPMs
   */
  public void setThrowerSpeed(double wheelTargetRPMs) {
    m_setpoint = wheelTargetRPMs; // * ThrowerPIDs.GEAR_RATIO_MOTOR_TO_WHEEL;
    System.out.println( "ThrowerSubsystem setThrowerSpeed() setpoint = " + m_setpoint);

    m_throwController.setReference(m_setpoint, ControlType.kVelocity);
    if(ThrowerPIDs.TUNE){
      SmartDashboard.putNumber("Thrower desired wheel RPM", m_setpoint);
      SmartDashboard.putNumber("Have now set the desired speed this many times: ", ii);
      ii ++;
    }
  }

  public double getThrowerSpeed() {
    return m_setpoint;
  }

  public void stopThrower() {
    m_setpoint = 0;
    // m_Follower.disable();
  }

    /**
     * Returns whether the motor's speed is within thresholdPercent of the desired speed
     * @param thresholdPercent (double) how close the speeds should be to return true, e.g. 0.01 for 1%
     * @return (boolean) true if the motor is that close to the desired speed
     */
  public boolean atSetpoint(double thresholdPercent) {
    return Math.abs(m_setpoint - m_throwEncoder.getVelocity()) <= Math.abs(m_setpoint*thresholdPercent);
  }
}
