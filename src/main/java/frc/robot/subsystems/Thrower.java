/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ThrowerConstants;
import frc.robot.Constants.ThrowerMotors;

public class Thrower extends SubsystemBase {

  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotor;

  private CANPIDController m_throwerController;
  private CANEncoder m_throwerEncoder;

  private double kP = ThrowerConstants.kP;
  private double kI = ThrowerConstants.kI;
  private double kD = ThrowerConstants.kD;
  private double kIz = ThrowerConstants.kIz;
  private double kFF = ThrowerConstants.FEED_FORWARD;
  private double kMaxOutput = ThrowerConstants.MAX_OUTPUT;
  private double kMinOutput = ThrowerConstants.MIN_OUTPUT;

  /**
   * Creates a new Thrower.
   */
  public Thrower() {
    m_leftMotor = new CANSparkMax(ThrowerMotors.LEFT_MOTOR, MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(ThrowerMotors.RIGHT_MOTOR, MotorType.kBrushless);

    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotor.setIdleMode(IdleMode.kCoast);

    m_rightMotor.follow(m_leftMotor, ThrowerMotors.INVERT);
    // Set the leader to a custom update rate, default 10ms
    m_leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, ThrowerConstants.UPDATE_RATE);

    // Pull out the objects to reference the native encoder & pid controller
    m_throwerController = m_leftMotor.getPIDController();
    m_throwerEncoder = m_leftMotor.getEncoder();
    
    // Set PIDF constants
    m_throwerController.setP(kP);
    m_throwerController.setI(kI);
    m_throwerController.setD(kD);
    m_throwerController.setFF(kFF);
    m_throwerController.setIZone(kIz);

    // Set output range constants
    m_throwerController.setOutputRange(kMinOutput, kMaxOutput);

    // Write to SmartDashboard
    SmartDashboard.putNumber("Thrower P", kP);
    SmartDashboard.putNumber("Thrower I", kI);
    SmartDashboard.putNumber("Thrower D", kD);
    SmartDashboard.putNumber("Thrower Feed Forward", kFF);
    SmartDashboard.putNumber("Thrower I Zone", kIz);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Tune the thrower's constants
    if(ThrowerConstants.TUNE){
      double p = SmartDashboard.getNumber("Thrower P", 0);
      double i = SmartDashboard.getNumber("Thrower I", 0);
      double d = SmartDashboard.getNumber("Thrower D", 0);
      double iz = SmartDashboard.getNumber("Thrower I Zone", 0);
      double ff = SmartDashboard.getNumber("Thrower Feed Forward", 0);
      double max = SmartDashboard.getNumber("Thrower Max Output", 0);
      double min = SmartDashboard.getNumber("Thrower Min Output", 0);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_throwerController.setP(p); kP = p; }
      if((i != kI)) { m_throwerController.setI(i); kI = i; }
      if((d != kD)) { m_throwerController.setD(d); kD = d; }
      if((iz != kIz)) { m_throwerController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_throwerController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_throwerController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
    }
  }

  /**
   * Set the target speed for the thrower
   * @param wheelTargetRPMs The signed target speed for the thrower wheel.
   */
  public void setSetpoint(double wheelTargetRPMs){
    m_throwerController.setReference(wheelTargetRPMs * ThrowerConstants.GEAR_RATIO_MOTOR_TO_WHEEL, ControlType.kVelocity);
  }

  /**
   * Disable the thrower motors by setting their output to 0 and letting them coast to a stop.
   */
  public void stopThrower(){
    m_leftMotor.disable();
  }

  public double getVelocity(){
    return m_throwerEncoder.getVelocity();
  }
}
