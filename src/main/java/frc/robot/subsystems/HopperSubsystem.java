/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperPIDs;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import com.ctre.phoenix.ParamEnum;

public class HopperSubsystem extends SubsystemBase {
  private final CANSparkMax m_hopperMotor;

  private CANPIDController m_hopperController;
  private CANEncoder m_hopperEncoder;

  private double kP = HopperPIDs.kP;
  private double kI = HopperPIDs.kI;
  private double kD = HopperPIDs.kD;
  private double kIz = HopperPIDs.kIz;
  private double kff = HopperPIDs.FEED_FORWARD;
  private double kMaxOutput = HopperPIDs.MAX_OUTPUT;
  private double kMinOutput = HopperPIDs.MIN_OUTPUT;
  
  private double m_reference = 0;
  /**
   * Creates a new HopperSubsystem.
   */
  public HopperSubsystem() {
    m_hopperMotor = new CANSparkMax(HopperConstants.HOPPER_MOTOR_ID, MotorType.kBrushed);

    //Don't want the hopper move beyond intention
    m_hopperMotor.setIdleMode(IdleMode.kBrake); 


    m_hopperMotor.configFactoryDefault();
    //m_hopperMotor.configGetParameter(ParamEnum.p, ordinal, timeoutMs);
    m_hopperController = m_hopperMotor.getPIDController();
    m_hopperEncoder = m_hopperMotor.getAlternateEncoder();

     
    resetReference();

    //Sets PID constants
    m_hopperController.setP(kP);
    m_hopperController.setD(kD);
    m_hopperController.setI(kI);
    m_hopperController.setIZone(kIz);
    m_hopperController.setFF(kff);
    
    m_hopperController.setOutputRange(kMinOutput,kMaxOutput);

    //Everything on the smartDashboard:
    SmartDashboard.putNumber("Thrower P", kP);
    SmartDashboard.putNumber("Thrower I", kI);
    SmartDashboard.putNumber("Thrower D", kD);
    SmartDashboard.putNumber("Thrower Feed Forward", kff);
    SmartDashboard.putNumber("Thrower I Zone", kIz);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
  }

  public void resetReference(){
    m_reference = m_hopperEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Move the daisy one full rotation around
  public void dischargeAll(){
    m_reference += HopperConstants.ONE_ROTATION;
    m_hopperController.setReference(m_reference, ControlType.kPosition);
  }

  // Move the daisy 1/6 rotation forward
  public void nextSlot(){
    m_reference += HopperConstants.ONE_ROTATION/6;
    m_hopperController.setReference(m_reference, ControlType.kPosition);
  }

  // Move the daisy 1/6 rotation backward
  public void previousSlot(){
    m_reference -= HopperConstants.ONE_ROTATION/6;
    m_hopperController.setReference(m_reference, ControlType.kPosition);
  }
}
