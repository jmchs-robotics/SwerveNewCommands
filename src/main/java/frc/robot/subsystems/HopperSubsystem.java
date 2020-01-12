/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperMotors;

public class HopperSubsystem extends SubsystemBase {
  private TalonSRX m_hopperMotor;
  
  /**
   * Creates a new Hopper.
   */
  public HopperSubsystem() {
    m_hopperMotor = new TalonSRX(HopperMotors.MAIN_MOTOR_ID);
    
    m_hopperMotor.configAllSettings(HopperMotors.GetMainMotorConfiguration());
    m_hopperMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Set the next setpoint to one full revolution forward. DO NOT call repeatedly in execute().
   */
  public void dischargeAll() {
    m_hopperMotor.set(ControlMode.Position, m_hopperMotor.getClosedLoopTarget() + HopperMotors.ENCODER_TICKS_TO_REVOLUTION);
  }

  /**
   * Set the next setpoint to one-sixth revolution forward. DO NOT call repeatedly in execute().
   */
  public void nextSlot() {
    m_hopperMotor.set(ControlMode.Position, m_hopperMotor.getClosedLoopTarget() + HopperMotors.ENCODER_TICKS_TO_REVOLUTION / 6);
  }

  /**
   * Set the next setpoint to one-sixth revolution backward. DO NOT call repeatedly in execute().
   */
  public void previousSlot() {
    m_hopperMotor.set(ControlMode.Position, m_hopperMotor.getClosedLoopTarget() - HopperMotors.ENCODER_TICKS_TO_REVOLUTION / 6);
  }

  /**
   * Checks whether the closed-loop control is within maxError of the setpoint.
   * @param maxError the maximum error (in encoder ticks)
   * @return Whether the absolute value of the closed loop error is less than maxError.
   */
  public boolean atSetpoint(int maxError) {
    return Math.abs(m_hopperMotor.getClosedLoopError()) < maxError;
  }

}
