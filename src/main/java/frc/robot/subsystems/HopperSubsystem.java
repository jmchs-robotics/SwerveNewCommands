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
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  private TalonSRX m_hopperMotor;
  private ColorSensorV3 m_hopperSensor; // To be replaced with the actual sensor for the hopper

  private int m_storeCount = 0;
  
  /**
   * Creates a new Hopper.
   */
  public HopperSubsystem() {
    m_hopperMotor = new TalonSRX(HopperConstants.MAIN_MOTOR_ID);
    m_hopperSensor = new ColorSensorV3(HopperConstants.COLOR_SENSOR_PORT);
    
    m_hopperMotor.configAllSettings(HopperConstants.GetMainMotorConfiguration());
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
    m_hopperMotor.set(ControlMode.Position, m_hopperMotor.getClosedLoopTarget() + HopperConstants.ENCODER_TICKS_TO_REVOLUTION);
    m_storeCount = 0;
  }

  /**
   * Set the next setpoint to one-sixth revolution forward. DO NOT call repeatedly in execute().
   */
  public void nextSlot() {
    m_hopperMotor.set(ControlMode.Position, m_hopperMotor.getClosedLoopTarget() + HopperConstants.ENCODER_TICKS_TO_REVOLUTION / 6);
  }

  /**
   * Set the next setpoint to one-sixth revolution backward. DO NOT call repeatedly in execute().
   */
  public void previousSlot() {
    m_hopperMotor.set(ControlMode.Position, m_hopperMotor.getClosedLoopTarget() - HopperConstants.ENCODER_TICKS_TO_REVOLUTION / 6);
  }

  /**
   * Checks whether the closed-loop control is within maxError of the setpoint.
   * @param maxError the maximum error (in encoder ticks)
   * @return Whether the absolute value of the closed loop error is less than maxError.
   */
  public boolean atSetpoint(int maxError) {
    return Math.abs(m_hopperMotor.getClosedLoopError()) < maxError;
  }

  /**
   * Valid only if {@code incStoredCount()} is called each time a ball is added to the hopper.
   * @return The number of balls stored in the hopper.
   */
  public int getStoredCount(){
    return m_storeCount;
  }

  /**
   * Increment the counter tracking the number of balls stored in the hopper. Remember that we can store 5 at most!
   */
  public void incStoredCount(){
    m_storeCount++;
  }

  /**
   * Detect whether a ball is in the loading slot of the hopper.
   * @return true if there is a ball in the loading slot.
   */
  public boolean ballLoaded(){
    return m_hopperSensor.getProximity() > HopperConstants.BALL_LOADED_THRESHOLD;
  }

}
