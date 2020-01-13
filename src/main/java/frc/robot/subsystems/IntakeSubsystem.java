/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeActuators;

public class IntakeSubsystem extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private Spark m_motor;

  private boolean m_lowered;

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    m_solenoid = new DoubleSolenoid(IntakeActuators.SOLENOID_FORWARD_CHANNEL, IntakeActuators.SOLENOID_REVERSE_CHANNEL);
    m_motor = new Spark(IntakeActuators.INTAKE_MOTOR_PWM_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Extends the intake piston.
   */
  public void lowerIntake(){
    m_solenoid.set(Value.kForward);
    m_lowered = true;
  }

  /**
   * Retracts the intake piston.
   */
  public void raiseIntake(){
    m_solenoid.set(Value.kReverse);
    m_lowered = false;
  }

  /**
   * Turns off the intake solenoid.
   */
  public void turnOffSolenoid(){
    m_solenoid.set(Value.kOff);
  }

  /**
   * Get the status of the intake
   * @return True if the intake is lowered
   */
  public boolean isLowered() {
    return m_lowered; // return true if the intake piston is forward (intake is lowered down)
  }

  /**
   * Set the intake motor to a speed between -1 and 1.
   * @param speed
   */
  public void setMotor(double speed){
    m_motor.set(speed);
  }
}
