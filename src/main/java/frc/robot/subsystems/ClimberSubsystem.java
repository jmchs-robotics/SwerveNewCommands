/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberActuators;

public class ClimberSubsystem extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private TalonSRX m_winchMotor;

  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {
    m_solenoid = new DoubleSolenoid(ClimberActuators.SOLENOID_FORWARD_CHANNEL, ClimberActuators.SOLENOID_REVERSE_CHANNEL);
    m_winchMotor = new TalonSRX(ClimberActuators.WINCH_MOTOR);

    // Motor settings
    m_winchMotor.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Elevate the climber to deliver the hook.
   */
  public void liftClimber(){
    m_solenoid.set(Value.kForward);
  }

  /**
   * Lower the climber arm out of the way.
   */
  public void lowerClimber(){
    m_solenoid.set(Value.kReverse);
  }

  /**
   * Turn off the solenoid.
   */
  public void turnOffSolenoid(){
    m_solenoid.set(Value.kOff);
  }

  /**
   * 80% output forward! Due to the mechanism ratcheting, no backdriving will be allowed.
   */
  public void climb() {
    m_winchMotor.set(ControlMode.PercentOutput, 0.8);
  }
}
