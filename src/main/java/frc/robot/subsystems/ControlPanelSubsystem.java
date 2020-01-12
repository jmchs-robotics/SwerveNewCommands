/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelActuators;

public class ControlPanelSubsystem extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private CANSparkMax m_spinner;

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ControlPanelSubsystem() {
    m_solenoid = new DoubleSolenoid(ControlPanelActuators.SOLENOID_FORWARD_CHANNEL, ControlPanelActuators.SOLENOID_REVERSE_CHANNEL);
    m_spinner = new CANSparkMax(ControlPanelActuators.SPINNER_ID, MotorType.kBrushless);

    m_spinner.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the soleoid to forward.
   */
  public void raiseSpinner(){
    m_solenoid.set(Value.kForward);
  }

  /**
   * Sets the solenoid to reverse.
   */
  public void lowerSpinner(){
    m_solenoid.set(Value.kReverse);
  }

  /**
   * Sets the solenoid to off.
   */
  public void turnOffSolenoid(){
    m_solenoid.set(Value.kOff);
  }

  public void setSpinMotor(double output){
    m_spinner.set(output);
  }
}
