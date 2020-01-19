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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelActuators;

public class ControlPanelSubsystem extends SubsystemBase {
  private TalonSRX m_controlPanelSpinner;
  private DoubleSolenoid m_controlPanelPiston;

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ControlPanelSubsystem() {
    m_controlPanelSpinner = new TalonSRX(ControlPanelActuators.motorCanID);
    m_controlPanelPiston = new DoubleSolenoid(ControlPanelActuators.soleniodFirst, ControlPanelActuators.soleniodSecond);
    m_controlPanelSpinner.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Turns the Control Panel motor off
  public void turnSpinnerMotorOff() {
    m_controlPanelSpinner.set(ControlMode.PercentOutput, 0.0);
  }

  //Sets the Control Panel Spinner to the given percentage
  public void setSpinnerMotor(double percentOut) {
    m_controlPanelSpinner.set(ControlMode.PercentOutput, percentOut);
  }

  //Set the Double Solenoid to extended
  public void setPanelPistonExtended() {
    m_controlPanelPiston.set(DoubleSolenoid.Value.kForward);
  }

  //Set the Double Solenoid to retracted
  public void setPanelPistonRetracted() {
    m_controlPanelPiston.set(DoubleSolenoid.Value.kReverse);
  }

  //Set the Double Solenoid to off
  public void setPanelPistonOff() {
    m_controlPanelPiston.set(DoubleSolenoid.Value.kOff);
  }
}
