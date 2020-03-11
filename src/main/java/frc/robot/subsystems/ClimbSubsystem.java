/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControlPanelActuators;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ColorTargets;

public class ClimbSubsystem extends SubsystemBase {
  //private Spark  m_controlPanelSpinner;
  private VictorSPX m_climbVictor;
  private DoubleSolenoid m_climbSolenoid, m_extendSolenoid;

  private double m_forwardSpeed;
  private double m_reverseSpeed;
  private boolean m_raised;

  /**
   * Creates a new ClimbSubsystem.
   */
  public ClimbSubsystem() {
    m_climbVictor = new VictorSPX(ClimbConstants.climbVictorID);
    m_climbSolenoid = new DoubleSolenoid(ClimbConstants.soleniodForward, ClimbConstants.soleniodBackward);
    m_extendSolenoid = new DoubleSolenoid(ClimbConstants.extendSolenoidForward, ClimbConstants.extendSolenoidReverse);
    m_forwardSpeed = ClimbConstants.forwardSpeed;
    m_reverseSpeed = ClimbConstants.reverseSpeed;

    if (ClimbConstants.TUNE){
      SmartDashboard.putNumber("Climb Motor Output Percent", m_climbVictor.getMotorOutputPercent());
      SmartDashboard.putBoolean("Climb Raised", isRaised());
      SmartDashboard.putNumber("Climb Motor Forward Speed", m_forwardSpeed);
      SmartDashboard.putNumber("Climb Motor Reverse Speed", m_reverseSpeed);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (ClimbConstants.TUNE){
      double fs, rs;
      SmartDashboard.putNumber("Climb Motor Output Percent", m_climbVictor.getMotorOutputPercent());
      SmartDashboard.putBoolean("Climb Raised", isRaised());
      fs = SmartDashboard.getNumber("Climb Motor Forward Speed", 0);
      rs = SmartDashboard.getNumber("Climb Motor Reverse Speed", 0);

      if( fs != m_forwardSpeed) {
        m_forwardSpeed = fs;
          setWinchMotor(fs);
      }
      if( rs != m_reverseSpeed) {
        m_reverseSpeed = rs;
        setWinchMotor(rs);
      }
    }
  }

  /**
   * Sets the soleoid to forward.
   */
  public void raiseArm(){
    m_climbSolenoid.set(Value.kReverse);
    m_raised = true;
  }
  public void extendArm() {
    m_extendSolenoid.set(Value.kReverse);
  }

  /**
   * Sets the solenoid to reverse.
   */
  public void lowerArm(){
    m_climbSolenoid.set(Value.kForward);
    m_raised = false;
  }
  public void retractArm() {
    m_extendSolenoid.set(Value.kForward);
  }
  /**
   * Sets the solenoid to off.
   */
  public void turnOffSolenoid(){
    m_climbSolenoid.set(Value.kOff);
  }

  /**
   * Run winch
   * @param output The percent to set the output to.
   */
  public void setWinchMotor(double output){
    m_climbVictor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Get the status of the control panel subsystem.
   * @return True if the system is raised (deployed).
   */
  public boolean isRaised(){
    return m_raised;
  }

  //Turns the victor for the winch motor off
  public void turnWinchMotorOff() {
    m_climbVictor.set(ControlMode.PercentOutput, 0.0);
  }

  //Set the Double Solenoid to off
  public void setArmOff() {
    m_climbSolenoid.set(DoubleSolenoid.Value.kOff);
  }
}
