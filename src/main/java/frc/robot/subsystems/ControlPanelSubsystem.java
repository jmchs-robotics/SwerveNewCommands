/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorTargets;
import frc.robot.Constants.ControlPanelActuators;

public class ControlPanelSubsystem extends SubsystemBase {
  private DoubleSolenoid m_solenoid;
  private CANSparkMax m_spinner;
  private ColorSensorV3 m_colorSensor;

  private ColorMatch m_colorMatcher = new ColorMatch();
  private boolean m_raised;

  /**
   * Creates a new ControlPanelSubsystem.
   */
  public ControlPanelSubsystem() {
    m_solenoid = new DoubleSolenoid(ControlPanelActuators.SOLENOID_FORWARD_CHANNEL, ControlPanelActuators.SOLENOID_REVERSE_CHANNEL);
    m_spinner = new CANSparkMax(ControlPanelActuators.SPINNER_ID, MotorType.kBrushless);
    m_colorSensor = new ColorSensorV3(ControlPanelActuators.COLOR_SENSOR_PORT);

    m_spinner.setIdleMode(IdleMode.kBrake);

    // Colors you want to search for must be added to the colorMatcher.
    m_colorMatcher.addColorMatch(ColorTargets.kBlueTarget);
    m_colorMatcher.addColorMatch(ColorTargets.kGreenTarget);
    m_colorMatcher.addColorMatch(ColorTargets.kRedTarget);
    m_colorMatcher.addColorMatch(ColorTargets.kYellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Given the game specific message, returns the color to search for. If the gameString is not one of 'R', 'B', 'G', 'Y', returns null.
   * @return The Color 90 degrees from the color to be under the sensor.
   */
  public Color getColorToSearchFor(String gameString){
    switch(gameString){
      case "R":
        return ColorTargets.kBlueTarget;
      case "B":
        return ColorTargets.kRedTarget;
      case "G":
        return ColorTargets.kYellowTarget;
      case "Y":
        return ColorTargets.kGreenTarget;
      default:
        return null;
    }
  }

  /**
   * Given a color the sensor is at, and a color to search for, return the sign of the direction the motor should spin. Returns 0 if the target is reached.
   * @param targetColor The color to target (use {@code getColorToSearchFor} to get the game specific color to search for).
   * @param presentColor The color the sensor is on.
   * @return 1 if the wheel should turn clockwise, -1 if the wheel should turn counterclockwise, 0 otherwise (as viewed from above).
   */
  public int getDirectionToTurn(Color targetColor, Color presentColor){
    if(targetColor == ColorTargets.kBlueTarget){
      if(presentColor == ColorTargets.kRedTarget){ return 0; }
      else if(presentColor == ColorTargets.kGreenTarget){ return -1; }

    } else if(targetColor == ColorTargets.kRedTarget) {
      if(presentColor == ColorTargets.kBlueTarget){ return 0; }
      else if(presentColor == ColorTargets.kYellowTarget){ return -1; }

    } else if(targetColor == ColorTargets.kGreenTarget){
      if(presentColor == ColorTargets.kYellowTarget){ return 0; }
      else if(presentColor == ColorTargets.kRedTarget){ return -1; }

    } else if(targetColor == ColorTargets.kYellowTarget){
      if(presentColor == ColorTargets.kGreenTarget){ return 0; }
      else if(presentColor == ColorTargets.kBlueTarget){ return -1; }
    }

    return 1; // The remaining 8 cases are all spin forwards
  }

  /**
   * Get the closest color being read by the color sensor.
   * @return One of Red, Blue, Green, or Yellow.
   */
  public Color readColor(){
    return m_colorMatcher.matchClosestColor(m_colorSensor.getColor()).color;
  }

  /**
   * Sets the soleoid to forward.
   */
  public void raiseSpinner(){
    m_solenoid.set(Value.kForward);
    m_raised = true;
  }

  /**
   * Sets the solenoid to reverse.
   */
  public void lowerSpinner(){
    m_solenoid.set(Value.kReverse);
    m_raised = false;
  }

  /**
   * Sets the solenoid to off.
   */
  public void turnOffSolenoid(){
    m_solenoid.set(Value.kOff);
  }

  /**
   * Spin the control panel.
   * @param output The percent to set the output to.
   */
  public void setSpinMotor(double output){
    m_spinner.set(output);
  }

  /**
   * Get the status of the control panel subsystem.
   * @return True if the system is raised (deployed).
   */
  public boolean isRaised(){
    return m_raised;
  }
}
