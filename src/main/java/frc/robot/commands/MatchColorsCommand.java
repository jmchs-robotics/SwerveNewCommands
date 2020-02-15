/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ColorTargets;
import frc.robot.subsystems.PatSajakSubsystem;


public class MatchColorsCommand extends CommandBase {
  
  private PatSajakSubsystem m_cPanelSubsystem;
  private ColorSensorV3 m_colorSensor;
  private RobotContainer m_container;
  private ColorMatch m_colorMatch = new ColorMatch();
  
  private String m_targetColor;
  
  /**
   * Creates a new MatchColorsCommand.
   */
  public MatchColorsCommand(PatSajakSubsystem subsystem, ColorSensorV3 sensor, RobotContainer container) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_cPanelSubsystem = subsystem;
    m_colorSensor = sensor;
    m_container = container;

    m_colorMatch.addColorMatch(ColorTargets.kBlueTarget);
    m_colorMatch.addColorMatch(ColorTargets.kGreenTarget);
    m_colorMatch.addColorMatch(ColorTargets.kRedTarget);
    m_colorMatch.addColorMatch(ColorTargets.kYellowTarget);

    

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_targetColor = m_container.getGameSpecificMessage();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute () {

    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatch.matchClosestColor(detectedColor);
    
    if (match.color == ColorTargets.kGreenTarget) {
      if (m_targetColor.equalsIgnoreCase("Y")){
        m_cPanelSubsystem.setSpinMotor(0);
      } else if(m_targetColor.equalsIgnoreCase("R")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("G")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("B")){
        m_cPanelSubsystem.setSpinMotor(-ColorTargets.cpSpinnerSpeed);
    }

    if (match.color == ColorTargets.kBlueTarget) {
      if (m_targetColor.equalsIgnoreCase("Y")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("R")){
        m_cPanelSubsystem.setSpinMotor(0);
      } else if(m_targetColor.equalsIgnoreCase("G")){
        m_cPanelSubsystem.setSpinMotor(-ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("B")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
    }

    if (match.color == ColorTargets.kYellowTarget) {
      if (m_targetColor.equalsIgnoreCase("Y")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("R")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("G")){
        m_cPanelSubsystem.setSpinMotor(0);
      } else if(m_targetColor.equalsIgnoreCase("B")){
        m_cPanelSubsystem.setSpinMotor(-ColorTargets.cpSpinnerSpeed);
    }

    if (match.color == ColorTargets.kRedTarget) {
      if (m_targetColor.equalsIgnoreCase("Y")){
        m_cPanelSubsystem.setSpinMotor(-ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("R")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("G")){
        m_cPanelSubsystem.setSpinMotor(ColorTargets.cpSpinnerSpeed);
      } else if(m_targetColor.equalsIgnoreCase("B")){
        m_cPanelSubsystem.setSpinMotor(0);
    }
  }
}
}
}

}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
