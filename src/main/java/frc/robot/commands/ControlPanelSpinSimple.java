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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorTargets;
import frc.robot.subsystems.PatSajakSubsystem;
import frc.robot.Constants.ControlPanelActuators;

public class ControlPanelSpinSimple extends CommandBase {
  //private final ColorMatch m_colorMatcher = new ColorMatch();

  private PatSajakSubsystem m_patSajak;

  //  private final ColorSensorV3 m_colorSensor;
  // private final ColorMatch m_colorMatcher = new ColorMatch();

  // private Color m_startColor, m_lastColor;
  // private int m_numRevolutions;

  /**
   * Creates a new SampleColor.
   */
  public ControlPanelSpinSimple(PatSajakSubsystem patSajak) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Requires the ControlPanel Subsystem
    addRequirements(patSajak);

    m_patSajak = patSajak;
    // m_colorSensor = sensor;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*
    m_startColor = m_patSajak.readColor();
    m_lastColor = m_startColor;
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * The Command will work mostly with when finished
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */

    //m_patSajak.raiseSpinner();
    m_patSajak.setSpinMotor(0.7);


    //Smart Dashboard for testing
    // Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    /* String colorString;
     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == ColorTargets.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == ColorTargets.kRedTarget) {
      colorString = "Red";
    } else if (match.color == ColorTargets.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == ColorTargets.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    */

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    if( ControlPanelActuators.TUNE) {
      /*
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);
      */
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_patSajak.turnSpinnerMotorOff();
    //m_patSajak.lowerSpinner();
    //m_numRevolutions = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    /*
     // Don't read the same panel multiple times!
     if(m_patSajak.readColor() != m_lastColor){
      m_lastColor = m_patSajak.readColor();
      
      if(m_lastColor == m_startColor) m_numRevolutions ++;
    }
    
    return m_numRevolutions > 6; // Turn at least 3 but no more than 5 times. The color swatches will pass by twice, so > 6 gives 3.5+ turns.
    */
  }
}
