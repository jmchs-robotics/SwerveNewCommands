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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorTargets;
import frc.robot.subsystems.ControlPanelSubsystem;

public class SampleColorCommand extends CommandBase {
  private final ControlPanelSubsystem m_subsystem;

  /**
   * Creates a new SampleColor.
   */
  public SampleColorCommand(ControlPanelSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Has no subsystem requirements -- simply reads the color sensor.

    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Color Sensor Sampling", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Color detectedColor = m_subsystem.readColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;

    if (detectedColor == ColorTargets.kBlueTarget) {
      colorString = "Blue";
    } else if (detectedColor == ColorTargets.kRedTarget) {
      colorString = "Red";
    } else if (detectedColor == ColorTargets.kGreenTarget) {
      colorString = "Green";
    } else if (detectedColor == ColorTargets.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Detected Color", colorString);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Color Sensor Sampling", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
