/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSubsystem;

public class SpinThreeTimesCommand extends CommandBase {
  private ControlPanelSubsystem m_subsystem;
  
  private Color m_startColor, m_lastColor;
  private int m_numRevolutions;

  /**
   * Creates a new SpinThreeTimesCommand.
   */
  public SpinThreeTimesCommand(ControlPanelSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startColor = m_subsystem.readColor();
    m_lastColor = m_startColor;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setSpinMotor(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Don't read the same panel multiple times!
    if(m_subsystem.readColor() != m_lastColor){
      m_lastColor = m_subsystem.readColor();
      
      if(m_lastColor == m_startColor) m_numRevolutions ++;
    }
    
    return m_numRevolutions > 6; // Turn at least 3 but no more than 5 times. The color swatches will pass by twice, so > 6 gives 3.5+ turns.
  }
}
