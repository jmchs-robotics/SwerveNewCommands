/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class IntakeReversePulseCommand extends CommandBase {
  //private final ColorMatch m_colorMatcher = new ColorMatch();

  private IntakeSubsystem m_intake;
  private final Timer m_timer;
  private double m_time;

  /**
   * Runs the intake motor in reverse for a specific duration.
   * Motor speed and duration are read from the intake subsystem.
   */
  public IntakeReversePulseCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Requires the ControlPanel Subsystem
    addRequirements(intake);

    m_intake = intake;
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    m_time = m_intake.getReversePulse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Run the intake motor
     */
    m_intake.setMotor(m_intake.getReverseSpeed());
  }

  // Called once the command ends or is interrupted.
  // stop the motor
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotor(0.0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_time);
  }
}
