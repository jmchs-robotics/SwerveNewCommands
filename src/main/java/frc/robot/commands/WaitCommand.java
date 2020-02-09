/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/** wait a given time, in seconds, then end. */
public class WaitCommand extends CommandBase {
  private final Timer m_timer;
  private double m_time;

  /** wait a given time, in seconds, then end.
   * @param t how long to wait, in seconds
  */

  public WaitCommand( double t) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_timer = new Timer();
    m_time = t;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_time);
  }
}
