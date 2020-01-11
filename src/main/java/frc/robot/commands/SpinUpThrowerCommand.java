/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ThrowerLUT;
import frc.robot.subsystems.Thrower;
import frc.robot.util.SocketVisionWrapper;

public class SpinUpThrowerCommand extends CommandBase {
  private Thrower m_thrower;
  private SocketVisionWrapper m_vision;

  /**
   * Creates a new SpinUpThrowerCommand.
   */
  public SpinUpThrowerCommand(Thrower thrower, SocketVisionWrapper visionTarget) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(thrower);

    m_thrower = thrower;
    m_vision = visionTarget;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_thrower.setSetpoint(ThrowerLUT.distanceToRPMs(m_vision.get().get_distance()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Wait for the thrower to spin up
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Do nothing -- the thrower should keep spinning!
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Within 5% of setpoint
    return m_thrower.atSetpoint(0.05);
  }
}
