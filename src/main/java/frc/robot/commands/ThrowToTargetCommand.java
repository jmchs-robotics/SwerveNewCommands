/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ThrowerLUT;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.util.SocketVision;
import frc.robot.util.SocketVisionWrapper;

public class ThrowToTargetCommand extends CommandBase {
  private ThrowerSubsystem m_subsystem;
  private SocketVisionWrapper m_vision;

  private double setpoint = 0;

  /**
   * Creates a new ThrowToTarget.
   */
  public ThrowToTargetCommand(ThrowerSubsystem thrower, SocketVisionWrapper vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(thrower);

    m_subsystem = thrower;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Make sure that the vision data is valid
    if(m_vision.get().get_direction() != SocketVision.NADA){
      setpoint = ThrowerLUT.distanceToRPMs(m_vision.get().get_distance());
    }
    
    m_subsystem.setSetpoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopThrower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
