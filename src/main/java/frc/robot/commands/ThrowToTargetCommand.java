/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.util.SocketVision;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.ThrowerLUT;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;

public class ThrowToTargetCommand extends CommandBase {
  private ThrowerSubsystem m_subsystem;
  private SocketVisionWrapper m_vision;

  private double setpoint = 0;

  /**
   * Creates a new ThrowToTarget.
   * Reads the distance to the RFT, from the Vision Coprocessor.
   * Looks up the desired RPM based on the distance, using the ThrowerLUT.
   * Sets the desired RPM of the thrower.
   * Assumes the green LED has already been turned on 
   *   and that the Vision Coprocessor has already been commanded to track the RFT.
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
    if( Constants.ThrowerPIDs.TUNE) {
      SmartDashboard.putNumber("ThrowToTargetCommand distance from Vision", m_vision.get().get_distance());
    }
    // Make sure that the vision data is valid
    if(m_vision.get().get_direction() != SocketVision.NADA){
      setpoint = -ThrowerLUT.distanceToRPMs(m_vision.get().get_distance());
    }
    else {
        setpoint = -ThrowerLUT.DEFAULT_RPM;
    }
    
    m_subsystem.setThrowerSpeed(setpoint);
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