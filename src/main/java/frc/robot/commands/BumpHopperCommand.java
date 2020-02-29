/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperPIDs;

public class BumpHopperCommand extends CommandBase {
  private HopperSubsystem m_subsystem;

  /**
   * Move the hopper 1/24 of a slot back, and wait for the hopper to finish.
   */
  public BumpHopperCommand(HopperSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem = subsystem; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.bumpBack();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("You Done Daisy", "Im am so done");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return m_subsystem.atSetpoint(0.02);
  }
}
