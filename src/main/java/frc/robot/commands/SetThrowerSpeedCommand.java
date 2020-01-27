/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ThrowerSubsystem;

import frc.robot.Constants.ThrowerMotor;
import frc.robot.Constants.ThrowerPIDs;

public class SetThrowerSpeedCommand extends CommandBase {
  private ThrowerSubsystem m_subsystem;
  private double setPoint = 1000;
  /**
   * Creates a new SetThrowerSpeedCommand.
   */
  public SetThrowerSpeedCommand(ThrowerSubsystem subsystem, double intendedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem=subsystem;
    setPoint=intendedSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //System.out.println("SetThrowerSpeedCommand initialize with setPoint= " + setPoint);
    m_subsystem.setThrowerSpeed(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.setThrowerSpeed(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetpoint(0.05);
    //0.01 means this is the percent we want for our thrower.
  }
}
