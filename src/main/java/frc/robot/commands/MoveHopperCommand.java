/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperPIDs;

public class MoveHopperCommand extends CommandBase {
  private HopperSubsystem m_subsystem;
  private double m_setPoint = 100;
  /**
   * Creates a new SetHopperSpeedCommand.
   */
  public MoveHopperCommand(HopperSubsystem subsystem, double intendedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem=subsystem;
    m_setPoint=intendedSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("MoveHopperCommand calling dischargeAll");
    
    m_subsystem.dischargeAll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_subsystem.setHopperSpeed(setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.atSetpoint(0.01);
    //0.01 means this is the percent we want for our Hopper.
  }
}
