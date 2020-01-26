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
  private double m_moveToWhere = 0;
  /**
   * Creates a new SetHopperSpeedCommand.
   */
  public MoveHopperCommand(HopperSubsystem subsystem, double moveToWhere) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    m_subsystem = subsystem;
    m_moveToWhere = moveToWhere;
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
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean b = m_subsystem.atSetpoint(0.01);
    if( b) {
      System.out.println( "MoveHopperCommand isFinished = true");
      return true; // m_subsystem.atSetpoint(0.01);
    }
    else {
      return false;
    }
  }
}
