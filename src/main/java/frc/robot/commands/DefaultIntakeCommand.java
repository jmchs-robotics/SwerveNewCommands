/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.commands;
 
import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class DefaultIntakeCommand extends CommandBase {
 
  private XboxController m_controller = new XboxController(1);
  private final IntakeSubsystem m_intake;
  
  /**
   * Creates a new DefaultIntakeCommand.
   */
  public DefaultIntakeCommand( IntakeSubsystem subsystem, XboxController stick) {
    m_intake = subsystem;
    m_controller = stick;
 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
 
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
 
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    final double yAxis = m_controller.getY(Hand.kLeft);
    final double xAxis = m_controller.getX(Hand.kRight);
 
    if(yAxis > 0){
     new IntakeRecieveCommand(m_intake);
    } else if(yAxis < 0){
      new IntakeReversePulseCommand(m_intake);
     }
 
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
 
 

