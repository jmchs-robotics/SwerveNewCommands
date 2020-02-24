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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class DefaultWinchCommand extends CommandBase {
 
  private XboxController m_controller = new XboxController(1);
  private final ClimbSubsystem m_subsystem;
  
  /**
   * Creates a new DefaultIntakeCommand.
   */
  public DefaultWinchCommand( ClimbSubsystem subsystem, XboxController stick) {
    m_subsystem = subsystem;
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
 
    final double yAxis = m_controller.getTriggerAxis(Hand.kLeft);

    // run the motor, if the joystick is past a deadband
    if(yAxis >= 0.5){
    //     new IntakeRecieveCommand(m_intake);
    m_subsystem.setWinchMotor((yAxis - 0.5) * 2); // setMotor( yAxis);
    // System.out.println("2");
    } else {
      m_subsystem.setWinchMotor(0);
     }
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setWinchMotor(0);
  }
 
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
 
 

