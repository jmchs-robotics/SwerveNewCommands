/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * An example command that uses an example subsystem.
 */
public class DefaultSwerveCommand extends CommandBase {
  
  private final SwerveDriveSubsystem m_subsystem;
  private final XboxController m_stick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param stick The XBoxController used by this command.
   */
  public DefaultSwerveCommand(SwerveDriveSubsystem subsystem, XboxController stick) {
    m_subsystem = subsystem;
    m_stick = stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  private double deadband(double input){
    if(Math.abs(input) < 0.05) return 0;
    return input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = m_stick.getY(Hand.kLeft);
    double strafe = m_stick.getX(Hand.kLeft);
    double rotation = m_stick.getX(Hand.kRight);

    forward *= Math.abs(forward);
		strafe *= Math.abs(strafe);
		rotation *= Math.abs(rotation);

		forward = deadband(forward);
		strafe = deadband(strafe);
    rotation = deadband(rotation);
    
    m_subsystem.holonomicDrive(forward, strafe, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Try to bring the robot to a dead stop before starting the next command
    m_subsystem.stopDriveMotors(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
