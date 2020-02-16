/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends CommandBase {
  //private final ColorMatch m_colorMatcher = new ColorMatch();

  private IntakeSubsystem m_intake;
 

  /**
   * Run the intake beater bar 'reverse,' for ejecting game pices (balls)
   */
  public IntakeReverseCommand(IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Requires the ControlPanel Subsystem
    addRequirements(intake);

    m_intake = intake;
    //m_int = m_int + 1;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run the intake motor     
    m_intake.setMotor(-0.7);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setMotor(0.0);
    //m_intake.lowerIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
