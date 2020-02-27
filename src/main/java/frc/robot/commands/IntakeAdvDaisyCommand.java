/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAdvDaisyCommand extends CommandBase {
  //private final ColorMatch m_colorMatcher = new ColorMatch();

  private IntakeSubsystem m_Intake;
  private HopperSubsystem m_Hopper;
 

  /**
   * Run the intake beater bar 'forward,' for collecting game pices (balls).
   */
  public IntakeAdvDaisyCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Requires the ControlPanel Subsystem
    addRequirements(intake, hopper);

    m_Intake = intake;
    m_Hopper = hopper;
    
    new ConditionalCommand(
      new SequentialCommandGroup(
          //new InstantCommand(m_Intake :: lowerIntake, m_Intake),
          new ParallelRaceGroup(
            new IntakeRecieveCommand(m_Intake),
            new WaitUntilCommand(()->{ return m_Hopper.ballLoaded();}) 
          ),
          new InstantCommand( m_Hopper::incBallCount, m_Hopper),
          new ConditionalCommand( // if not full, advance Daisy; if full, reverse intake
              new SequentialCommandGroup(
                new IntakeReversePulseCommand( m_Intake),
                new BumpHopperCommand(m_Hopper),
                new MoveHopperCommand(m_Hopper, 1),
                new InstantCommand( m_Hopper::incBallCount, m_Hopper)
              ),
              new IntakeReverseCommand( m_Intake),
              () -> {return m_Hopper.daisyIsFull();} 
          )
        ),
      new IntakeReverseCommand( m_Intake),
      () -> {return m_Hopper.daisyIsFull();} // The conditional: if there are less than 5 balls, intake; else, backdrive
    );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    return false;
  }
}
