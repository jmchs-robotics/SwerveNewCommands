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

public class IntakeAdvDaisyCommand extends ConditionalCommand {

  /**
   * Run the intake beater bar 'forward,' for collecting game pices (balls), shifting the hopper to a new slot as they fill.
   * @param intake The intake subystem
   * @param hopper The hopper subsystem
   */
  public IntakeAdvDaisyCommand(IntakeSubsystem intake, HopperSubsystem hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    // Requires the ControlPanel Subsystem

    // This is just the constructor for a conditional command, wrapped in the IntakeAdvDaisyCommand name
    // super(Command onTrue, Command onFalse, BooleanSupplier condition)
    super(
      new SequentialCommandGroup(
          //new InstantCommand(intake :: lowerIntake, intake),
          new ParallelRaceGroup(
            new IntakeRecieveCommand(intake),
            new WaitUntilCommand(()->{ return hopper.ballLoaded();}) 
          ),
          new InstantCommand( hopper::incBallCount, hopper),
          new ConditionalCommand( // if not full, advance Daisy; if full, reverse intake
              new SequentialCommandGroup(
                new IntakeReversePulseCommand( intake),
                new BumpHopperCommand(hopper),
                new MoveHopperCommand(hopper, 1),
                new InstantCommand( hopper::incBallCount, hopper)
              ),
              new IntakeReverseCommand( intake),
              () -> {return hopper.daisyIsFull();} 
          )
        ),
      new IntakeReverseCommand( intake),
      () -> {return hopper.daisyIsFull();} // The conditional: if there are less than 5 balls, intake; else, backdrive
    );
  }

}
