package frc.robot.commands;

import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.util.SocketVision;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.ThrowerLUT;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.util.*;
import frc.robot.commands.*;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.SwerveDriveModule;

/**
 * Collection of commands to shoot all balls into the target.
 */
public class UnloadCommand extends SequentialCommandGroup {

    /**
     * Collection of commands to shoot all balls into the target.
     * @param swerve The swerve subsystem on the robot
     * @param thrower The thrower subsystem on the robot
     * @param sender The one-way send connection to the vision coprocessor
     * @param rft The one-way read connection to the vision coprocessor on the high-goal tracking port
     * @param waitATick seconds to pause to give vision coprocessor a chance to track the RFT before VisionAim or SpinUpThrower
     */
    public UnloadCommand( SwerveDriveSubsystem swerve, ThrowerSubsystem thrower, HopperSubsystem hopper, SocketVisionSendWrapper sender, SocketVisionWrapper rft, double waitATick) {

        // This is kinda like 'new SequentialCommandGroup()' but it's the internal method that the constructor calls, so this is synonymous with
        // calling the superconstructor -- which would look like the IntakeAdvDaisyCommand, constructing the superclass SequentialCommandGroup (like the
        // ConditionalCommand superconstructor was called in IntakeAdvDaisyCommand) here.
        // In short, making the subsystems private variables is moot, as nothing gets done beyond the constructor here.
        addCommands( // super(
            new InstantCommand(thrower::turnOnLED, thrower), // Turn on green LED
            new SendVisionCommand(sender, "R"), // Can't be a lambda because Sender's aren't subsystems
            new WaitCommand( waitATick), // give the vision processor a chance to find the RFT
            new ParallelCommandGroup( // waits for both to end
                new SpinUpThrowerCommand(thrower, swerve, rft),  // set thrower speed to vision distance, end when it's there
                new VisionAimGyroCommand( swerve, rft) // aim the robot
            ),
            new ParallelRaceGroup(
                new ThrowToTargetCommand(thrower, swerve, rft),  // never ends
                new MoveHopperCommand(hopper, 6)
            ),
            new SetThrowerSpeedCommand(thrower, 0),
            new SendVisionCommand(sender, "_"),
            new InstantCommand(thrower::turnOffLED, thrower) // Turn on green LED
        );
    }
}