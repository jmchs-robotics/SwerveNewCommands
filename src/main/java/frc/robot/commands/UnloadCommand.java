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
 * @param swerve
 * @param thrower
 * @param sender
 * @param rft
 */
public class UnloadCommand extends CommandBase {
    SwerveDriveSubsystem m_swerve;
    ThrowerSubsystem m_Thrower;
    SocketVisionSendWrapper sender_;
    SocketVisionWrapper rft_;
    private HopperSubsystem m_Hopper;

    public UnloadCommand( SwerveDriveSubsystem swerve, ThrowerSubsystem thrower, HopperSubsystem m_Hopper, SocketVisionSendWrapper sender, SocketVisionWrapper rft) {
        m_swerve = swerve;
        m_Thrower = thrower;
        sender_ = sender;   
        rft_ = rft;       

        new SequentialCommandGroup(
            new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
            new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
            new WaitCommand( 0.25), // give the vision processor a chance to find the RFT
            new ParallelCommandGroup( // waits for both to end
                new SpinUpThrowerCommand(m_Thrower, rft_),  // set thrower speed to vision distance, end when it's there
                new VisionAim( m_swerve, rft_, 9, 18)
            ),
            new ParallelRaceGroup(
                new ThrowToTargetCommand(m_Thrower, rft_),  // never ends
                new MoveHopperCommand(m_Hopper, 6)
            ),
            new SetThrowerSpeedCommand(m_Thrower, 0),
            new SendVisionCommand(sender_, "_"),
            new InstantCommand(m_Thrower::turnOffLED, m_Thrower) // Turn on green LED
        );
    }

    @Override
    public void initialize() {
    }
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
        boolean b = m_Hopper.atSetpoint(0.01);
        if( b) {
        // System.out.println( "MoveHopperCommand isFinished = true");
        return true; // m_subsystem.atSetpoint(0.01);
        }
        else {
            return false;
        }
    }
}