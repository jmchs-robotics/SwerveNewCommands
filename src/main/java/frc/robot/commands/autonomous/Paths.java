package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.util.*;
// import jdk.vm.ci.code.InstalledCode;
import frc.robot.commands.*;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.SwerveDriveModule;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class Paths { // extends CommandBase {

    //private SwerveDriveSubsystem m_swerve;
    SwerveDriveSubsystem m_swerve;
    ThrowerSubsystem m_Thrower;
    SocketVisionSendWrapper sender_;
    SocketVisionWrapper rft_;

    public Paths( SwerveDriveSubsystem swerve, ThrowerSubsystem thrower, SocketVisionSendWrapper sender, SocketVisionWrapper rft) {
        m_swerve = swerve;
        m_Thrower = thrower;
        sender_ = sender;   
        rft_ = rft;        
    }

    /**
     * Testing
     * Go from fence to scoring position, staying right of another robot striaght in front of goal 
     * aim, score
     */
    public Command Path1Command() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
        //new SetWheelAngleCommand( m_swerve, -18),  // point the wheels in the direction we want to go
        new WaitCommand( 2), // 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
        // new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), // test doing DriveForDist at slow speed
        //new DriveForDist2910Command( m_swerve, -37, -12), // go to destination 
        // new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        //new WaitCommand( 0.1), // give vision coprocessor a chance to find the target
        // TODO: UnloadCommand().  remove VisionAim and any last WaitCommand()
        
        new VisionAimCommand( m_swerve, rft_), // aim at RFT
        new WaitCommand( 1),// give the drivetrain a chance to respond to the SetWheelAngle command

        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower) // Turn off green LED
      );
    }

    /**
     * Main path - drive from the fence and start line, aim, score
     * staying to the right of another robot striaght in front of goal
     */
    public Command Path2Command() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new WaitCommand( 03), // give the vision coprocessor a chance to compute
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower)// Testing
         
        
        // to add: 
        /*
        // spin up the thrower to expected speed, in parallel.  
        // turn wheels to the angle we're about to drive, then drive where we want to shoot from
        // set thrower speed to vision distance
        new ParallelCommandGroup(  
          new SetThrowerSpeedCommand( m_Thrower, 4900), // speed from our table to score at this distance
          new SequentialCommandGroup(
            new SetWheelAngleCommand( m_swerve, -18), // FIX angle
            new WaitCommand( 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
            new DriveForDistanceCommand( m_swerve, -37, -12)  // FIX where we want to move to
          )
        ),
        
        new SpinUpThrowerCommand(m_Thrower, m_swerve, rft_),  // set thrower speed to vision distance
        new VisionAimCommand( m_swerve, rft_)
*/
        // to add:
        // ThrowToTarget while spinning Diasy 1 full rotation
      );
    }
/*
    @Override
    public void initialize() {   
    }

    @Override
    public boolean isFinished() {

        return false;
    }

    @Override
    public void end( boolean interrupted) {

    }
    */
}
