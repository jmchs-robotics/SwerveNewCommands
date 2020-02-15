package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.util.*;
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

    public Paths( SwerveDriveSubsystem swerve, ThrowerSubsystem thrower, SocketVisionSendWrapper sender) {
        m_swerve = swerve;
        m_Thrower = thrower;
        sender_ = sender;           
    }

    /**
     * Test 
     */
    public Command Path1Command() {
      return new SequentialCommandGroup(
        new WaitCommand( 1), // give the drivetrain a chance to respond to the SetWheelAngle command
        
        new SetWheelAngleCommand( m_swerve, 45), // FIX angle
        new WaitCommand( 1), // give the drivetrain a chance to respond to the SetWheelAngle command
        new DriveForDist2910Command( m_swerve, 15, 15)  // FIX where we want to move to
      );
    }
    /**
     * Main path - drive from the fence and start line, aim, score
     */
    public Command Path2Command() {
        // this.m_swerve = m_swerve;

        return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new WaitCommand( 03), // give the vision coprocessor a chance to compute
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower) // Turn on green LED
        
        // to add: 
        /*
        // spin up the thrower to anticipated speed, in parallel.  
        // turn wheels to the angle we're about to drive, then drive where we want to shoot from
        // set thrower speed to vision disance
        new ParallelCommandGroup(  
          new SetThrowerSpeedCommand( m_Thrower, 5000), // FIX rpm.  speed from our table to score at this distance
          new SequentialCommandGroup(
            new SetWheelAngleCommand( m_swerve, -15), // FIX angle
            new WaitCommand( 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
            new DriveForDistanceCommand( m_swerve, -15, -15)  // FIX where we want to move to
          )
        ),
        
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new SpinUpThrowerCommand(m_Thrower, rft_),  // set thrower speed to vision distance
        */
        /*
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new WaitCommand( 0.1), // give the vision coprocessor a chance to compute
        new VisionAim( m_swerve, rft_, 18, 18)
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
