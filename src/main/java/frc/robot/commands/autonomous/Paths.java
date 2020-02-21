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
    
    /**
     * PathG
     * Start against left fence, with front of robot against the fence 
     * and intake toward our driver station (-90 degrees from front-facing).
     * Robot on our driver's side of initiation line
     */
    /*
    PathG
    // change field orientation offset by -90
    // angle wheels at -95, to drive mostly left and a little backwards (robot oriented), i.e. backwards and a little right (field oreinted)
    // 130 = initiation line to table edge.  31.5 = width of robot.  12.5 = beater bar outside the robot + buffer distance
    // 8 = (56 - 40.5) / 2 = (table width - robot length) / 2
    // field oriented, drive at full speed backwards (130 - 31.5 - 12.5) and right 8
    // intake down
    // turn robot -20 degrees (robot oriented), which makes ball nearer fence 3.4" closer to the beater bar and the ball closer to the center of the field 3.4" farther from the beater bar
    // angle wheels at -70
    // intake on (inward)
    // 8 = buffer distance. 7 = ball diameter. 7 = 2nd ball diameter.
    // drive at slow speed backwards ( 8+7+7) and zero to the right
    // pause, to intake second ball
    // ange wheels at 
    // 162 = half field width. 8 = how far we already moved from fence. 20 = half the robot's length. 
    // 65 = center of field to center of goal. 20 = half the robot's lenght. 14 = safety distance if alliance robot is aligned center of goal and still on initiation line.
    // drive at full speed forward ((130 - 31.5 - 12.5) + ( 8+7+7) - 6) and right (162-8-20+65-20-14)
    // turn robot +20 degrees (robot oriented) to face forwards
    // VisionAim and Unload
    */
}
