package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.*;
import frc.robot.util.*;
//import jdk.vm.ci.code.InstalledCode;
// import sun.tools.tree.InstanceOfExpression;
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
    HopperSubsystem m_Hopper;
    IntakeSubsystem m_Intake;
    SocketVisionSendWrapper sender_;
    SocketVisionWrapper rft_;

    public Paths( SwerveDriveSubsystem swerve, ThrowerSubsystem thrower, HopperSubsystem hopper, IntakeSubsystem intake, SocketVisionSendWrapper sender, SocketVisionWrapper rft) {
        m_swerve = swerve;
        m_Thrower = thrower;
        sender_ = sender;   
        rft_ = rft;     
        m_Hopper = hopper;   
        m_Intake = intake;
    }


    /**
     * test stuff
     * @return
     */
    public Command PathTestCommand() {
      return new SequentialCommandGroup(
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new WaitCommand( 1), 
        new BumpHopperCommand(m_Hopper),
        new WaitCommand(.2),
        new MoveHopperCommand(m_Hopper, 1),
        new WaitCommand( 1),// give the drivetrain a chance to respond to the SetWheelAngle command
        new BumpHopperCommand(m_Hopper),
        new WaitCommand(.2),
        new MoveHopperCommand(m_Hopper, 1),
        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower) // Turn off green LED
      );
    }

    /**
     * test stuff
     * @return
     */
    public Command TestUnload() {
      return new UnloadCommand(m_swerve, m_Thrower, m_Hopper, sender_, rft_, 1);
    }

    /**
     * Go from fence to scoring position, staying right of another robot striaght in front of goal 
     * aim, score
     */
    public Command Path1Command() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::holonomicDriveToZero, m_swerve),
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
        new SetWheelAngleCommand( m_swerve, -18-90),  // point the wheels in the direction we want to go
        new WaitCommand( 0.2), // 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), // test doing DriveForDist at slow speed
        new DriveForTime2910Command(m_swerve, 1, .2, -.5), // this drives pretty close to -57, -12
        //new DriveForDist2910Command( m_swerve, -57, -12), // go to destination 94 - (25+6.5)/2 - 28
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new WaitCommand( 1), // give vision coprocessor a chance to find the target
        new SetPoseAngle2910Command( m_swerve, 10),
        new WaitCommand( 1), // give vision coprocessor a chance to find the target
        // TODO: UnloadCommand().  remove VisionAim and any last WaitCommand()
        new VisionAimGyroCommand( m_swerve, rft_), // aim the robot
        new ParallelRaceGroup(
          new ThrowToTargetCommand(m_Thrower, m_swerve, rft_),  // never ends
            new SequentialCommandGroup( 
              new WaitCommand(2),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1),
              new BumpHopperCommand( m_Hopper),
              new WaitCommand(.2),
              new MoveHopperCommand(m_Hopper,1),
              new WaitCommand(1)
            )
          ),
        //new SetThrowerSpeedCommand(m_Thrower, 0),
              
        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower), // Turn off green LED
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)
      );
    }

    /**
     * after Path1, drive into our own trench and retrieve first 3 balls.  We would only do this if our starting position is nearest the trench among our alliance.
     * — drive completely on encoders
     * — someday drive on Vision of balls
     * — end command via encoders, or someday by counting balls
     * -- Someday set pose angle absolute, then strafe to near the first ball, then a command to “drive by vision toward the ball until the ball is acquired or we hit something (accelerometer impact)” command
     * return to target range and shoot them
     * -- need to drive to a field position, probably only one move (i.e. direct angle) or maybe 2, then rotate to straight at target, then setPoseAngleToVisionRFT
     * @return
     */
    public Command PathCCommand() {
      return new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SetWheelAngleCommand( m_swerve, Math.atan2( 57-28, -(86-12-(34+6.5)/2))),  // point the wheels in the direction we want to go
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new DriveForDist2910Command( m_swerve, 57-28, -(86-12-(34+6.5)/2)), // go to front side of trench, aligned with balls
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new SetPoseAngle2910Command(m_swerve, -90), // point intake at the balls by turning left 90 degrees
        new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), 
        new InstantCommand(m_Intake::lowerIntake, m_Intake),
        new ParallelRaceGroup( 
          new IntakeAdvDaisyCommand(m_Intake, m_Hopper), // intake and auto-advance the Daisy
          new DriveForDist2910Command( m_swerve, 0, -(108+12)) // drive through 3 balls
        ),
        new InstantCommand(m_Intake::raiseIntake, m_Intake),
        new SetWheelAngleCommand( m_swerve, Math.atan2( -(57-28), 86-12-(34+6.5)/2 + 108+12)),  // point the wheels in the direction we want to go
        new DriveForDist2910Command( m_swerve, -(57-28), 86-12-(34+6.5)/2 + 108+12),
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
        new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
        new SetPoseAngle2910Command(m_swerve, -5),
        
        new ParallelCommandGroup( // waits for both to end
                  new SpinUpThrowerCommand(m_Thrower, m_swerve, rft_),  // set thrower speed to vision distance, end when it's there
                  new VisionAimGyroCommand( m_swerve, rft_) // aim the robot
        ),
        new ParallelRaceGroup( // ends when first command ends
          new ThrowToTargetCommand(m_Thrower, m_swerve, rft_),  // never ends
          new SequentialCommandGroup(
            new BumpHopperCommand(m_Hopper),
            new MoveHopperCommand(m_Hopper, 6)
          )
        ),
        new SetThrowerSpeedCommand(m_Thrower, 0),
    
        // very last thing
        new InstantCommand(m_Thrower::turnOffLED, m_Thrower), // Turn off green LED
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)

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
