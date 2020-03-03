/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.*;
import frc.robot.util.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Vision objects
  private final SocketVisionWrapper rft_ = new SocketVisionWrapper("10.59.33.255", 5801);
  private final SocketVisionWrapper piece_ = new SocketVisionWrapper("10.59.33.255", 5805);
  private final SocketVisionSendWrapper sender_ = new SocketVisionSendWrapper("10.59.33.255", 5800);

  // Define subsystems here
  private final SwerveDriveSubsystem m_swerve = new SwerveDriveSubsystem();
  private final ClimbSubsystem m_Climb = new ClimbSubsystem();
  private final ThrowerSubsystem m_Thrower = new ThrowerSubsystem();
  private final HopperSubsystem m_Hopper = new HopperSubsystem();
  private final PatSajakSubsystem m_PatSajak = new PatSajakSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();

  // Color Sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  // DriveStation for GameSpecificMessage
  DriverStation m_station = DriverStation.getInstance();

  // Define Joysticks and Buttons here
  private final XboxController m_primaryController = new XboxController(0);
  private final XboxController m_secondaryController = new XboxController(1);

  private final JoystickButton m_primaryController_A = new JoystickButton(m_primaryController,
      XboxController.Button.kA.value); // Line up robot to the retroflective tape
  // want B to put climber arm down
  private final JoystickButton m_primaryController_B = new JoystickButton(m_primaryController,
      XboxController.Button.kB.value); // Line up robot with a power cell
  private final JoystickButton m_primaryController_LeftBumper = new JoystickButton(m_primaryController, 
      XboxController.Button.kBumperLeft.value); // turns off field orientation
  private final JoystickButton m_primaryController_RightBumper = new JoystickButton(m_primaryController, 
      XboxController.Button.kBumperRight.value); // Brake Mode Activate
  // want climber arm up
  private final JoystickButton m_primaryController_X = new JoystickButton(m_primaryController, XboxController.Button.kX.value);
  private final JoystickButton m_primaryController_Y = new JoystickButton(m_primaryController, XboxController.Button.kY.value); // Test on Control Panel Rotation
  // add d-pad up and for winch
  private final POVButton m_primaryController_DPad_Up = new POVButton(m_primaryController,0);
  private final POVButton m_primaryController_DPad_Down = new POVButton(m_primaryController, 180);
  // right trigger for shooter sequence with vision
  private final JoystickAnalogButton m_primaryController_RightTrigger = new JoystickAnalogButton(m_primaryController, 
      Hand.kRight, 0.5);
  // left trigger for shooter without vision
  private final JoystickAnalogButton m_primaryController_LeftTrigger = new JoystickAnalogButton(m_primaryController,
      Hand.kLeft, 0.5);
  
  // private final JoystickButton m_primaryController_Start = new JoystickButton(m_primaryController, XboxController.Button.kStart.value);
  // private final JoystickButton m_primaryController_Back = new JoystickButton(m_primaryController, XboxController.Button.kBack.value);

  private final JoystickButton m_secondaryController_Back = new JoystickButton(m_secondaryController, XboxController.Button.kBack.value);

  // private final JoystickButton m_secondaryController_StickLeft = new JoystickButton(m_secondaryController, XboxController.Button.kStickLeft.value); // runs sample color
  // want b to start Pat Sajak rotation control
  private final JoystickButton m_secondaryController_B = new JoystickButton(m_secondaryController, 
      XboxController.Button.kB.value);
  //want A to put Pat Sajak down
  private final JoystickButton m_secondaryController_A = new JoystickButton(m_secondaryController, 
      XboxController.Button.kA.value); //Launch all the power cells towards the high power port
  // want y to put Pat Sajak up
  private final JoystickButton m_secondaryController_Y = new JoystickButton(m_secondaryController, 
      XboxController.Button.kY.value); //Test to the Hopper
  // want x to start Pat Sajak position control
  private final JoystickButton m_secondaryController_X = new JoystickButton(m_secondaryController, 
      XboxController.Button.kX.value); // Move Daisy one slot
  private final JoystickButton m_secondaryController_LeftBumper = new JoystickButton(m_secondaryController, 
      XboxController.Button.kBumperLeft.value); // Intake up
  private final JoystickButton m_secondaryController_RightBumper = new JoystickButton(m_secondaryController, 
      XboxController.Button.kBumperRight.value); // Intake down
  private final JoystickButton m_secondaryController_Start = new JoystickButton(m_secondaryController, 
      XboxController.Button.kStart.value);
  // private final JoystickButton m_secondaryController_Back = new JoystickButton(m_secondaryController, XboxController.Button.kBack.value);
  // add d-pad up and d-pad down for daisy index and daisy unjame sequence respectively
  private final POVButton m_secondaryController_DPad_Up = new POVButton(m_secondaryController, 0);
  private final POVButton m_secondaryController_DPad_Down = new POVButton(m_secondaryController, 180);
  // right trigger for intake with daisy advance sequence(pick up the balls)
  // left trigger for intake reverse
  private final JoystickAnalogButton m_secondaryController_LeftTrigger = new JoystickAnalogButton( m_secondaryController, Hand.kLeft, 0.5);
  private final JoystickAnalogButton m_secondaryController_RightTrigger = new JoystickAnalogButton( m_secondaryController, Hand.kRight, 0.5);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the default commands
    configureDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  public SocketVisionWrapper getRftSocketReader() {
    return rft_;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //left bumper switches from field orientated to robot oriented // has a fun lambda
    m_primaryController_LeftBumper.whenPressed(
        new InstantCommand(()-> {m_swerve.setFieldOriented(false);})
      ).whenReleased(
        new InstantCommand(()-> {m_swerve.setFieldOriented(true);})
      );

    // Put brake mode on a button!
    m_primaryController_RightBumper.whenPressed(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve)
      ).whenReleased(
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)
      );

    // Thrower on primary controller, Left Trigger to throw without vision, Right Trigger throw with vision
    // Left Trigger: spin up the thrower to the right speed //, then hold it there while simultaneously spinning the Daisy one full rotation
      m_primaryController_DPad_Up.whenHeld(
        new SequentialCommandGroup(
          new SetThrowerSpeedCommand(m_Thrower, -ThrowerLUT.DEFAULT_RPM).perpetually()
          /*new ParallelCommandGroup( 
            new SetThrowerSpeedCommand( m_Thrower, -ThrowerLUT.DEFAULT_RPM),
            new MoveHopperCommand(m_Hopper, 6)
          )*/
        )
      );

      // Thrower on primary controller, Right Trigger to throw with vision
      // turn on LED, command vision processor to track RFT, spin up thrower based on RFT distance
      // once thrower is at the speed, keep it at the speed based on RFT distance and simlutanously spin Daisy one rotation
      m_primaryController_RightTrigger.whenHeld(  // by using whenHeld, the command gets canceled when the 'button' is released
        //new UnloadCommand( m_swerve, m_Thrower, m_Hopper, sender_, rft_, 0.5)
        new SequentialCommandGroup(
        //new InstantCommand(m_Thrower::turnOnLED, m_Thrower),
       // new WaitCommand(2),
        //new VisionAimGyroCommand( m_swerve, rft_) // aim the robot
        //)
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new WaitCommand(0.5), // give the vision processor a chance to find the RFT
        
        new VisionAimGyroCommand( m_swerve, rft_), // aim the robot
        new ThrowToTargetCommand(m_Thrower, m_swerve, rft_)
        /*      
        new SequentialCommandGroup(
        //new InstantCommand(m_Thrower::turnOnLED, m_Thrower),
       // new WaitCommand(2),
        //new VisionAimGyroCommand( m_swerve, rft_) // aim the robot
        //)
        new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new WaitCommand(0.5), // give the vision processor a chance to find the RFT
            //new ParallelCommandGroup( // waits for both to end
               // new SpinUpThrowerCommand(m_Thrower, m_swerve, rft_),  // set thrower speed to vision distance, end when it's there
                new VisionAimGyroCommand( m_swerve, rft_), // aim the robot
                new WaitCommand(0.5),    
            //new ParallelRaceGroup(
                new ThrowToTargetCommand(m_Thrower, m_swerve, rft_)  // never ends
                //new MoveHopperCommand(hopper, 6)
            //),
              //) 
        )
      )
            .whenReleased(
              new SequentialCommandGroup(              
                new SetThrowerSpeedCommand(m_Thrower, 0),
                new SendVisionCommand(sender_, "_"),
               new InstantCommand(m_Thrower::turnOffLED, m_Thrower) // Turn on green LED
              */
              )
              
        );  
        
    
    // Pat Sajak commands.
    m_secondaryController_A.whenPressed(
      new InstantCommand(m_PatSajak::lowerSpinner, m_PatSajak)
    );
    m_secondaryController_B.whenPressed(
      new InstantCommand(m_PatSajak::raiseSpinner, m_PatSajak)
    );
    m_secondaryController_Y.whenPressed(
      new ControlPanelRotation(m_PatSajak, m_colorSensor)
    );
    m_secondaryController_X.whenPressed(
      new ControlPanelSpinSimple(m_PatSajak) // simple for testing
      // new ControlPanelPosition(m_PatSajak, m_colorSensor)
    );
    m_secondaryController_X.whenReleased(
      new InstantCommand(m_PatSajak :: turnSpinnerMotorOff, m_PatSajak)
    );

    m_secondaryController_Back.whenPressed(
      new InstantCommand(m_Thrower::turnOffLED, m_Thrower)
    );
    //
    // Intake
    // Intake forward/reverse are on 2nd game controller, left joystick, Y axis
    m_secondaryController_RightBumper.whenPressed(
      new InstantCommand(m_Intake :: lowerIntake, m_Intake)
    );
    m_secondaryController_LeftBumper.whenPressed(
      new InstantCommand(m_Intake :: raiseIntake, m_Intake)
    );

    //
    // m_secondaryController_RightTrigger  Intake w/ Daisy Advanced sequence
    //
    // enable (and test) when we have solid non-jamming Daisy operation
    //
    // if Daisy isn't full, run intake beater bar inward until a ball is in the loading slot,
    //   and then if have less than 5 balls pulse the beater bar in reverse and move Daisy one index forward
    //     otherwise reverse the beater bar
    // if Daisy is full, run intake beater bar in reverse
    //
    m_secondaryController_RightTrigger.whileHeld( 
      new IntakeAdvDaisyCommand( m_Intake, m_Hopper)
    ).whenReleased( // On release lift the intake, then outtake at 0.7 power for 1.5 seconds. Note that beforeStarting is a decorator that is written after the command body...
      new ParallelRaceGroup(
        new RunCommand(()->{m_Intake.setMotor(-0.7);}, m_Intake),
        new WaitCommand(1.5)
      ).beforeStarting(m_Intake::raiseIntake, m_Intake)
    ) ;


    //
    // Hopper (Daisy)
    //
    m_secondaryController_Start.whenPressed(new MoveHopperCommand(m_Hopper, 6));
    m_secondaryController_DPad_Up.whenPressed( 
        new SequentialCommandGroup( 
          new BumpHopperCommand( m_Hopper),
          new WaitCommand(.2),
          new MoveHopperCommand(m_Hopper,1)
        )
      ).whileHeld(m_Hopper :: smartDashIndex); //m_Hopper ); // Index ?; Commented out requirements so the print command doesn't interfere with the move commands
    m_secondaryController_DPad_Down.whenPressed(new MoveHopperCommand(m_Hopper, -1));
    m_secondaryController_LeftTrigger.whileHeld(
      new InstantCommand( m_Hopper::moveForwardSlowly, m_Hopper)
    ).whenReleased( new InstantCommand( m_Hopper::stopMotor, m_Hopper)); // stop

    //
    //Climb
    //
    // control winch via default command, so can read trigger %
    // m_primaryController_LeftTrigger.whileHeld(new ClimbWinchUpCommand(m_Climb)); // control
    m_primaryController_DPad_Down.whileHeld(new ClimbWinchDownCommand(m_Climb));
    m_primaryController_B.whenPressed(
      new InstantCommand(m_Climb::raiseArm, m_Climb)
    );
    m_primaryController_A.whenPressed(
      new InstantCommand(m_Climb::lowerArm, m_Climb)
    );
    m_primaryController_X.whenPressed(
      new InstantCommand(m_Climb::extendArm, m_Climb)
    );
    m_primaryController_Y.whenPressed(
      new InstantCommand(m_Climb::retractArm, m_Climb)
    );

    //Tells the output of the light sensor used for telling if powercell is in daisy
    //m_secondaryController_Y.whileHeld(new InstantCommand(()->{SmartDashboard.putNumber("LightSensor output voltage", m_lightSensor.getVoltage());}));
  }

  private void configureDefaultCommands() {
    // Using StartEnd commands because by default they do not have isFinished return true, unlike InsantCommands. Alternative is to use the perpetually() decorator.
    // default swerve drive is to read from joysticks
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(m_swerve, m_primaryController));
    // winch on primary controller, left trigger
    m_Climb.setDefaultCommand(new DefaultWinchCommand(m_Climb, m_primaryController));
    
    // intake on secondary controller, Y axis of Left joystick
    m_Intake.setDefaultCommand(new DefaultIntakeCommand(m_Intake, m_secondaryController));
    // default thrower is to spin down to still
    m_Thrower.setDefaultCommand(new StartEndCommand( ()->{m_Thrower.stopThrower(); m_Thrower.turnOffLED();}, ()->{}, m_Thrower)); // Spin down thrower and turn off LED on startup, do nothing on end.

  }

  /**
   * This grabs the game message from the Driver Station
   * @return The game specific message.
   */
  public String getGameSpecificMessage(){
    return m_station.getGameSpecificMessage();
  }

  /**
   * reset the hopper reference point, called from Robot.autoInit() and .teleopInit()
   */
  public void resetHopperReference( boolean moveToSlot) {
    m_Hopper.resetReference(); // set encoder back to range [0, ONE_ROTATION), i.e. (0,360)
    m_Hopper.selectNearestSlot( moveToSlot); // set the index to match the current position of the Daisy
  }

  /**
   * Call this to initialize the SocketVision objects. Must be called in periodInit() methods
   * if you want to use vision in that periodPeriodic() time (where period is autonomous or teleop).
   */
  public void visionInit(){
    sender_.init();
    rft_.init();
    piece_.init();
  }

  /**
   * Call this to shut down the SocketVision objects. Must be called in disabledInit() to reduce stray threads
   * running in disabled.
   */
  public void visionShutDown(){
    sender_.shutDown();
    rft_.shutDown();
    piece_.shutDown();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    Paths p = new Paths( m_swerve,m_Thrower, m_Hopper, m_Intake, sender_, rft_);
    Command autoCommand = new SequentialCommandGroup(
      new InstantCommand( m_Hopper::setBallCountTo3, m_Hopper),
      //p.PathTestCommand()
      p.Path1Command()
    );
    return autoCommand;
    /*
    Command autoCommand = new SequentialCommandGroup(
      new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
      new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
      new SendVisionCommand(sender_, "R"), // tell vision coprocessor to track the RFT
      new SetWheelAngleCommand( m_swerve, -18-90),  // point the wheels in the direction we want to go
      new WaitCommand( 2), // 0.2), // give the drivetrain a chance to respond to the SetWheelAngle command
      new InstantCommand( m_swerve::setDrivePIDToSlow, m_swerve), // test doing DriveForDist at slow speed
      new DriveForDist2910Command( m_swerve, -37, -12), // go to destination 
      new InstantCommand( m_swerve::setDrivePIDToFast, m_swerve), // put DriveForDist at regular speed
      //new WaitCommand( 0.1), // give vision coprocessor a chance to find the target
      // TODO: UnloadCommand().  remove VisionAim and any last WaitCommand()
      new WaitCommand( 1),
      new ParallelCommandGroup( // waits for both to end
                new SpinUpThrowerCommand(m_Thrower, m_swerve, rft_),  // set thrower speed to vision distance, end when it's there
                new VisionAimGyroCommand( m_swerve, rft_) // aim the robot
      ),
      new WaitCommand( 1),// give the drivetrain a chance to respond to the SetWheelAngle command
      new ParallelRaceGroup(
                new ThrowToTargetCommand(m_Thrower, m_swerve, rft_),  // never ends
                new MoveHopperCommand(m_Hopper, 6)
      ),
      new SetThrowerSpeedCommand(m_Thrower, 0),
            
      // very last thing
      new InstantCommand(m_Thrower::turnOffLED, m_Thrower) // Turn off green LED
    );
    return autoCommand;
    */
  }
}
