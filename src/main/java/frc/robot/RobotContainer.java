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
import frc.robot.commands.ClimbWinchDownCommand;
import frc.robot.commands.ClimbWinchUpCommand;
import frc.robot.commands.ControlPanelPosition;
import frc.robot.commands.ControlPanelRotation;
import frc.robot.commands.ControlPanelSpinSimple;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.commands.IntakeRecieveCommand;
import frc.robot.commands.SampleColorCommand;
import frc.robot.commands.SendVisionCommand;
import frc.robot.commands.SetThrowerSpeedCommand;
import frc.robot.commands.SpinUpThrowerCommand;
import frc.robot.commands.ThrowToTargetCommand;
import frc.robot.commands.VisionApproachTarget;
import frc.robot.commands.VisionLineUpWithTarget;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.util.SocketVisionSendWrapper;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.util.JoystickAnalogButton;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PatSajakSubsystem;
import frc.robot.commands.MoveHopperCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;;


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
  private final AnalogInput m_lightSensor = new AnalogInput(0);
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
  private final JoystickButton m_primaryController_X = new JoystickButton(m_primaryController,
      XboxController.Button.kX.value);
  private final JoystickButton m_primaryController_Y = new JoystickButton(m_primaryController, 
      XboxController.Button.kY.value); // Test on Control Panel Rotation
  // add d-pad up and for winch
  private final POVButton m_primaryController_DPad_Up = new POVButton(m_primaryController,0);
  private final POVButton m_primaryController_DPad_Down = new POVButton(m_primaryController, 180);
  // right trigger for shooter sequence with vision
  private final JoystickAnalogButton m_primaryController_RightTrigger = new JoystickAnalogButton(m_primaryController, 
      Hand.kRight, 0.5);
  // left trigger for shooter without vision
  private final JoystickAnalogButton m_primaryController_LeftTrigger = new JoystickAnalogButton(m_primaryController,
      Hand.kLeft, 0.5);
  private final JoystickButton m_primaryController_Start = new JoystickButton(m_primaryController, 
      XboxController.Button.kStart.value);
  private final JoystickButton m_primaryController_Back = new JoystickButton(m_secondaryController, 
      XboxController.Button.kBack.value);

  private final JoystickButton m_secondaryController_StickLeft = new JoystickButton(m_secondaryController,
      XboxController.Button.kStickLeft.value); // runs sample color
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
  private final JoystickButton m_secondaryController_Back = new JoystickButton(m_secondaryController, 
      XboxController.Button.kBack.value);
  // add d-pad up and d-pad down for daisy index and daisy unjame sequence respectively
  private final POVButton m_secondaryController_DPad_Up = new POVButton(m_secondaryController, 0);
  private final POVButton m_secondaryController_DPad_Down = new POVButton(m_secondaryController, 180);
  // right trigger for intake with daisy advance sequence(pick up the balls)
  // left trigger for intake reverse
  private final JoystickAnalogButton m_secondaryController_LeftTrigger = new JoystickAnalogButton( m_primaryController, Hand.kLeft, 0.5);
  private final JoystickAnalogButton m_secondaryController_RightTrigger = new JoystickAnalogButton( m_primaryController, Hand.kRight, 0.5);
  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the default commands
    configureDefaultCommands();
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Complex commands
    m_primaryController_A.whenPressed(      
      new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new VisionLineUpWithTarget(m_swerve, rft_), 
        new SendVisionCommand(sender_, "_")
      ) 
    );

    m_primaryController_X.whenPressed( // Inline command group!
      new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SendVisionCommand(sender_, "G"), // Can't be a lambda because Sender's aren't subsystems
        new VisionLineUpWithTarget(m_swerve, piece_), 
        new SendVisionCommand(sender_, "_")
      )
    );

    //left bumper is field orientated // has a fun lambda
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

    

    /*m_secondaryController_StickLeft.whileHeld(
      new SampleColorCommand(m_colorSensor)
    );*/

    /*m_primaryController_Y.whenPressed(
      new ControlPanelRotation(m_PatSajak, m_colorSensor)
    );*/

    // Thrower on primary controller, Left Trigger to throw without vision, Right Trigger throw with vision
    // spin up the thrower to the right speed, then hold it there while simultaneously spinning the Daisy one full rotation
      m_primaryController_LeftTrigger.whenHeld(
        new SequentialCommandGroup(
          new SetThrowerSpeedCommand(m_Thrower, 700), //.perpetually() // m_Thrower.getThrowerSpeed())
          new ParallelCommandGroup( 
            new SetThrowerSpeedCommand( m_Thrower, 700),
            new MoveHopperCommand(m_Hopper, 6)
          )
        )
      );

      // Thrower on primary controller, Right Trigger throw with vision
      // turn on LED, command vision processor to track RFT, spin up thrower based on RFT distance
      // once thrower is at the speed, keep it at the speed based on RFT distance and simlutanously spin Daisy one rotation
      m_primaryController_RightTrigger.whenHeld(  // by using whenHeld, the command gets canceled when the 'button' is released
        new SequentialCommandGroup(
          new InstantCommand(m_Thrower::turnOnLED, m_Thrower), // Turn on green LED
          new SendVisionCommand(sender_, "R"), // tell Vision Coprocessor to track RFT
          // TODO: drivetrain rotate to target, Parallel Command Group (simultaneously) with spin up thrower.
          new SpinUpThrowerCommand(m_Thrower, rft_),
          new ParallelRaceGroup(
            new ThrowToTargetCommand(m_Thrower, rft_),
            new MoveHopperCommand(m_Hopper, 6)
          )
          // Turning off LED is handled by thrower default command
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

    // Intake
    m_secondaryController_Start.whenPressed(
      new IntakeRecieveCommand(m_Intake)
    ).whenReleased(m_Intake :: stopMotor, m_Intake);
    // m_secondaryController_RightTrigger  Intake w/ Daisy Advanced sequence
    m_secondaryController_RightTrigger.whileHeld(
      new SequentialCommandGroup(
          new InstantCommand(m_Intake :: lowerIntake, m_Intake),
          new ParallelCommandGroup( 
            new IntakeRecieveCommand(m_Intake),
            new MoveHopperCommand(m_Hopper, 5)
          )
        )
      );
    m_secondaryController_RightBumper.whenPressed(
      new InstantCommand(m_Intake :: lowerIntake, m_Intake)
    );
    m_secondaryController_LeftBumper.whenPressed(
      new InstantCommand(m_Intake :: raiseIntake, m_Intake)
    );

    // Hopper (Daisy)
    m_secondaryController_Back.whenPressed(
      new MoveHopperCommand(m_Hopper, 1)
    );
    m_secondaryController_LeftTrigger.whenHeld(
      new InstantCommand( m_Hopper::moveForwardSlowly, m_Hopper)
    ); 
    m_secondaryController_LeftTrigger.whenReleased(
      new InstantCommand( m_Hopper::stopMotor, m_Hopper)
    ); // stop
    //m_secondaryController_DPad_Up.whenHeld(m_Hopper :: ); // Index ?

    //Climb
    m_primaryController_DPad_Up.whileHeld(new ClimbWinchUpCommand(m_Climb));
    m_primaryController_DPad_Down.whileHeld(new ClimbWinchDownCommand(m_Climb));
    
    m_primaryController_B.whenPressed(
      new InstantCommand(m_Climb::raiseArm, m_Climb)
    );

    m_primaryController_A.whenPressed(
      new InstantCommand(m_Climb::lowerArm, m_Climb)
    );

    /*m_primaryController_Start.whileHeld(
      new ClimbWinchUpCommand(m_Climb)
    );

    m_primaryController_Back.whileHeld(
      new ClimbWinchDownCommand(m_Climb)
    ); */ 


    /* example how to aim the robot at the RFT and spin up the thrower at the same time
    m_secondaryController_A.whenPressed(
      new SequentialCommandGroup(
        new ParallelCommandGroup(
          new SetThrowerSpeedCommand(m_Thrower, 500),
          // Angle the robot toward the retroflective tape
          new SequentialCommandGroup(
            new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
            new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
            new VisionLineUpWithTarget(m_swerve, rft_), 
            new SendVisionCommand(sender_, "_")
          )
        ),
        new SequentialCommandGroup(
          //Spin the daisy and reset the Ball Count to 0
          // Stop thrower
        )
      )
    );  
    */

    /*m_secondaryController_X.whenPressed(new MoveHopperCommand(m_Hopper, 1));
    m_secondaryController_X.whenReleased(new InstantCommand( m_Hopper::stopMotor, m_Hopper)); // stop
    // for testing, to check whether to invert the motor or the sensor
    m_secondaryController_Y.whenHeld(new InstantCommand( m_Hopper::moveForwardSlowly, m_Hopper)); 
    m_secondaryController_Y.whenReleased(new InstantCommand( m_Hopper::stopMotor, m_Hopper)); // stop
    *///The sequence for loading 5 powercells into the daisy
    /*m_secondaryController_X.whileHeld(
        new SequentialCommandGroup(
          //if the ball count is less than five
            new SequentialCommandGroup(
              // Lower intake command
              //spin intake forward
              // put power cell in daisy
            ),
            new ParallelCommandGroup(
              new SequentialCommandGroup(
                // move the hopper into the next slot
                // Ball count ++
              )
              //spin the intake backwards (little bit)
            ),
          //else the ball count is 5 (hopefully not higher)
            new SequentialCommandGroup(
              //raise intake
              //spin backwards
          )
        )
        
    );*/

    //Tells the output of the light sensor used for telling if powercell is in daisy
    //m_secondaryController_Y.whileHeld(new InstantCommand(()->{SmartDashboard.putNumber("LightSensor output voltage", m_lightSensor.getVoltage());}));
  }

  private void configureDefaultCommands() {
    // Using StartEnd commands because by default they do not have isFinished return true, unlike InsantCommands. Alternative is to use the perpetually() decorator.
    // default swerve drive is to read from joysticks
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(m_swerve, m_primaryController));
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
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command autoCommand = new VisionApproachTarget(m_swerve, rft_, 100, 5, 5);
    
    return autoCommand;
  }
}
