/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.commands.DischargeAllCommand;
import frc.robot.commands.SampleColorCommand;
import frc.robot.commands.SendVisionCommand;
import frc.robot.commands.SpinThreeTimesCommand;
import frc.robot.commands.SpinUpThrowerCommand;
import frc.robot.commands.ThrowToTargetCommand;
import frc.robot.commands.VisionApproachTargetCommand;
import frc.robot.commands.VisionLineUpWithTargetCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ThrowerSubsystem;
import frc.robot.util.SocketVisionSendWrapper;
import frc.robot.util.SocketVisionWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define subsystems here
  private final SwerveDriveSubsystem m_swerveDrive = new SwerveDriveSubsystem();
  private final ThrowerSubsystem m_thrower = new ThrowerSubsystem();
  private final HopperSubsystem m_hopper = new HopperSubsystem();
  private final ControlPanelSubsystem m_controlPanel = new ControlPanelSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // Vision objects
  private final SocketVisionWrapper rft_ = new SocketVisionWrapper("10.59.33.255", 5801);
  private final SocketVisionWrapper piece_ = new SocketVisionWrapper("10.59.33.255", 5805);
  private final SocketVisionSendWrapper sender_ = new SocketVisionSendWrapper("10.59.33.255", 5800);

  // DriveStation for GameSpecificMessage
  DriverStation m_station = DriverStation.getInstance();

  // Define Joysticks and Buttons here
  private final XboxController m_primaryController = new XboxController(0);
  private final XboxController m_secondaryController = new XboxController(1);

  private final JoystickButton m_primaryController_A = new JoystickButton(m_primaryController,
      XboxController.Button.kA.value);
  private final JoystickButton m_primaryController_B = new JoystickButton(m_primaryController,
      XboxController.Button.kB.value);
  private final JoystickButton m_primaryController_LeftBumper = new JoystickButton(m_primaryController, 
      XboxController.Button.kBumperLeft.value);
  private final JoystickButton m_primaryController_RightBumper = new JoystickButton(m_primaryController,
      XboxController.Button.kBumperRight.value);
  private final JoystickButton m_primaryController_Y = new JoystickButton(m_primaryController,
      XboxController.Button.kY.value);

  private final JoystickButton m_secondaryController_StickLeft = new JoystickButton(m_secondaryController,
      XboxController.Button.kStickLeft.value);
  private final JoystickButton m_secondaryController_A = new JoystickButton(m_secondaryController, 
      XboxController.Button.kA.value);
  private final JoystickButton m_secondaryController_B = new JoystickButton(m_secondaryController,
      XboxController.Button.kB.value);
  private final JoystickButton m_secondaryController_Y = new JoystickButton(m_secondaryController, 
      XboxController.Button.kY.value);
  private final JoystickButton m_secondaryController_X = new JoystickButton(m_secondaryController, 
      XboxController.Button.kX.value);
  private final JoystickButton m_secondaryController_LeftBumper = new JoystickButton(m_secondaryController,
      XboxController.Button.kBumperLeft.value);
  private final JoystickButton m_secondaryController_RightBumper = new JoystickButton(m_secondaryController,
      XboxController.Button.kBumperRight.value);

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
    m_primaryController_A.whenPressed(      
      new SequentialCommandGroup(
        new InstantCommand(m_swerveDrive::setBrakeOn, m_swerveDrive), // Brake mode on!
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new VisionLineUpWithTargetCommand(m_swerveDrive, rft_), 
        new SendVisionCommand(sender_, "_")
      )
    );

    m_primaryController_B.whenPressed( // Inline command group!
      new SequentialCommandGroup(
        new InstantCommand(m_swerveDrive::setBrakeOn, m_swerveDrive), // Brake mode on!
        new SendVisionCommand(sender_, "G"), // Can't be a lambda because Sender's aren't subsystems
        new VisionLineUpWithTargetCommand(m_swerveDrive, piece_), 
        new SendVisionCommand(sender_, "_")
      )
    );

    // Put brake mode on a button!
    m_primaryController_RightBumper.whenPressed(
      new InstantCommand(m_swerveDrive::setBrakeOn, m_swerveDrive)
    ).whenReleased(
      new InstantCommand(m_swerveDrive::setBrakeOff, m_swerveDrive)
    );

    // Put field orientation on a button.
    m_primaryController_LeftBumper.whenPressed(
      new InstantCommand(() -> m_swerveDrive.setFieldOriented(false), m_swerveDrive) // Note the use of lambdas to pass parameters to the subsystem method.
    ).whenReleased(
      new InstantCommand(()-> m_swerveDrive.setFieldOriented(true), m_swerveDrive)
    );

    // Put accumulate & print output on a button!
    m_primaryController_Y.whileHeld(
      new ParallelCommandGroup(
        new InstantCommand(m_swerveDrive::accumulatePosition), // No need to state that this uses the swerve subsystem b/c it's only a sensor read.
        new InstantCommand(() -> SmartDashboard.putNumberArray("Robot Position: ", m_swerveDrive.getPosition())) // Use a lambda to get at SmartDashboard
      )
    );

    m_secondaryController_StickLeft.whileHeld(
      new SampleColorCommand(m_controlPanel)
    );

    // Throw at the RFT target on a button
    m_secondaryController_A.whenPressed(
      new SequentialCommandGroup(
        new SpinUpThrowerCommand(m_thrower, rft_),
        new ParallelRaceGroup(
          new ThrowToTargetCommand(m_thrower, rft_),
          new DischargeAllCommand(m_hopper)
        )
      )
    );

    // Put intake on the secondary B button for now. Note the use of lambdas to do two things at start & end each.
    m_secondaryController_B.whileHeld(
      new ConditionalCommand(
        new SequentialCommandGroup( // First lower the intake, then in parallel run the intake while checking for a new ball to enter the daisy.
          new InstantCommand(m_intake::lowerIntake, m_intake),
          new ParallelRaceGroup(
            new StartEndCommand(()->{m_intake.setMotor(0.7);},()->{m_intake.setMotor(-0.2);}, m_hopper), // Note that there is no interrupted qualifier for the onEnd Runnable
            new WaitUntilCommand(()->{return m_hopper.ballLoaded();}) // Color sensor proximity is an 11 bit value that is low when the target is far and high when it is near.
          ),
          new FunctionalCommand(
            ()->{m_hopper.nextSlot(); m_hopper.incStoredCount();}, // Move hopper to next slot
            ()->{m_intake.setMotor(-0.2);}, // Back drive the intake motor slowly.
            (interrupted)->{ if(interrupted) m_intake.raiseIntake(); }, // If interrupted, get the intake back up.
            ()->{return m_hopper.atSetpoint(5);}, // 5 encoder ticks of leeway
            m_hopper, m_intake // Requires hopper & intake
           ) 
        ),
        new PerpetualCommand(new InstantCommand(()->{m_intake.raiseIntake(); m_intake.setMotor(-0.7);})), // Raise intake and backdrive if there are 5 balls in the daisy
        () -> {return m_hopper.getStoredCount() < 5;} // The conditional to branch on: if there are less than 5 balls, intake; else, raise intake and backdrive
      )  
    ).whenReleased( // On release lift the intake, then outtake at 0.7 power for 1.5 seconds. Note that beforeStarting is a decorator that is written after the command body...
      new ParallelRaceGroup(
        new RunCommand(()->{m_intake.setMotor(-0.7);}, m_intake),
        new WaitCommand(1.5)
      ).beforeStarting(m_intake::raiseIntake, m_intake)
    );

    // Raise the climber arm on the X button (lower the arm on release)
    m_secondaryController_X.whenPressed(
      new InstantCommand(m_climber::liftClimber, m_climber)
    ).whenReleased(
      new InstantCommand(m_climber::lowerClimber, m_climber)
    );

    // Run the climb motor on the Y button
    m_secondaryController_Y.whenHeld(
      new RunCommand(m_climber::climb, m_climber)
    );

    // Put spin to color on the left bumper
    m_secondaryController_LeftBumper.whileHeld(
      new SequentialCommandGroup(
        new InstantCommand(m_controlPanel::raiseSpinner, m_controlPanel),
        new ParallelRaceGroup(
          new RunCommand(
            ()->{
              // Set the output at 0.7 times the direction to turn.
              m_controlPanel.setSpinMotor(0.7 * m_controlPanel.getDirectionToTurn(m_controlPanel.getColorToSearchFor(getGameSpecificMessage()), m_controlPanel.readColor()));
            },
            m_controlPanel),
          new WaitUntilCommand(()->{ return m_controlPanel.getDirectionToTurn(m_controlPanel.getColorToSearchFor(getGameSpecificMessage()), m_controlPanel.readColor()) == 0; })
        )
      )
    );

    // Right bumper is spin the control panel 3 times
    m_secondaryController_RightBumper.whenPressed( new SpinThreeTimesCommand(m_controlPanel) ); // Doesn't cancel command on let up
    
  }

  // I'm using StartEnd commands because by default they do not have isFinished return true, unlike InsantCommands. Alternative is to use the perpetually() decorator.
  private void configureDefaultCommands() {
    m_swerveDrive.setDefaultCommand(new DefaultSwerveCommand(m_swerveDrive, m_primaryController));

    m_thrower.setDefaultCommand(new StartEndCommand(m_thrower::stopThrower, ()->{}, m_thrower)); // Spin down thrower on startup, do nothing on end.

    m_controlPanel.setDefaultCommand(new StartEndCommand( ()->{
        m_controlPanel.turnOffSolenoid();
        m_controlPanel.setSpinMotor(0);
      }, ()->{}, m_controlPanel)); // Turn off spinny motor and solenoid on startup, do nothing on end. Control groups will have to be responsible for lowering system.

    // No default command for hopper because it should be maintaining its position via internal PID.

    m_intake.setDefaultCommand(new StartEndCommand( ()-> {
        m_intake.turnOffSolenoid();
        m_intake.setMotor(0);
      }, ()->{}, m_intake));  // Turn off spinny motor and solenoid on startup, do nothing on end. Control groups will have to be responsible for raising system.

    m_climber.setDefaultCommand(new InstantCommand(m_climber::turnOffSolenoid, m_climber).perpetually()); // By default, have the solenoid off. Never ends via perpetually() decorator
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
    Command autoCommand = new VisionApproachTargetCommand(m_swerveDrive, rft_, 100, 5, 5);
    
    return autoCommand;
  }
}
