/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.commands.SampleColorCommand;
import frc.robot.commands.SendVisionCommand;
import frc.robot.commands.VisionLineUpWithTarget;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.SocketVisionSendWrapper;
import frc.robot.util.SocketVisionWrapper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final SwerveDriveSubsystem m_swerve = new SwerveDriveSubsystem();

  // Vision objects
  private final SocketVisionWrapper rft_ = new SocketVisionWrapper("10.59.33.255", 5801);
  private final SocketVisionWrapper piece_ = new SocketVisionWrapper("10.59.33.255", 5805);
  private final SocketVisionSendWrapper sender_ = new SocketVisionSendWrapper("10.59.33.255", 5800);

  // Color Sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  // Define Joysticks and Buttons here
  private final XboxController m_primaryController = new XboxController(0);
  private final XboxController m_secondaryController = new XboxController(1);

  private final JoystickButton m_primaryController_A = new JoystickButton(m_primaryController,
      XboxController.Button.kA.value);
  private final JoystickButton m_primaryController_B = new JoystickButton(m_primaryController,
      XboxController.Button.kB.value);
  private final JoystickButton m_primaryController_LeftBumper = new JoystickButton(m_primaryController, 
      XboxController.Button.kBumperLeft.value);

  private final JoystickButton m_secondaryController_StickLeft = new JoystickButton(m_secondaryController,
      XboxController.Button.kStickLeft.value);
  
  // Define commands here (if necessary)
  // Many, or even most, commands can be declared inline.
  private final DefaultSwerveCommand m_autoCommand = new DefaultSwerveCommand(m_swerve, m_primaryController);

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
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SendVisionCommand(sender_, "R"), // Can't be a lambda because Sender's aren't subsystems
        new VisionLineUpWithTarget(m_swerve, rft_), 
        new SendVisionCommand(sender_, "_")
      )
    );

    m_primaryController_B.whenPressed( // Inline command group!
      new SequentialCommandGroup(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve), // Brake mode on!
        new SendVisionCommand(sender_, "G"), // Can't be a lambda because Sender's aren't subsystems
        new VisionLineUpWithTarget(m_swerve, piece_), 
        new SendVisionCommand(sender_, "_")
      )
    );

    // Put brake mode on a button!
    m_primaryController_LeftBumper.whenPressed(
        new InstantCommand(m_swerve::setBrakeOn, m_swerve)
      ).whenReleased(
        new InstantCommand(m_swerve::setBrakeOff, m_swerve)
      );

    m_secondaryController_StickLeft.whileHeld(
      new SampleColorCommand(m_colorSensor)
    );
  }

  private void configureDefaultCommands() {
    m_swerve.setDefaultCommand(new DefaultSwerveCommand(m_swerve, m_primaryController));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
