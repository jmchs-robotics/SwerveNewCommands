/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  private int m_sdThrottleCtr = 0;

  private final SendableChooser<String> startPosChooser = new SendableChooser<>();	
	

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    startPosChooser.setDefaultOption("Path1 back 12, left 57", "1");
    startPosChooser.addOption("Path2 fwd 12, left 57", "2");
		
		// 'print' the Chooser to the dashboard
		SmartDashboard.putData("Path Chosen", startPosChooser);


    m_robotContainer = new RobotContainer();
    m_robotContainer.visionInit();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    m_sdThrottleCtr ++;
    if( m_sdThrottleCtr % 10 == 0) {
      String a = "NADA";
      try {
        a = m_robotContainer.getRftSocketReader().get().get_direction();
        SmartDashboard.putNumber("RFT Degrees", m_robotContainer.getRftSocketReader().get().get_degrees_x());
        // Pull down the match specific string and put it on the Dashboard
        SmartDashboard.putString("Match String", m_robotContainer.getGameSpecificMessage());
      } catch( Exception e ) {
      }
      a = a + " " + a + " " + a;
      SmartDashboard.putString("RFT Vision", a);
    }
    m_sdThrottleCtr = m_sdThrottleCtr % 50;
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    // Kill vision
   // m_robotContainer.visionShutDown();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Initialize vision
    m_robotContainer.visionInit();
    // reset the Daisy encoder and index, to match its current position
    m_robotContainer.resetHopperReference( false);

    String startPos = startPosChooser.getSelected();
		
    m_autonomousCommand = m_robotContainer.getAutonomousCommand( startPos);

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Initialize vision
    m_robotContainer.visionInit();
    // reset the Daisy encoder and index, to match its current position
    // m_robotContainer.resetHopperReference( false);
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
