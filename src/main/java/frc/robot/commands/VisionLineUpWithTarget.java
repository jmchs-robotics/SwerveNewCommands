/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.SocketVisionWrapper;

public class VisionLineUpWithTarget extends CommandBase {
  SwerveDriveSubsystem m_drivetrain;
  SocketVisionWrapper m_vision;

  private double vision_error_x = 0;

  /**
   * Creates a new VisionLineUpWithTarget.
   */
  public VisionLineUpWithTarget(SwerveDriveSubsystem drivetrain, SocketVisionWrapper vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetPID();

    // Continuous input for rotation
    m_drivetrain.setRotationWraparoundInputRange(0, 360);
    m_drivetrain.setRotationSetpoint(m_drivetrain.getGyroAngle());
    m_drivetrain.setRotationTolerance(18, 18); // +/- 5% of setpoint is OK for pos and vel.
    m_drivetrain.setRotationOutputRange(-1, 1);
    
    // Set up forward and strafe controls:
    m_drivetrain.setStrafeTolerance(16, 16); // +/- 5% of setpoint is OK for pos and vel.
    m_drivetrain.setStrafeOutputRange(-1, 1);

    // leave Forward PID controller alone
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check vision for valid results
    String visionValid = m_vision.get().get_direction();
    if(visionValid != "nada"){
      vision_error_x = m_vision.get().get_degrees_x();
    } else {
      // Assume robot keeps moving to the side
      vision_error_x = vision_error_x - m_drivetrain.getStrafeErrorDerivative();
    }

    m_drivetrain.pidMove(0, vision_error_x, m_drivetrain.getGyroAngle(), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDriveMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: make finish timer
    return m_drivetrain.forwardAtSetpoint() && m_drivetrain.strafeAtSetpoint() && m_drivetrain.rotationAtSetpoint();
  }
}
