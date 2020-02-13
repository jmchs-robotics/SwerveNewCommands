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
import frc.robot.Constants.Vision;

public class VisionAim extends CommandBase {
  SwerveDriveSubsystem m_drivetrain;
  SocketVisionWrapper m_vision;

  private double m_proximity, m_tolerance, m_velTolerance;
  private double m_aimTolerance, m_aimVelTolerance;
  private double previousDistance = 0;
  private double previousXCoord = 0;

  /**
   * Creates a new VisionApproachTarget.
   * It is recommended that proximityTolerance and fwdVelTolerance are both 5% of the distance detector's range.
   * @param drivetrain The {@link SwerveDriveSubsystem} to drive
   * @param vision The {@link SocketVisionWrapper} object tracking the appropriate target
   * @param targetProximity The acceptable distance from the target.
   * @param proximityTolerance The tolerance (i.e. half the acceptable range around the targetProximity) for which the PID controller will consider the targetProximity to be reached.
   * @param fwdVelTolerance The largest magnitude of velocity that is allowable for the controller to consider the target distance reached.
   */
  public VisionAim(SwerveDriveSubsystem drivetrain, SocketVisionWrapper vision, double aimTolerance, double aimVelTolerance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_vision = vision;
    m_proximity = 100; // targetProximity;
    m_tolerance = 5; // proximityTolerance;
    m_velTolerance = 5; // = fwdVelTolerance;
    m_aimTolerance = aimTolerance;
    m_aimVelTolerance = aimVelTolerance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetPID();

    // Rotation PID (has continuous input)
    m_drivetrain.setRotationWraparoundInputRange(0, 360); // Still need wraparound to translate negative values to positive degrees
    // m_drivetrain.setRotationSetpoint(m_drivetrain.getGyroAngle()); // Use 0 as the rotation setpoint so we can let the camera use the target as the center of the robot's coordinate system
    m_drivetrain.setRotationTolerance(m_aimTolerance, m_aimVelTolerance);
    m_drivetrain.setRotationOutputRange(-1, 1);
    
    // Set up strafe pid:
    m_drivetrain.setStrafeTolerance(16, 16); // +/- 5% of setpoint is OK for pos and vel.
    m_drivetrain.setStrafeOutputRange(-1, 1);

    // Set up forward pid:
    m_drivetrain.setForwardSetpoint(m_proximity); // Hold the robot at some distance from the target
    m_drivetrain.setForwardTolerance(m_tolerance, m_velTolerance);
    m_drivetrain.setForwardOutputRange(-1, 1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check vision error for valid result
    if(m_vision.get().get_direction() != "nada"){
      previousXCoord = m_vision.get().get_degrees_x() + Vision.RFT_X_OFFSET; // pixels converted to approximate degrees of field of view of camera
      previousDistance = m_vision.get().get_distance();
    } else {
      // Assume robot continued to move at same rate
      previousXCoord = previousXCoord - m_drivetrain.getStrafeErrorDerivative();
      previousDistance = previousDistance - m_drivetrain.getForwardErrorDerivative();
    }

    // Pass each error term into the appropriate parameter: forwardError <=> distance, strafeError <=> XCoord, angleError <=> gyroAngle OR visionAngle
    // Don't use fieldOriented so the robot operates radially, centered on the target. This is OK because both strafe & rotation targets are 0. This would break if
    // either target was nonzero and both relied on vision for the error term
    // Using gyro for rotation and vision for X (fieldOriented off) would be cartesian coordinates perpendicular (forward) and parallel (strafe) to target orientation.
    // Using fieldOriented would be cartesian in forward & strafe; useful for moving some set distance forward and tracking a target in strafe & rotation (i.e. picking up a ball)
    // We may want to provide different sets of PID constants for encoder error inputs vs. image inputs for strafe & forward, and vision vs. gyro for rotation.
    m_drivetrain.pidMove(previousDistance, previousXCoord, previousXCoord * Vision.RFT_PIXELS_TO_DEGREES, false);
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
    return m_drivetrain.rotationAtSetpoint() && m_drivetrain.strafeAtSetpoint() && m_drivetrain.forwardAtSetpoint(); // Wait for all PIDs to be satisfied
  }
}
