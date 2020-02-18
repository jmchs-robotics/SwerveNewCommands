package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.Constants.AUTO;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class DriveForDistanceCommand extends CommandBase {

    private final SwerveDriveSubsystem m_drivetrain;
    private double fwd;
    private double strafe;
    private final double angle;
    private final double distance;
    private final double distRight, distForward;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    private double [] encoderStart = {0,0,0,0};


    /**
     * As of 2/17/20, don't use this.  Use DriveForDist2910Command instead!
     * drive this many inches forward and to the right
     * @param fwd inches forward, or negative inches to go backward
     * @param strafe inches to the right, or negative inches to go to the left
     */
    public DriveForDistanceCommand(SwerveDriveSubsystem drivetrain, double fwd, double strafe) {
        this.m_drivetrain = drivetrain;
        this.distForward = fwd;
        this.distRight = strafe;
        this.angle = Math.toDegrees(Math.atan2(distRight, distForward));
        this.distance = Math.sqrt(distRight * distRight + distForward * distForward);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
        m_drivetrain.resetPID();
        // Rotation PID (has continuous input)
        m_drivetrain.setRotationWraparoundInputRange(0, 360);
        m_drivetrain.setRotationSetpoint(m_drivetrain.getGyroAngle());
        m_drivetrain.setRotationTolerance(18, 18); // +/- 5% of setpoint is OK for pos and vel.
        m_drivetrain.setRotationOutputRange(-1, 1);
        
        // Set up strafe pid:
        m_drivetrain.setStrafeTolerance(6, 16); // +/- 5% of setpoint is OK for pos and vel.
        m_drivetrain.setStrafeOutputRange(-1, 1);
        m_drivetrain.setStrafeSetpoint( distRight);

        // Set up forward pid:
        m_drivetrain.setForwardSetpoint( distForward);
        m_drivetrain.setForwardTolerance(6, 15);
        m_drivetrain.setForwardOutputRange(-1, 1);

        finishTimer.stop();
        finishTimer.reset();
        isTimerStarted = false;

        //initialDrivetrainAngle = m_drivetrain.getGyroAngle();
        for( int i=0; i<4; i++) {
            encoderStart[i] = m_drivetrain.getSwerveModule(i).getDriveDistance();
        }

    }

    @Override
    public void execute() {
      
      double forwardFactor = distForward / distance;
      double strafeFactor = distRight / distance;
      double encError = 0;
      for( int i=0; i<4; i++) {
          encError += encoderStart[i] - m_drivetrain.getSwerveModule(i).getDriveDistance();
      }
      encError /= 4;
      double fwdErr = encError  * forwardFactor;
      double strafeErr = encError * strafeFactor;
  
      m_drivetrain.pidMove(fwdErr, strafeErr, m_drivetrain.getGyroAngle(), true);
    }

    @Override
    public boolean isFinished() {

        boolean inBuffer = m_drivetrain.forwardAtSetpoint() && m_drivetrain.strafeAtSetpoint() && m_drivetrain.rotationAtSetpoint();
        
        if (inBuffer) {
            if (!isTimerStarted) {
                finishTimer.start();
                isTimerStarted = true;
            }
        } else {
            finishTimer.stop();
            finishTimer.reset();
            isTimerStarted = false;
        }

        return finishTimer.hasPeriodPassed(AUTO.DISTANCE_CHECK_TIME);

    }

    @Override
    public void end( boolean isInterrupted) {
        m_drivetrain.stopDriveMotors();
    }
}
