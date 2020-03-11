package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.Constants.AUTO;

/** 
 * Point the robot toward a given pose angle.  Don't drive anywhere.
 * Is field oriented.
 */
public class SetPoseAngleCommand extends CommandBase {

    private final SwerveDriveSubsystem m_drivetrain;
    private final double angle;
    private final double distRight, distForward;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    /**
     * As of 2/17/20, don't use this.  Use SetPoseAngle2910Command instead!
     * Point the robot toward a given pose angle.
     * Is field oriented
     * @param angle angle to point the robot at
     */
    public SetPoseAngleCommand(SwerveDriveSubsystem drivetrain, double angle) {
        this.m_drivetrain = drivetrain;
        this.angle = (angle + 360) % 360; // normalize to range [0,360)
        this.distRight = 0; // go nowhere
        this.distForward = 0; // go nowhere

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        
        m_drivetrain.resetPID();
        // Rotation PID (has continuous input)
        m_drivetrain.setRotationWraparoundInputRange(0, 360);
        m_drivetrain.setRotationSetpoint(angle);
        m_drivetrain.setRotationTolerance(9, 18); // +/- 2.5%, 5% of setpoint is OK for pos and vel.
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
    }

    @Override
    public void execute() {
      m_drivetrain.pidMove(0, 0, m_drivetrain.getGyroAngle(), true);
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
