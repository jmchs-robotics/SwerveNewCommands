package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.Constants.Vision;

/**
 * Command to turn the robot (set the robot's pose) to the desired angle
 * Constructor takes robot's drivetrain and desired angle
 */
public class VisionAimGyroCommand extends CommandBase {
    private static final double ANGLE_CHECK_TIME = 0.1;
    private static final double TARGET_ANGLE_BUFFER = 1.5;

    private final SwerveDriveSubsystem drivetrain;
    private double targetAngle = 0;
    private final PIDController angleController;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;
    SocketVisionWrapper m_vision;

    /**

     * turn the robot to angle reported by the vision coprocessor (in intialize()), 
     *   using the gyro while turninng (in execute())
     * @param drivetrain (SwerveDriveSubsystem)
     * @param targetAngle angle to turn to, in degrees (double) Positive is CCW, negative is CW.
     */
    public VisionAimGyroCommand(SwerveDriveSubsystem drivetrain, SocketVisionWrapper vision) {
        this.drivetrain = drivetrain;
        m_vision = vision;
        angleController = new PIDController(DrivetrainConstants.POSE_ANGLE_kP, DrivetrainConstants.POSE_ANGLE_kI, DrivetrainConstants.POSE_ANGLE_kD);
        angleController.enableContinuousInput(0, 360);
        angleController.reset();
        
        double x = 0;
        SmartDashboard.putNumber("x in VisionAimGyro", x);
        targetAngle = 0;
        
        SmartDashboard.putNumber("Target angle in VisionAimGyro", targetAngle);

        /*
        angleController = new PIDController(0.03, 0, 0.075, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return drivetrain.getGyroAngle();
            }
        }, output -> {
            for (int i = 0; i < 4; i++)
                drivetrain.getSwerveModule(i).setTargetSpeed(output);
        });
        if (Robot.PRACTICE_BOT)
            angleController.setP(0.025);
        
        angleController.setInputRange(0, 360);
        angleController.setOutputRange(-0.5, 0.5);
        angleController.setContinuous(true);
*/
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double x;
        // set target pose angle based on current pose angle and angle of vision target
        if(m_vision.get().get_direction() != "nada"){
            x = m_vision.get().get_degrees_x() + Vision.RFT_X_OFFSET; // target position in field of view of camera, in pixels from center
          } else {
              if( Vision.TUNE) {
                  System.out.println( "VisionAimGyroCommand: read NADA from vision socket.");
              }
            x = 0; 
          }
        SmartDashboard.putNumber("x in VisionAimGyro", x * Vision.RFT_PIXELS_TO_DEGREES);
        targetAngle = drivetrain.getGyroAngle() - x * Vision.RFT_PIXELS_TO_DEGREES;
        targetAngle %= 360;
        if( targetAngle < 0) {
            targetAngle += 360;
        }
        SmartDashboard.putNumber("Target angle in VisionAimGyro", targetAngle);

        angleController.setSetpoint(targetAngle);

        finishTimer.stop();
        finishTimer.reset();
        isTimerStarted = false;

        double a = -(DrivetrainConstants.WHEELBASE / DrivetrainConstants.TRACKWIDTH);
        double b = (DrivetrainConstants.WHEELBASE / DrivetrainConstants.TRACKWIDTH);
        double c = -(DrivetrainConstants.TRACKWIDTH / DrivetrainConstants.WHEELBASE);
        double d = (DrivetrainConstants.TRACKWIDTH / DrivetrainConstants.WHEELBASE);

        double[] angles = new double[]{
                Math.atan2(b, c),
                Math.atan2(b, d),
                Math.atan2(a, d),
                Math.atan2(a, c)
        };

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(Math.toDegrees(angles[i]));
        }

        //angleController.enable();
        if( DrivetrainConstants.TUNE) {
            System.out.printf("VisionAimGyroCommand Turning to %.3f%n", targetAngle);
        }
    }

    @Override
    public void execute() {
        double output = angleController.calculate( drivetrain.getGyroAngle());
        for (int i = 0; i < 4; i++)
            drivetrain.getSwerveModule(i).setTargetSpeed(output);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = drivetrain.getGyroAngle();
        double currentError = currentAngle - targetAngle;
        System.out.println("current Error for Vision Aim: " + currentError);
        System.out.println("current Angle for Vision Aim: " + currentAngle);
        boolean inTargetBuffer = Math.abs(currentError) < TARGET_ANGLE_BUFFER
                | 360 - Math.abs(currentError) < TARGET_ANGLE_BUFFER;

        if (inTargetBuffer) {
            if (!isTimerStarted) {
                finishTimer.start();
                isTimerStarted = true;
            }
        } else {
            finishTimer.stop();
            finishTimer.reset();
            isTimerStarted = false;
        }

        return finishTimer.hasPeriodPassed(ANGLE_CHECK_TIME);
    }

    @Override
    public void end( boolean isInterrupted) {
        if( DrivetrainConstants.TUNE) {
            System.out.println("VisionAimGyroCommand: Done turning");
        }
        // angleController.disable();
        drivetrain.holonomicDrive(0, 0, 0);
    }
}
