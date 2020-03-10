package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.DrivetrainConstants;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.*;


import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.SocketVisionWrapper;
import frc.robot.Constants.Vision;

// switching to Limelight 3/9
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Command to turn the robot (set the robot's pose) to the desired angle
 * Constructor takes robot's drivetrain and desired angle
 */
public class VisionAimCommand extends CommandBase {
    
    private static final double ANGLE_CHECK_TIME = 0.1; // seconds
    private static final double TARGET_ANGLE_BUFFER = 5.0; // degrees

    private final SwerveDriveSubsystem drivetrain;
    private final double targetAngle;
    private final PIDController angleController;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    // switching to Limelight 3/9
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry tx = table.getEntry("tx");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tv = table.getEntry("tv");


  SwerveDriveSubsystem m_drivetrain;
  SocketVisionWrapper m_vision;

  private double previousXCoord = 0;

    /**
     * turn the robot to aim at the target
     */
    public VisionAimCommand(SwerveDriveSubsystem drivetrain, SocketVisionWrapper vision) {
        this.drivetrain = drivetrain;

        this.targetAngle = 0;
        angleController = new PIDController(DrivetrainConstants.POSE_ANGLE_kP, DrivetrainConstants.POSE_ANGLE_kI, DrivetrainConstants.POSE_ANGLE_kD);
        angleController.enableContinuousInput(0, 360);
        angleController.reset();
        m_vision = vision;
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
        angleController.setSetpoint(targetAngle);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
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
    }

    @Override
    public void execute() {
        /*
        if(m_vision.get().get_direction() != "nada"){
            previousXCoord = m_vision.get().get_degrees_x() + Vision.RFT_X_OFFSET; // pixels converted to approximate degrees of field of view of camera
            //previousDistance = m_vision.get().get_distance();
          } else {
            // Assume robot continued to move at same rate
            previousXCoord = 0; //previousXCoord - m_drivetrain.getStrafeErrorDerivative();
            //previousDistance = previousDistance - m_drivetrain.getForwardErrorDerivative();
          } 
          */
          previousXCoord = tx.getDouble(0) + Vision.RFT_X_OFFSET_LL; // switching to Limelight 3/9
        double rotation = angleController.calculate( previousXCoord * Vision.RFT_PIXELS_TO_DEGREES);
        // rotation = Math.min( 0.5, Math.max( -0.5, rotation));  // clamp
        for (int i = 0; i < 4; i++)
            drivetrain.getSwerveModule(i).setTargetSpeed(rotation);
    }

    @Override
    public boolean isFinished() {
        // double currentAngle = (m_vision.get().get_degrees_x() + Vision.RFT_X_OFFSET) * Vision.RFT_PIXELS_TO_DEGREES;
        double currentAngle = tx.getDouble(0) + Vision.RFT_X_OFFSET_LL; // 3/9 Limelight
        double currentError = currentAngle - targetAngle;

        boolean inTargetBuffer = Math.abs(currentError) < TARGET_ANGLE_BUFFER;

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
            System.out.println("VisionAim turning " + tx.getDouble(0));
        }
        // angleController.disable();
        drivetrain.holonomicDrive(0, 0, 0);
    }
}
