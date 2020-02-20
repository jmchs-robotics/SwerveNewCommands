package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class SetFieldOrientedAngleCommand extends CommandBase {

    private final SwerveDriveSubsystem drivetrain;
    private final double angleWant;

    /**
     * change the field angle of the field orientation.  Does not move the robot.
     * e.g. set to +90 at the start of the match if the robot starts the match placed on the field turned 90 degrees CW (to its right)
     *    set to -90 at the start of the match if the robot starts the match placed on the field turned 90 degrees CCW (to its left)
     * @param drivetrain
     * @param a change the gyro's internal setting to this
     */
    public SetFieldOrientedAngleCommand(SwerveDriveSubsystem drivetrain, double a) {
        this.drivetrain = drivetrain;
        this.angleWant = a;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {   
        drivetrain.setAdjustmentAngle(angleWant);
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end( boolean interrupted) {

    }
}
