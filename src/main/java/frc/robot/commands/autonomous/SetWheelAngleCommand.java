package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class SetWheelAngleCommand extends CommandBase {

    private final SwerveDriveSubsystem drivetrain;
    private final double angleWant;

    public SetWheelAngleCommand(SwerveDriveSubsystem drivetrain, double a) {
        this.drivetrain = drivetrain;
        this.angleWant = a;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {   
        for (SwerveDriveModule module : drivetrain.getSwerveModules()) {
            module.setTargetAngle(angleWant);
        }
    }

    @Override
    public boolean isFinished() {

        return true;
    }

    @Override
    public void end( boolean interrupted) {

    }
}
