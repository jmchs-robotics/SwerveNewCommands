package frc.robot.commands.autonomous;

import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;


public class DriveForTime2910Command extends CommandBase {
    
    private final Timer timer = new Timer();
    private final SwerveDriveSubsystem drivetrain;
    private final double time;
    private final double forward;
    private final double strafe;

    public DriveForTime2910Command(SwerveDriveSubsystem drivetrain, double time, double forward, double strafe) {
        this.drivetrain = drivetrain;
        this.time = time;
        this.forward = forward;
        this.strafe = strafe;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        drivetrain.holonomicDrive(forward, strafe, 0);
    }
    @Override
    public void execute() {
        drivetrain.holonomicDrive(forward, strafe, 0);
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasPeriodPassed(time);
    }

    @Override
    public void end( boolean isInterrupted) {
        timer.stop();
        timer.reset();
        drivetrain.holonomicDrive(0, 0, 0);
    }

}