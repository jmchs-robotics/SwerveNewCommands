package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DrivetrainConstants;

// import the drivetrain constants statically so they can simply be used
// as variables (see swervedrivemodules below)
import static frc.robot.Constants.DrivetrainMotors.*;

public class SwerveDriveSubsystem extends HolonomicDrivetrain {
    // Macro PIDControllers for new synchronous structure
    // 1/5/2020
    
    PIDController strafeController;
    PIDController forwardController;
    PIDController rotationController;

    private double forwardMinOutput = -1;
    private double strafeMinOutput = -1;
    private double rotationMinOutput = -1;
    private double forwardMaxOutput = 1;
    private double strafeMaxOutput = 1;
    private double rotationMaxOutput = 1;

	/*
	 * 0 is Front Left
	 * 1 is Front Right
	 * 2 is Back Right
	 * 3 is Back Left
	 */
    private SwerveDriveModule[] mSwerveModules;

    private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

    public SwerveDriveSubsystem() {
        super(DrivetrainConstants.WIDTH, DrivetrainConstants.LENGTH);
        zeroGyro();
        
        // 10/26/19 Big Switch from Talon to Spark Max controllers
        // 11/26/19 less positive angle offset settings turns wheel angle clockwise looking from the top
        //  more positive angle offset turns wheel counterclockwise looking from top
        mSwerveModules = new SwerveDriveModule[] {
            new SwerveDriveModule(0, 
                new CANSparkMax(FRONT_LEFT_ANGLE, MotorType.kBrushless),
                new CANSparkMax(FRONT_LEFT_DRIVE, MotorType.kBrushless),
                221.25 + 7) , // 81.25- 65+7 + 180 + 18) //offset need to be between 0 and 360
            new SwerveDriveModule(1, 
                new CANSparkMax(FRONT_RIGHT_ANGLE, MotorType.kBrushless),
                new CANSparkMax(FRONT_RIGHT_DRIVE, MotorType.kBrushless),
                296 - 200),//68.421 + 10 + 17), // offset needs to be between 0 and 360
            // 10/26/19 need to change the other 2 modules to SparkMax
            new SwerveDriveModule(2,
                new CANSparkMax(BACK_RIGHT_ANGLE, MotorType.kBrushless),
                new CANSparkMax(BACK_RIGHT_DRIVE, MotorType.kBrushless),
                290), // 300-8-2) // offset needs to be between 0 and 360
            // 11/26/19 less positive angle offset settings turns wheel angle clockwise looking from the top
            new SwerveDriveModule(3,
                new CANSparkMax(BACK_LEFT_ANGLE, MotorType.kBrushless),
                new CANSparkMax(BACK_LEFT_DRIVE, MotorType.kBrushless),
                 153), //offset needs to be between 0 and 360
            // 11/26/19 less positive angle offset settings turns wheel angle clockwise looking from the top   
        };

        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetAngle(0);
            module.setDriveGearRatio(5.7777);
            module.setDriveWheelRadius(module.getDriveWheelRadius() * 1.05);
            module.setMotionConstraints(getMaxAcceleration(), getMaxVelocity());
            module.setAngleKD(DrivetrainConstants.ANGLE_kD);
            module.setAngleKI(DrivetrainConstants.ANGLE_kI);
            module.setAngleKP(DrivetrainConstants.ANGLE_kP);
            module.zeroDistance();  // 200223 set drive encoder to zero
        }

        strafeController = new PIDController(DrivetrainConstants.STRAFE_kP, DrivetrainConstants.STRAFE_kI, DrivetrainConstants.STRAFE_kD);
        forwardController = new PIDController(DrivetrainConstants.FORWARD_kP, DrivetrainConstants.FORWARD_kI, DrivetrainConstants.FORWARD_kD);
        rotationController = new PIDController(DrivetrainConstants.ROTATION_kP, DrivetrainConstants.ROTATION_kI, DrivetrainConstants.ROTATION_kD);
    }

    public void periodic() {
      if( DrivetrainConstants.TUNE) {
        SmartDashboard.putNumber( "Module 0 Max PID output", mSwerveModules[0].getDriveMotor().getPIDController().getOutputMax());
        SmartDashboard.putNumber("Gyro Degrees", getGyroAngle());
      }
    }
    /**
     * compute the angles of the four modules and return a vector of them
     * uses private member isFieldOriented to decide to adjust based on gyro reading
     * @param forward how far forward the robot is going
     * @param strafe how far to the side the robot is going
     * @param rotation how much to spin the robot 
     * @param fo true if the computation is to be based on the field (field oriented)
     * @return vector of angles of the set points for the four modules, in order: LF, RF, RB, LB
     */
    public double[] calculateSwerveModuleAngles(double forward, double strafe, double rotation) {
        if (isFieldOriented()) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }

        double a = strafe - rotation * (DrivetrainConstants.WHEELBASE / DrivetrainConstants.TRACKWIDTH);
        double b = strafe + rotation * (DrivetrainConstants.WHEELBASE / DrivetrainConstants.TRACKWIDTH);
        double c = forward - rotation * (DrivetrainConstants.TRACKWIDTH / DrivetrainConstants.WHEELBASE);
        double d = forward + rotation * (DrivetrainConstants.TRACKWIDTH / DrivetrainConstants.WHEELBASE);

        return new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };
    }

    public AHRS getNavX() {
        return mNavX;
    }

    public double getGyroAngle() {
        double angle = mNavX.getAngle() - getAdjustmentAngle();
        angle %= 360;
        if (angle < 0) angle += 360;
        return 360 - angle;
    }

    public double getGyroRate() {
        return mNavX.getRate();
    }

    public double getRawGyroAngle() {
        double angle = mNavX.getAngle();
        angle %= 360;
        if (angle < 0) angle += 360;

        return angle;
    }

    public SwerveDriveModule getSwerveModule(int i) {
        return mSwerveModules[i];
    }

    @Override
    public void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented) {
        forward *= getSpeedMultiplier();
        strafe *= getSpeedMultiplier();
 
        if (fieldOriented) {
            double angleRad = Math.toRadians(getGyroAngle());
            double temp = forward * Math.cos(angleRad) +
                    strafe * Math.sin(angleRad);
            strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
            forward = temp;
        }
        
        double a = strafe - rotation * (DrivetrainConstants.WHEELBASE / DrivetrainConstants.TRACKWIDTH);
        double b = strafe + rotation * (DrivetrainConstants.WHEELBASE / DrivetrainConstants.TRACKWIDTH);
        double c = forward - rotation * (DrivetrainConstants.TRACKWIDTH / DrivetrainConstants.WHEELBASE);
        double d = forward + rotation * (DrivetrainConstants.TRACKWIDTH / DrivetrainConstants.WHEELBASE);

        double[] angles = new double[]{
                Math.atan2(b, c) * 180 / Math.PI,
                Math.atan2(b, d) * 180 / Math.PI,
                Math.atan2(a, d) * 180 / Math.PI,
                Math.atan2(a, c) * 180 / Math.PI
        };
        

        double[] speeds = new double[]{
                Math.sqrt(b * b + c * c),
                Math.sqrt(b * b + d * d),
                Math.sqrt(a * a + d * d),
                Math.sqrt(a * a + c * c)
        };

        double max = speeds[0];

        for (double speed : speeds) {
            if (speed > max) {
                max = speed;
            }
        }

        for (int i = 0; i < 4; i++) {
            if (Math.abs(forward) > 0.05 ||
                    Math.abs(strafe) > 0.05 ||
                    Math.abs(rotation) > 0.05) {
                mSwerveModules[i].setTargetAngle(angles[i] + 180);
            } else {
                mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
            }
            mSwerveModules[i].setTargetSpeed(speeds[i]);
        }
    }

    /**
     * As of 2/17/20, don't use pidMove.  It doesn't yet work.
     * @param forwardError
     * @param strafeError
     * @param angleError
     * @param fieldOriented
     */
    public void pidMove(double forwardError, double strafeError, double angleError, boolean fieldOriented){
      double forward = forwardController.calculate(forwardError);
      double strafe = strafeController.calculate(strafeError);
      double rotation = rotationController.calculate(angleError);

      forward = MathUtil.clamp(forward, forwardMinOutput, forwardMaxOutput);
      strafe = MathUtil.clamp(strafe, strafeMinOutput, strafeMaxOutput);
      rotation = MathUtil.clamp(rotation, rotationMinOutput, rotationMaxOutput);

      holonomicDrive(forward, strafe, rotation, fieldOriented);
    }
    
    @Override
    public void stopDriveMotors() {
        for (SwerveDriveModule module : mSwerveModules) {
            module.setTargetSpeed(0);
        }
    }
    
    public void resetMotors() {
    	for(int i = 0; i < mSwerveModules.length; i++) {
    		mSwerveModules[i].resetMotor();
    	}
    }

    public SwerveDriveModule[] getSwerveModules() {
        return mSwerveModules;
    }

    @Override
    public double getMaxAcceleration() {
      return 5.5;
    }

    @Override
    public double getMaxVelocity() {
      return 10; 
    }

    /**
     * Setting all the modules to be brake or coast
     */
    public void setBrake(boolean b)
    {
      for (int i = 0; i < 4; i++)
      {
        mSwerveModules[i].setMotorBrake(b);
      }
    }

    /**
     * Sets the brake mode off. Same as calling {@code setBrake(false)}.
     */
    public void setBrakeOff(){
      setBrake(false);
    }
    /**
     * Sets the brake mode on. Same as calling {@code setBrake(true)}.
     */
    public void setBrakeOn(){
      setBrake(true);
    }

    /**
     * set the min, max allowable output for all drive PIDs
     * @param min
     * @param max
     */
    public void setDrivePIDOutputRange( double min, double max) {
      for( int i=0; i<4; i++) {
        mSwerveModules[i].setDrivePIDOutputRange(min, max);
      }
    }

    /**
     * So can set the PID output range from an InstantCommand
     */
    public void setDrivePIDToSlow() {
      double x = 0.3;
      setDrivePIDOutputRange(-1 * x, x);
    }

    public void setDrivePIDToFast() {
      double x = 1;
      setDrivePIDOutputRange(-1 * x, x);
    }
    /*
     # from Eric 1/2020
     * ALL the PID methods. Like, all of them. There are a lot.
     */
    public void setForwardSetpoint(double setpoint){
      forwardController.setSetpoint(setpoint);
    }
    public void setStrafeSetpoint(double setpoint){
      strafeController.setSetpoint(setpoint);
    }
    public void setRotationSetpoint(double setpoint){
      rotationController.setSetpoint(setpoint);
    }
    public void setForwardTolerance(double positionTolerance, double velocityTolerance){
      forwardController.setTolerance(positionTolerance, velocityTolerance);
    }
    public void setStrafeTolerance(double positionTolerance, double velocityTolerance){
      strafeController.setTolerance(positionTolerance, velocityTolerance);
    }
    public void setRotationTolerance(double positionTolerance, double velocityTolerance){
      rotationController.setTolerance(positionTolerance, velocityTolerance);
    }
    public void setForwardWraparoundInputRange(double min, double max){
      forwardController.enableContinuousInput(min, max);
    }
    public void setStrafeWraparoundInputRange(double min, double max){
      strafeController.enableContinuousInput(min, max);
    }
    public void setRotationWraparoundInputRange(double min, double max){
      rotationController.enableContinuousInput(min, max);
    }
    public void setForwardAccumulationRange(double min, double max){
      forwardController.setIntegratorRange(min, max);
    }
    public void setStrafeAccumulationRange(double min, double max){
      strafeController.setIntegratorRange(min, max);
    }
    public void setRotationAccumulationRange(double min, double max){
      rotationController.setIntegratorRange(min, max);
    } 
    public void setForwardOutputRange(double minOutput, double maxOutput){
      forwardMinOutput = minOutput;
      forwardMaxOutput = maxOutput;
    }
    public void setStrafeOutputRange(double minOutput, double maxOutput){
      strafeMinOutput = minOutput;
      strafeMaxOutput = maxOutput;
    }
    public void setRotationOutputRange(double minOutput, double maxOutput) {
      rotationMinOutput = minOutput;
      rotationMaxOutput = maxOutput;
    }
    public double getForwardErrorDerivative(){
      return forwardController.getVelocityError();
    }
    public double getStrafeErrorDerivative(){
      return forwardController.getVelocityError();
    }
    public double getRotationErrorDerivative(){
      return rotationController.getVelocityError();
    }
    public boolean forwardAtSetpoint(){
      return forwardController.atSetpoint();
    }
    public boolean strafeAtSetpoint(){
      return strafeController.atSetpoint();
    }
    public boolean rotationAtSetpoint(){
      return strafeController.atSetpoint();
    }
    /**
     * Clear all I accumulation, disable continuous input, and set all 
     * setpoints to 0.
     */
    public void resetPID(){
      // clear I accumulation
      forwardController.reset();
      strafeController.reset();
      rotationController.reset();

      // reset to noncontinuous input
      forwardController.disableContinuousInput();
      strafeController.disableContinuousInput();
      rotationController.disableContinuousInput();

      // set all setpoints to 0
      forwardController.setSetpoint(0);
      strafeController.setSetpoint(0);
      rotationController.setSetpoint(0);

      // set all I accumulation ranges to defaults
      forwardController.setIntegratorRange(-1.0, 1.0);
      strafeController.setIntegratorRange(-1.0,1.0);
      rotationController.setIntegratorRange(-1.0, 1.0);

      forwardController.setTolerance(0.05, Double.POSITIVE_INFINITY);
      strafeController.setTolerance(0.05, Double.POSITIVE_INFINITY);
      rotationController.setTolerance(0.05, Double.POSITIVE_INFINITY);

      forwardMinOutput = -1;
      strafeMinOutput = -1;
      rotationMinOutput = -1;
      forwardMaxOutput = 1;
      strafeMaxOutput = 1;
      rotationMaxOutput = 1;
    }

    public void holonomicDriveToZero() {
      holonomicDrive(0,0,0);
    }
}
