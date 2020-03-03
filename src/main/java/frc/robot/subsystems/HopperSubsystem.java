/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperPIDs;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

// import com.ctre.phoenix.ParamEnum;

public class HopperSubsystem extends SubsystemBase {
  private final TalonSRX m_hopperMotor;

  private double kP = HopperPIDs.kP;
  private double kI = HopperPIDs.kI;
  private double kD = HopperPIDs.kD;
  private double kF = HopperPIDs.kF;
  private double kMaxOutput = HopperPIDs.MAX_OUTPUT;
  private double kMinOutput = HopperPIDs.MIN_OUTPUT;
  private double m_setpoint = 0;

  private long m_desiredDaisyPosition = 0;
  private Boolean m_daisyMoving = false;
  private Timer daisyTimeMoving;
  private Timer daisyTimeInPosition;
  private Timer daisyJammedTimer;
  private boolean daisyJammed = false;

  // photdiode work
  private final AnalogInput m_lightSensor = new AnalogInput(0);
  private Boolean photoIsDark = false;
  private double photoSamples[];
  private int photoSamplesI = 0;

  /**
   * The index is to see the rotation position on the robot
   * This will count up throughout the match, so if you want only the 0-6 range, use (daisyIndex%6)
   */
  private int daisyIndex;
  private int lastIndex;
  private boolean justBumpedBack = false;
  private int ballCount = 0; // how many balls are currently loaded
  private int sdThrottlerCtr = 0;
  
  /**
   * Creates a new HopperSubsystem.
   */
  public HopperSubsystem() {
    kP = HopperPIDs.kP;
    kI = HopperPIDs.kI;
    kD = HopperPIDs.kD;
    kF = HopperPIDs.kF;
    
    m_hopperMotor = new TalonSRX(HopperConstants.HOPPER_MOTOR_ID);
  
    daisyIndex = 0;
    photoSamples = new double [ HopperConstants.PHOTO_NUM_SAMPLES];

    /* Factory Default all hardware to prevent unexpected behaviour */
    m_hopperMotor.configFactoryDefault();
    
		
    /* Config the sensor used for Primary PID and sensor direction */
    // CTRE's sample code uses _Relative, but as of 1/26 finding _Absolute works better for us.
    m_hopperMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, // _Relative _Absolute,
                                            HopperPIDs.kPIDLoopIdx,
                                            HopperPIDs.kTimeoutMs);

    /* Ensure sensor is positive when output is positive */
		m_hopperMotor.setSensorPhase(HopperPIDs.kSensorPhase);

		/**
		 * Set based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. 
		 */ 
		m_hopperMotor.setInverted(HopperPIDs.kMotorInvert);

		/* Config the peak and nominal outputs, 12V means full */
		m_hopperMotor.configNominalOutputForward(0, HopperPIDs.kTimeoutMs);
		m_hopperMotor.configNominalOutputReverse(0, HopperPIDs.kTimeoutMs);
		m_hopperMotor.configPeakOutputForward(kMaxOutput, HopperPIDs.kTimeoutMs);
		m_hopperMotor.configPeakOutputReverse(kMinOutput, HopperPIDs.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		m_hopperMotor.configAllowableClosedloopError(HopperPIDs.kPIDLoopIdx, HopperConstants.ALLOWABLE_ERROR, HopperPIDs.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		m_hopperMotor.config_kF(HopperPIDs.kPIDLoopIdx, kF, HopperPIDs.kTimeoutMs);
		m_hopperMotor.config_kP(HopperPIDs.kPIDLoopIdx, kP, HopperPIDs.kTimeoutMs);
		m_hopperMotor.config_kI(HopperPIDs.kPIDLoopIdx, kI, HopperPIDs.kTimeoutMs);
		m_hopperMotor.config_kD(HopperPIDs.kPIDLoopIdx, kD, HopperPIDs.kTimeoutMs);
    
    m_setpoint = m_hopperMotor.getSensorCollection().getPulseWidthPosition();
    
    resetReference();  
    selectNearestSlot( false);
    //m_hopperMotor.set(ControlMode.Position, HopperConstants.DAISY_OFFSET);
    
    if(HopperPIDs.TUNE){
      SmartDashboard.putNumber("Hopper setpoint", m_setpoint);
      SmartDashboard.putNumber("Hopper error between setpoint and encoder position", m_hopperMotor.getClosedLoopError());
      SmartDashboard.putNumber("Hopper P", kP);
      SmartDashboard.putNumber("Hopper I", kI);
      SmartDashboard.putNumber("Hopper D", kD);
        //      SmartDashboard.putNumber("Hopper I Zone", kIz);
      SmartDashboard.putNumber("Hopper Feed Forward", kF);
      SmartDashboard.putNumber("Hopper Max Output", kMaxOutput);
      SmartDashboard.putNumber("Hopper Min Output", kMinOutput);

      SmartDashboard.putNumber("Daisy Index", daisyIndex); // Puts the dasiy index on SmartDash
    }
  }

  /**
   * from CTRE's sample code https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/PositionClosedLoop/src/main/java/frc/robot/Robot.java.  
   * Not used as of 1/26/20.
   * Grab the 360 degree position of the MagEncoder's absolute
   * position, and intitally set the relative sensor to match.
   */
  public void resetReference(){
    int absolutePosition = m_hopperMotor.getSensorCollection().getPulseWidthPosition();
    // Mask out overflows, keep bottom 12 bits 
    absolutePosition &= 0xFFF;

		if (HopperPIDs.kSensorPhase) { absolutePosition *= -1; }
		if (HopperPIDs.kMotorInvert) { absolutePosition *= -1; }

    m_setpoint = absolutePosition; // m_hopperMotor.getSelectedSensorPosition(HopperPIDs.kPIDLoopIdx); // getSensorCollection().getPulseWidthPosition();//getPosition();

    // Set the quadrature (relative) sensor to match absolute
		m_hopperMotor.setSelectedSensorPosition(absolutePosition, HopperPIDs.kPIDLoopIdx, HopperPIDs.kTimeoutMs);
  }

  /**
   * Sets the daisy index to the last slot passed.  Called from teleopInit() and autoInit()
   * Assume we've just loaded the balls before the start of the match.  We'll want to hand-advance Daisy
   * so the balls won't jam at first move.  So assume that means aligning the petals with a position we mark on
   * the side of the daisy, which is about 25% ahead of as though the ball were just loaded by the intake.
   * Then rounding the position down makes sense, and we instantly advance one to reveal the (rest of) the next
   * slot.
   * @param moveToSlot move the Daisy to the next slot, if true.
   */
  public void selectNearestSlot(boolean moveToSlot){
    m_setpoint = m_hopperMotor.getSelectedSensorPosition();
    lastIndex = daisyIndex;
    // set the index to the prevous slot, from the current position 5% beyond the perfect slot location
    // assumes the position has already been normalized to the range [0,ONE_ROTATION)
    daisyIndex = (int) (((m_setpoint - HopperConstants.DAISY_OFFSET) * 1.05 / HopperConstants.ONE_ROTATION) * 360.0 / 60.0);
    if( HopperPIDs.TUNE) {
      SmartDashboard.putNumber("DAISY MOVES ONE SIXTH ROTATION, to index", daisyIndex);
    } 
    if(moveToSlot){
      lastIndex = daisyIndex;
      daisyIndex++;
      m_setpoint = daisyIndex * HopperConstants.ONE_ROTATION * 60 / 360 + HopperConstants.DAISY_OFFSET;
      m_hopperMotor.set(ControlMode.Position,m_setpoint);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // keep track of photodiode
    photodiodeMovingWindow();

    // put PIDs on SmartDashboard and read them if the user changes them
    if(HopperPIDs.TUNE){
      SmartDashboard.putString("MOVING DAISY SLOWLY", "");
      double p = SmartDashboard.getNumber("Hopper P", 0);
      double i = SmartDashboard.getNumber("Hopper I", 0);
      double d = SmartDashboard.getNumber("Hopper D", 0);
      // double iz = SmartDashboard.getNumber("Hopper I Zone", 0);
      double ff = SmartDashboard.getNumber("Hopper Feed Forward", 0);
      double max = SmartDashboard.getNumber("Hopper Max Output", 0);
      double min = SmartDashboard.getNumber("Hopper Min Output", 0);
      SmartDashboard.putNumber("Hopper setpoint", m_setpoint);
      SmartDashboard.putNumber("Hopper error between setpoint and encoder position", m_hopperMotor.getClosedLoopError());
      SmartDashboard.putNumber("Hopper encoder position from getSelectedSensorPosition()", m_hopperMotor.getSelectedSensorPosition());

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { 
         kP = p; 	
         m_hopperMotor.config_kP(HopperPIDs.kPIDLoopIdx, kP, HopperPIDs.kTimeoutMs);
        }
      if((i != kI)) { 
        kI = i; 
        m_hopperMotor.config_kI(HopperPIDs.kPIDLoopIdx, kI, HopperPIDs.kTimeoutMs); 
      }
      if((d != kD)) {
        kD = d;
        m_hopperMotor.config_kD(HopperPIDs.kPIDLoopIdx, kD, HopperPIDs.kTimeoutMs);
       }
      //if((iz != kIz)) { m_hopperMotor.setIZone(iz); 
      //  kIz = iz; }
      if((ff != kF)) { 
        kF = ff; 
        m_hopperMotor.config_kF(HopperPIDs.kPIDLoopIdx, kF, HopperPIDs.kTimeoutMs);
      }
      if(max != kMaxOutput) {
        kMaxOutput = max;
        m_hopperMotor.configPeakOutputForward(kMaxOutput, HopperPIDs.kTimeoutMs);
      }
      if( min != kMinOutput){
        kMinOutput = min;
    		m_hopperMotor.configPeakOutputReverse(kMinOutput, HopperPIDs.kTimeoutMs);
      }
    }
    sdThrottlerCtr++;  // if we update the SmartDash every cycle it overruns and triggers the watchdog timer
    if( sdThrottlerCtr > 50)
    {
      SmartDashboard.putNumber("Daisy Index", daisyIndex);
      SmartDashboard.putNumber("Daisy Ball Count", ballCount);
      sdThrottlerCtr = HopperConstants.sdThrottleReset;
    }
    if( sdThrottlerCtr == 2 || sdThrottlerCtr == 14 || sdThrottlerCtr == 27 || sdThrottlerCtr == 41 ) {
      boolean b = ballLoaded();
      SmartDashboard.putBoolean("Ball received", b);
      SmartDashboard.putNumber("Photodiode Instant value", m_lightSensor.getVoltage());
    }
  }
  
  /**
   *  Move the daisy one full rotation, from current position
   */
  public void dischargeAll(){
    daisyIndex += 6;
    m_setpoint = daisyIndex * HopperConstants.ONE_ROTATION * 60 / 360 + HopperConstants.DAISY_OFFSET;
    if( HopperPIDs.TUNE) {
      System.out.println("HopperSubsystem Setting the hopper setpoint to " + m_setpoint);
    }
    m_hopperMotor.set(ControlMode.Position, m_setpoint); 
    resetBallCount();
  }

  /**
   * move the daisy back 1/24th of a rotation, from the current position
   * used in conjunction with (i.e. right before) nextSlot() to prevent jams
   */
  public void bumpBack() {
    justBumpedBack = true;
    m_setpoint = daisyIndex * HopperConstants.ONE_ROTATION * 60 / 360 + HopperConstants.DAISY_OFFSET - HopperConstants.ONE_ROTATION * 60 / 360 / 2;
    m_hopperMotor.set(ControlMode.Position, m_setpoint);
  }
  /**
   *  Move the daisy 1/6 rotation forward, from current position
   */
  public void nextSlot(){
    // only point to next index position if it's our very first move
    // or we were trying to advance and we've fully accomplished the advance
    // or if the last attempt was to go backwards
    // or we just bumped backwards in anticipation of the forwards move
    if(( lastIndex < daisyIndex && m_hopperMotor.getSelectedSensorPosition() > m_setpoint - .05*60/360*HopperConstants.ONE_ROTATION) 
      || (lastIndex >= daisyIndex)
      || justBumpedBack) {
    //if(atSetpoint(0.01)){
      lastIndex = daisyIndex;
      daisyIndex++;
      m_setpoint = daisyIndex * HopperConstants.ONE_ROTATION * 60 / 360 + HopperConstants.DAISY_OFFSET;
    }

    m_hopperMotor.set(ControlMode.Position, m_setpoint);
    if( HopperPIDs.TUNE) {
      SmartDashboard.putNumber("DAISY MOVES ONE SIXTH ROTATION, to index", daisyIndex);
    }       
    justBumpedBack = false;
  }

  /**
   *  Move the daisy 1/6 rotation backward, from current position
   */
  public void previousSlot() {
    // only point to previous index position if were last trying to backwards and we've fully accomplished the last reverse
    // or if the last attempt was to go forwards
    if(( lastIndex > daisyIndex && m_hopperMotor.getSelectedSensorPosition() < m_setpoint + .05*60/360*HopperConstants.ONE_ROTATION) 
      || lastIndex < daisyIndex) {
      lastIndex = daisyIndex;
    // if(atSetpoint(0.01)){
      daisyIndex--;
      m_setpoint = daisyIndex * HopperConstants.ONE_ROTATION * 60 / 360 + HopperConstants.DAISY_OFFSET;
    }

    m_hopperMotor.set(ControlMode.Position, m_setpoint);

    if( HopperPIDs.TUNE) {
      SmartDashboard.putNumber("DAISY MOVES ONE SIXTH ROTATION BACKWARD, to index", daisyIndex);
    }
    justBumpedBack = false;
  }

  public void moveForwardSlowly() {
    m_hopperMotor.set(ControlMode.PercentOutput, 0.2);
    if( HopperPIDs.TUNE) {
      SmartDashboard.putString("MOVING DAISY SLOWLY", "shooooooooosh!!");
    }
  }
  // stop
  public void stopMotor() {
    m_hopperMotor.set(ControlMode.PercentOutput, 0);
  }

  public boolean atSetpoint(double thresholdPercent) {
    SmartDashboard.putNumber("CLosed Loop Error", m_hopperMotor.getClosedLoopError());
    return Math.abs( m_hopperMotor.getClosedLoopError()) < HopperConstants.ONE_ROTATION * thresholdPercent;
    // return Math.abs(m_setpoint - m_hopperMotor.getSensorCollection().getPulseWidthPosition()) <= Math.abs(m_setpoint*thresholdPercent);
  }

  public void smartDashIndex()
  {
    SmartDashboard.putNumber("Daisy Index.IndexMod6", daisyIndex + (daisyIndex%6)/10.0);
  }

  /**
   * keep the most recent (HopperConstants.PHOTO_NUM_SAMPLES) photodiode samples
   */
  public void photodiodeMovingWindow() {
    photoSamples[ photoSamplesI] = m_lightSensor.getVoltage();
    photoSamplesI++;
    photoSamplesI = photoSamplesI % HopperConstants.PHOTO_NUM_SAMPLES;
  }
  /** 
   * compute the average of photodiode sensor voltages as kept in the moving average.
   * @return true if the average value of the photodiode over recent samples is less than the threshold (HopperConstants.DARK_THRESH)
  */
  public boolean photoDiodeAveIsDark() { 
    double s = 0;
    for( int i=0; i<HopperConstants.PHOTO_NUM_SAMPLES; i++) {
      s += photoSamples[ i];
    }
    
    if( s / HopperConstants.PHOTO_NUM_SAMPLES < HopperConstants.DARK_THRESH) { 
      photoIsDark = true; 
    }
    else { 
      photoIsDark = false;
    }

    return photoIsDark;
  }

  /**
   * is a ball loaded?
   * @return true if a ball is loaded
   */
  public boolean ballLoaded() {
    return photoDiodeAveIsDark();
  }

  /**
   * returns the position (index) of Daisy, in the range [0..5]
   */
  public int getIndexMod6() {
    return daisyIndex % 6;
  }

  /**
   * returns true if we have indexed 5 balls
   */
  public boolean daisyIsFull() {
    return ballCount >= 5;
  }

  /**
   * returns true if there are fewer than 5 balls in Daisy
   */
  public boolean okToAdvance() {
    return ballCount <= 4;
  }
    
  /**
   * set the ball count
   * @param bc the number to set it to
   */
  public void setBallCount( int bc) {
    ballCount = bc;
  }
  public void setBallCountTo3(){
    setBallCount(3);
  }

  public int incBallCount() {
    ballCount ++;
    return ballCount;
  }
  public int decBallCount() {
    ballCount--;
    return ballCount;
  }
  public void resetBallCount() {
    ballCount = 0;
  }
}
