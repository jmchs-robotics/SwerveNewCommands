/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.revrobotics.CANSparkMax;
//import com.revrobotics.ControlType;
//import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.HopperPIDs;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import frc.robot.Constants.ThrowerPIDs;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.revrobotics.CANEncoder;
//import com.revrobotics.CANPIDController;

import com.ctre.phoenix.ParamEnum;

public class HopperSubsystem extends SubsystemBase {
  private final TalonSRX m_hopperMotor;

  //private CANPIDController m_hopperController;
  //private CANEncoder m_hopperEncoder;

  
  private double kP = HopperPIDs.kP;
  private double kI = HopperPIDs.kI;
  
  private double kD = HopperPIDs.kD;
  private double kIz = HopperPIDs.kIz;
  private double kF = HopperPIDs.FEED_FORWARD;
  private double kMaxOutput = HopperPIDs.MAX_OUTPUT;
  private double kMinOutput = HopperPIDs.MIN_OUTPUT;
  private double m_setpoint = 0;
  
  
  private double m_reference = 0;
  /**
   * Creates a new HopperSubsystem.
   */
  public HopperSubsystem() {
    m_hopperMotor = new TalonSRX(HopperConstants.HOPPER_MOTOR_ID);

    /* Factory Default all hardware to prevent unexpected behaviour */
		m_hopperMotor.configFactoryDefault();
		
		/* Config the sensor used for Primary PID and sensor direction */
    m_hopperMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,
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
		m_hopperMotor.configPeakOutputForward(1, HopperPIDs.kTimeoutMs);
		m_hopperMotor.configPeakOutputReverse(-1, HopperPIDs.kTimeoutMs);

		/**
		 * Config the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		m_hopperMotor.configAllowableClosedloopError(0, HopperPIDs.kPIDLoopIdx, HopperPIDs.kTimeoutMs);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		m_hopperMotor.config_kF(HopperPIDs.kPIDLoopIdx, kF, HopperPIDs.kTimeoutMs);
		m_hopperMotor.config_kP(HopperPIDs.kPIDLoopIdx, kP, HopperPIDs.kTimeoutMs);
		m_hopperMotor.config_kI(HopperPIDs.kPIDLoopIdx, kI, HopperPIDs.kTimeoutMs);
		m_hopperMotor.config_kD(HopperPIDs.kPIDLoopIdx, kD, HopperPIDs.kTimeoutMs);

		/**
		 * Grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = m_hopperMotor.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (HopperPIDs.kSensorPhase) { absolutePosition *= -1; }
		if (HopperPIDs.kMotorInvert) { absolutePosition *= -1; }
		
		/* Set the quadrature (relative) sensor to match absolute */
		m_hopperMotor.setSelectedSensorPosition(absolutePosition, HopperPIDs.kPIDLoopIdx, HopperPIDs.kTimeoutMs);

     SmartDashboard.putNumber("Hopper desired wheel RPM", 0);
     SmartDashboard.putNumber("Hopper P", 0);
     SmartDashboard.putNumber("Hopper I", 0);
     SmartDashboard.putNumber("Hopper D", 0);
     SmartDashboard.putNumber("Hopper I Zone", 0);
     SmartDashboard.putNumber("Hopper Feed Forward", 0);
     SmartDashboard.putNumber("Hopper Max Output", 0);
     SmartDashboard.putNumber("Hopper Min Output", 0);

    
  }

  public void resetReference(){
    m_reference = m_hopperMotor.getSensorCollection().getPulseWidthPosition();//getPosition();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hopperMotor.set(ControlMode.Position, m_setpoint);
    if(HopperPIDs.TUNE){
      double speed = SmartDashboard.getNumber("Hopper desired wheel RPM", 0);
      double p = SmartDashboard.getNumber("Hopper P", 0);
      double i = SmartDashboard.getNumber("Hopper I", 0);
      double d = SmartDashboard.getNumber("Hopper D", 0);
      double iz = SmartDashboard.getNumber("Hopper I Zone", 0);
      double ff = SmartDashboard.getNumber("Hopper Feed Forward", 0);
      double max = SmartDashboard.getNumber("Hopper Max Output", 0);
      double min = SmartDashboard.getNumber("Hopper Min Output", 0);
    
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      // if(( speed != m_setpoint)) { setHopperSpeed( speed); }
      if((p != kP)) { // m_hopperMotor.setP(p);
         kP = p; 	
         m_hopperMotor.config_kP(HopperPIDs.kPIDLoopIdx, kP, HopperPIDs.kTimeoutMs);
        }
        
      if((i != kI)) { m_hopperMotor.config_kI(HopperPIDs.kPIDLoopIdx, kI, HopperPIDs.kTimeoutMs); 
        kI = i; 
      }
      if((d != kD)) {m_hopperMotor.config_kD(HopperPIDs.kPIDLoopIdx, kD, HopperPIDs.kTimeoutMs);
         kD = d;
       }
      //if((iz != kIz)) { m_hopperMotor.setIZone(iz); 
      //  kIz = iz; }
      if((ff != kF)) { m_hopperMotor.config_kF(HopperPIDs.kPIDLoopIdx, kF, HopperPIDs.kTimeoutMs);
        kF = ff; 
      }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        //m_hopperMotor.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        
      }
    }
  }
  

  // Move the daisy one full rotation around
  public void dischargeAll(){
    m_reference += HopperConstants.ONE_ROTATION;
    System.out.println("HopperSubsystem Setting the hopper motor");
    //m_hopperMotor.setReference(m_reference, ControlType.kPosition);
  }



  // Move the daisy 1/6 rotation forward
  public void nextSlot(){
    m_reference += HopperConstants.ONE_ROTATION/6;
    //m_hopperMotor.setReference(m_reference, ControlType.kPosition);
  }

  // Move the daisy 1/6 rotation backward
  public void previousSlot(){
    m_reference -= HopperConstants.ONE_ROTATION/6;
    //m_hopperMotor.setReference(m_reference, ControlType.kPosition);
  }

  public boolean atSetpoint(double thresholdPercent) {
    
    return Math.abs(m_setpoint - m_hopperMotor.getSensorCollection().getPulseWidthPosition()) <= Math.abs(m_setpoint*thresholdPercent);
  }
}
