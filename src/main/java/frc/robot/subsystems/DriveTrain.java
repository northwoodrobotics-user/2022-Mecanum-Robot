/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private ADXRS450_Gyro gyro;
  //private LinearFilter gyro_filter;
  private MedianFilter gyro_filter;

  private WPI_TalonFX leftMasterFalcon;
  private WPI_TalonFX leftSlaveFalcon;

  private WPI_TalonFX rightMasterFalcon;
  private WPI_TalonFX rightSlaveFalcon;

  private SpeedControllerGroup leftMotorGroup;
  private SpeedControllerGroup rightMotorGroup;

  private DifferentialDrive drivetrain;

  private DigitalInput processorEntry = new DigitalInput(PROCESSOR_INDEX_1);

  
  /** 
   * 
   * Drivetrain constructor 
   * 
   * Initialize motors, encoders, and gyro
   * 
   */
  public Drivetrain() {

    // Initialize drivetrain motors
    initFalconDrivetrain();

    // Initialize Roborio gyro
    gyro = new ADXRS450_Gyro();
    SmartDashboard.putNumber("Zero Gyro", 0);
    gyro.calibrate();
    zeroGyro();

    // Initialize moving average filter for gyro
    //gyro_filter = LinearFilter.movingAverage(FILTER_WINDOW_SIZE);
    gyro_filter = new MedianFilter(FILTER_WINDOW_SIZE);

    // Zero drivetrain encoders
    SmartDashboard.putNumber("Zero Encoders", 0);
    zeroEncoders();
  }


  /**
   * 
   * Drivetrain periodic
   * 
   * Things that need to happen on a periodic basis as
   * the drivetrain is being used.
   * 
   */
  @Override
  public void periodic() {

    // Put info on the dashboard
    SmartDashboard.putNumber("Gyro", getGyroAngle());
    SmartDashboard.putNumber("Left Master Encoder", getMasterLeftEncoderPosition());
    SmartDashboard.putNumber("Right Master Encoder", getMasterRightEncoderPosition());
    SmartDashboard.putNumber("Left Master Velocity", leftMasterFalcon.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Right Master Velocity", rightMasterFalcon.getSelectedSensorVelocity());

    
    //Graphable boolean :)
    if (getProcessorEntry()){
      SmartDashboard.putNumber("Processor Entry", 1);
    } else {
      SmartDashboard.putNumber("Processor Entry", 0);
    }

    // Zero the gyro on driver command
    double zeroGyro = SmartDashboard.getNumber("Zero Gyro", 0);
    if (zeroGyro == 1)
    {
      SmartDashboard.putNumber("Zero Gyro", 0);
      gyro.calibrate();
      zeroGyro();
    }

    // Zero the encoders on driver command
    double zeroEncoders = SmartDashboard.getNumber("Zero Encoders", 0);
    if (zeroEncoders == 1)
    {
      SmartDashboard.putNumber("Zero Encoders", 0);
      gyro.calibrate();
      zeroEncoders();
    }


  }


  /**
   * 
   * Configure the drivetrain motors as a differential drive
   * and configure encoders for left and right sides
   * 
   */
  private void initFalconDrivetrain(){

    //Init motors, speed controller groups, and drivetrain
    leftMasterFalcon = new WPI_TalonFX(LEFT_MASTER_F);
    leftSlaveFalcon = new WPI_TalonFX(LEFT_SLAVE_F);
    leftMotorGroup = new SpeedControllerGroup(leftMasterFalcon, leftSlaveFalcon);

    rightMasterFalcon = new WPI_TalonFX(RIGHT_MASTER_F);
    rightSlaveFalcon = new WPI_TalonFX(RIGHT_SLAVE_F);
    rightMotorGroup = new SpeedControllerGroup(rightMasterFalcon, rightSlaveFalcon);

    drivetrain = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
    drivetrain.setRightSideInverted(false);

    //Set follower mode
    leftSlaveFalcon.follow(leftMasterFalcon);
    rightSlaveFalcon.follow(rightMasterFalcon);

    //Set brake mode (check Phoenix Tuner software to confirm)
    leftMasterFalcon.setNeutralMode(NeutralMode.Brake);
    leftSlaveFalcon.setNeutralMode(NeutralMode.Brake);
    rightMasterFalcon.setNeutralMode(NeutralMode.Brake);
    rightSlaveFalcon.setNeutralMode(NeutralMode.Brake);

    //Config encoders
    leftMasterFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    leftSlaveFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    rightMasterFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    rightSlaveFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);

    //Invert appropriately
    leftMasterFalcon.setInverted(!kMotorInvert);
    leftSlaveFalcon.setInverted(!kMotorInvert);
    rightMasterFalcon.setInverted(kMotorInvert);
    rightSlaveFalcon.setInverted(kMotorInvert);

    //Zero encoders
    leftMasterFalcon.setSelectedSensorPosition(0);
    leftSlaveFalcon.setSelectedSensorPosition(0);
    rightMasterFalcon.setSelectedSensorPosition(0);
    rightSlaveFalcon.setSelectedSensorPosition(0);

    SmartDashboard.putNumber("Left Drive Speed", 0);
    SmartDashboard.putNumber("Right Drive Speed", 0);
  }


  /**
   * 
   * Main teleop drive method
   * 
   * @param leftJoyY: left joystick position
   * @param rightJoyY: right joystick position
   * 
   */
  public void drive(double leftJoyY, double rightJoyY) {

    //Configure software-based voltage protection measure (will require testing to determine optimal values)
    double speedCap = kHighGearSpeedCap;

    // Drive the motors
    // Direction multiplier indicates drive direction
    if(DIRECTION_MULTIPLIER == 1){
      SmartDashboard.putNumber("Left Drive Speed", speedCap * DIRECTION_MULTIPLIER * leftJoyY);
      SmartDashboard.putNumber("Right Drive Speed", speedCap * DIRECTION_MULTIPLIER * rightJoyY);
      drivetrain.tankDrive(speedCap * DIRECTION_MULTIPLIER * leftJoyY, speedCap * DIRECTION_MULTIPLIER * rightJoyY);   
    
    }
    else{
      SmartDashboard.putNumber("Left Drive Speed", speedCap * DIRECTION_MULTIPLIER * rightJoyY);
      SmartDashboard.putNumber("Right Drive Speed", speedCap * DIRECTION_MULTIPLIER * leftJoyY);
      drivetrain.tankDrive(speedCap * DIRECTION_MULTIPLIER * rightJoyY, speedCap * DIRECTION_MULTIPLIER * leftJoyY); 
    
    }

  }


  /**
   * 
   * Run drivetrain during autonomous
   * 
   * @param leftSpeed: speed for left side motors
   * @param rightSpeed: speed for right side motors
   * 
   */
  public void autoDrive(double leftSpeed, double rightSpeed){
    SmartDashboard.putNumber("Left Drive Speed", leftSpeed);
    SmartDashboard.putNumber("Right Drive Speed", rightSpeed);
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }


  /** Stop the drive train */
  public void stopDrive(){

    drivetrain.tankDrive(0, 0);
  }

  public void changeSpeed(){
    if (kJoystickSpeedCorr == 0.55){
      kJoystickSpeedCorr = .7;
    } else if (kJoystickSpeedCorr == 0.7) {
      kJoystickSpeedCorr = 0.55;
    }
  }

  /** Zero the encoders */
  public void zeroEncoders(){

    leftMasterFalcon.setSelectedSensorPosition(0);
    leftSlaveFalcon.setSelectedSensorPosition(0);
    rightMasterFalcon.setSelectedSensorPosition(0);
    rightSlaveFalcon.setSelectedSensorPosition(0);

  }


  /**
   * 
   * Get position of all left encoders
   * 
   * @return array of encoder positions
   * 
   */
  public double[] getLeftEncoders(){

    double[] encoders = new double[2];

    encoders[0] = leftMasterFalcon.getSelectedSensorPosition();
    encoders[1] = leftSlaveFalcon.getSelectedSensorPosition();
    
    return encoders;

  }

  /**
   * 
   * Get position of all right encoders
   * 
   * @return array of encoder positions 
   * 
   */
  public double[] getRightEncoders(){

    double[] encoders = new double[2];

    encoders[0] = rightMasterFalcon.getSelectedSensorPosition();
    encoders[1] = rightSlaveFalcon.getSelectedSensorPosition();
    
    return encoders;

  }

  /**
   * 
   * Get position of left master encoder
   * 
   * @return encoder position
   * 
   */
  public double getMasterLeftEncoderPosition(){

    return leftMasterFalcon.getSelectedSensorPosition();

  }


  /**
   * 
   * Get position of right master encoder
   * 
   * @return encoder position
   * 
   */
  public double getMasterRightEncoderPosition(){

    return rightMasterFalcon.getSelectedSensorPosition();

  }


  /**
   * 
   * Get current gyro angle
   * 
   * @return angle
   * 
   */
  public double getGyroAngle(){

    double correctedGyro = gyro_filter.calculate(gyro.getAngle() % 360.0);
    return correctedGyro;

  }


  /**
   * 
   * Reset current gyro heading to zero
   * 
   */
  public void zeroGyro(){
    // gyro.calibrate();
    gyro.reset();

  }
  

  /** Invert the direction of driving */
  public void invertDirection(){

    DIRECTION_MULTIPLIER *= -1;

  }

  public boolean getProcessorEntry(){
    return processorEntry.get();
  }
}
