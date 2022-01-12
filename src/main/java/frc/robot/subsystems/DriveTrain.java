/*----------------------------------------------------------------------------*/
/* Copyright (c) 2022 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.DriveWithJoysticksCommand;


/**
 * This class defines the standard, 4-wheel mecanum drive train
 */
public class DriveTrain {

  //Define local variables
  double gyroAngle;
  double speedX, speedY, speedZ;

	//Initialize motor controllers
  public WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(RobotMap.FRONT_LEFT_MOTOR_ID);
  public WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(RobotMap.FRONT_RIGHT_MOTOR_ID);    
  public WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(RobotMap.BACK_LEFT_MOTOR_ID);
  public WPI_TalonSRX backRightMotor = new WPI_TalonSRX(RobotMap.BACK_RIGHT_MOTOR_ID);


  //Initialize mecanum drive
  MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
  
  //Teleop drive method
  @Override
  public void drive(double rightJoyX, double rightJoyY, double rightJoyZ, boolean useGyro) {
    
    frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    frontRightMotor.setNeutralMode(NeutralMode.Brake);
    backLeftMotor.setNeutralMode(NeutralMode.Brake);
    backRightMotor.setNeutralMode(NeutralMode.Brake);

    //Set properties of drive
    mecanumDrive.setSafetyEnabled(false);	
    mecanumDrive.setMaxOutput(0.95);
    
    //Get joystick values and scale
    speedX = rightJoyX * RobotMap.DRIVE_SPEED;
    speedY = rightJoyY * RobotMap.DRIVE_SPEED;
    speedZ = rightJoyZ * RobotMap.DRIVE_SPEED;

    //If using the pi gyro board, mod the returned value by 360 to avoid a spinning robot
    if(useGyro) {

      gyroAngle = Robot.driveAngle.getDouble(0) % 360.0;
  
      mecanumDrive.driveCartesian(speedX,  speedY,  speedZ, gyroAngle);
  
    } else {

      mecanumDrive.driveCartesian(speedX,  speedY,  speedZ);

    }

    // SmartDashboard.putString("Gyro Angle", Double.toString(gyroAngle));
    // SmartDashboard.putString("Gyro Yaw", Double.toString(Robot.gyroYaw.getDouble(0)));

    SmartDashboard.putNumber("Front Left Drive", frontLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Back Left Drive", backLeftMotor.getOutputCurrent());
    SmartDashboard.putNumber("Front Right Drive", frontRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("Back Right Drive", backRightMotor.getOutputCurrent());
  }


  // WCD autonomous drive method - not used.  Needs to be here for inheritance reasons
  @Override
  public void autoDrive(double leftSpeed, double rightSpeed) {
  }

  //Mecanum autonomous drive method
  @Override
  public void autoDrive(double speed, double angle, double rotation){

    mecanumDrive.setSafetyEnabled(false);
		
		mecanumDrive.setMaxOutput(0.95);

    mecanumDrive.drivePolar(speed, angle, rotation);

  }

  //Method to halt the robot if necessary
  @Override
  public void robotStop(){

    mecanumDrive.drivePolar(0, 0, 0);
  }

  // the stuff below are Copied from genericDrivetrain
  public void initDefaultCommand() {
        
    //All drive trains have default command to drive with the joysticks
      setDefaultCommand(new DriveWithJoysticksCommand(false));
  }
    //Method for driving during Teleop - used for WCD
    public void drive(double leftJoyX, double leftJoyY, double rightJoyX, double rightJoyY) {}
    
    public void drive(double rightJoyX, double rightJoyY, double rightJoyZ, boolean useGyro) {}

    //Method for driving during Autonomous - used for WCD
    public void autoDrive(double leftSpeed, double rightSpeed) {}

    //Method for driving during Autonomous - used for Mecanum
    public void autoDrive(double speed, double angle, double rotation) {}

    //Method to halt the robot if necessary
    public void robotStop(){}

}
