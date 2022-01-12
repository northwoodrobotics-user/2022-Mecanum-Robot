/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.extraClasses.PIDControl;
import frc.robot.extraClasses.VisionUtilities;

public class AutoDriveToLimitSwitch extends Command {
  
  //Declare class level variables
  double driveAngle;  //drive angle
  double stopTime;  //timeout time
  double gyroAngle;
  double angleCorrection;
  double robotAngle;      //angle of the robot with respect to robot front
  double startTime;
  double speedMultiplier;
  boolean useGyroAngle; //2019: use for driving straight forward after an assist align

  PIDControl pidControl;
  VisionUtilities visionUtilities;

  public Timer timer = new Timer();
  public Timer limitTimer = new Timer();
  private boolean limitStartTimeInit = false;
  
  boolean thereYet = false;

  double limitStartTime = 0;

  //Default constructor
  public AutoDriveToLimitSwitch(double ang, double orientAng, double time, double speed, boolean useGyro){

    requires(Robot.drivetrain);
    	
    //Set local variables
    driveAngle = ang;
    robotAngle = orientAng;
    stopTime = time;
    speedMultiplier = speed;
    useGyroAngle = useGyro;
          
    //Set up PID control
    pidControl = new PIDControl(RobotMap.kP_Straight, RobotMap.kI_Straight, RobotMap.kD_Straight);

}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    boolean visionFound = Robot.foundVisionTarget.getBoolean(false);
    if(!visionFound){

      thereYet = true;

    }

    if(stopTime != 0)
    {
        timer.start();
        startTime= timer.get();
    }

    angleCorrection = 0;

    //Find proper target alignment angle
    //RobotMap.VISION_TARGET_ANGLE = visionUtilities.FindTargetAngle(Robot.gyroYaw.getDouble(0));

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    gyroAngle = Robot.gyroYaw.getDouble(0);

    robotAngle = RobotMap.VISION_TARGET_ANGLE;

    angleCorrection = 0.0;
    
    if(useGyroAngle)
    {
      if (robotAngle == 180 || robotAngle == -180)
      {
        if (gyroAngle >= 0 && gyroAngle < 179.5)
        {
          angleCorrection = pidControl.Run(gyroAngle, 180.0, 2);
        }
        else if(gyroAngle <= 0 && gyroAngle > -179.5)
        {
          angleCorrection = pidControl.Run(gyroAngle, -180.0, 2);
        }
      }
      else
      {
        angleCorrection = pidControl.Run(gyroAngle, robotAngle, 2);
      }
    }

    //possibly substitute driveAngle with driveAngle - gyroAngle to allow for proper slewing
    Robot.drivetrain.autoDrive(RobotMap.AUTO_DRIVE_SPEED *speedMultiplier, driveAngle, -angleCorrection*0.3);    	    	
    
    SmartDashboard.putString("Angle Correction", Double.toString(angleCorrection));
    SmartDashboard.putString("Gyro Yaw", Double.toString(gyroAngle));
    SmartDashboard.putString("Gyro Angle", Double.toString(Robot.driveAngle.getDouble(0)));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    //Initialize return value
    thereYet = false;
 
    //Check for master kill switch
    if (RobotMap.KILL_AUTO_COMMAND == true)
    {

      thereYet = true;
    }
    else
    {
      //Check elapsed time and limit switch
      if(stopTime != 0)
      {
        if(!Robot.oi.hatchLimitSwitch.get() == true)
        {
          thereYet = true;
        }
        else
        {
          if(stopTime <= timer.get() - startTime)
          {
            //Too much time has elapsed.  Stop this command.
            thereYet = true; 		
          }
        }
      }
      else 
      {
        thereYet = true;
      }
    }

    //Return flag
    return thereYet;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    //Stop the robot
    Robot.drivetrain.robotStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}

}