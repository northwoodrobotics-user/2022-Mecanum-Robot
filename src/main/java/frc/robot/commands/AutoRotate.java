/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.extraClasses.PIDControl;

public class AutoRotate extends Command {

  double robotAngle;        //angle of the robot with respect to robot front
  double gyroAngle;         //angle read from VMX gyro
  double angleCorrection;   //PID calculated correction factor
  double direction;         //direction to rotate
	double startTime;         //time at start of movement
  double stopTime;          //timeout time
  double speedMultiplier;
  boolean useVision;
	
	PIDControl pidControl;

	public Timer timer = new Timer();

  //Class constructor
  public AutoRotate(double ang, double time, double speed, boolean useVisionAngle) {

    requires(Robot.drivetrain);
 
    //Initialize internal variables
    robotAngle = ang;
    stopTime = time;
    speedMultiplier = speed;
    useVision = useVisionAngle;

    //Set up PID control
    pidControl = new PIDControl(RobotMap.kP_Turn, RobotMap.kI_Turn, RobotMap.kD_Turn);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    if (stopTime != 0.0)
    {

      timer.start();
      startTime= timer.get();

    }

    angleCorrection = 0.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    gyroAngle = Robot.gyroYaw.getDouble(0);

    if(useVision)
    {
      robotAngle = RobotMap.VISION_TARGET_ANGLE;
    }

    angleCorrection = 0.0;
    
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

    Robot.drivetrain.autoDrive(0.0, 0.0, -angleCorrection*RobotMap.AUTO_DRIVE_SPEED * speedMultiplier);    	    	
        
    SmartDashboard.putString("Angle Correction", Double.toString(angleCorrection));
    SmartDashboard.putString("Gyro Yaw", Double.toString(gyroAngle));
    SmartDashboard.putString("Gyro Angle", Double.toString(Robot.driveAngle.getDouble(0)));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {

    //Initialize return flag
    boolean thereYet = false;
 
    //Check master kill switch
    if (RobotMap.KILL_AUTO_COMMAND == true)
    {

      thereYet = true;
    }
    else
    {

      //Check elapsed time
      if (stopTime != 0.0)
      {

        if(stopTime<=timer.get()-startTime)
        {
      
          //Too much time has elapsed.  Stop this command.
          thereYet = true;
      
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}