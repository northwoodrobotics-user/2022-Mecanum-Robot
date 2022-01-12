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


public class AutoAssistAlignRobotToTarget extends Command {
  
  //Declare class level variables
  double offsetTolerance = .1;
  double cameraOffset = 0;
  boolean visionFound;
  double visionOffset;

  double angleCorrection;
  double speedCorrection;
  double gyroAngle;
  double robotAngle; //angle of the robot with respect to robot front
  double startTime;
  double stopTime;
  double speedMultiplier;
  
  public Timer timer = new Timer();
  VisionUtilities visionUtilities;
  PIDControl pidControl;
  PIDControl pidControlAlign;
  
  public AutoAssistAlignRobotToTarget(double speed) {
    
    //Required subsystems
    requires(Robot.drivetrain);

    //Set variables
    speedMultiplier = speed;

    //Create vision utilities object
    visionUtilities = new VisionUtilities();

    //Set up PID control
    pidControl = new PIDControl(RobotMap.kP_Turn, RobotMap.kI_Turn, RobotMap.kD_Turn);
    pidControlAlign = new PIDControl(RobotMap.kP_Slew, RobotMap.kI_Slew, RobotMap.kD_Slew);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    stopTime = 2.0;

    if(stopTime != 0)
    {
      timer.start();
      startTime= timer.get();
    }
 
    //Find proper target alignment angle
    //RobotMap.VISION_TARGET_ANGLE = visionUtilities.FindTargetAngle(Robot.gyroYaw.getDouble(0));

    angleCorrection = 0;
    speedCorrection = 0;
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Declare local variables
    double slewDirection = 0.0;

    //Get vision processing results
    visionFound = Robot.visionTable.getEntry("FoundVisionTarget").getBoolean(false);
    visionOffset = Robot.visionTable.getEntry("VisionTargetOffset").getDouble(0) - cameraOffset;

    //Only proceed if target has been found
    if(visionFound)
    {
      //Check vision offset
      if(visionOffset > 0)
      {
        //slew left
        slewDirection = 180;
      }
      else
      {
        //slew right
        slewDirection = 0;
      }
      
      gyroAngle = Robot.gyroYaw.getDouble(0);
      robotAngle = RobotMap.VISION_TARGET_ANGLE;

      angleCorrection = 0.0;
        
        if (robotAngle == 180 || robotAngle == -180)
        {
          if (gyroAngle >= 0 && gyroAngle < 179.5)
          {
            angleCorrection = pidControl.Run(gyroAngle, 180.0, 0.5);
          }
          else if(gyroAngle <= 0 && gyroAngle > -179.5)
          {
            angleCorrection = pidControl.Run(gyroAngle, -180.0, 0.5);
          }
        }
        else
        {
          angleCorrection = pidControl.Run(gyroAngle, robotAngle, 0.5);
        }

        speedCorrection = pidControlAlign.Run(Math.abs(visionOffset), 0, offsetTolerance);
        //speedCorrection = 1;
        SmartDashboard.putNumber("Align Correction: ", speedCorrection);

        //possibly substitute driveAngle with driveAngle - gyroAngle to allow for proper slewing
        Robot.drivetrain.autoDrive(speedCorrection * RobotMap.AUTO_DRIVE_SPEED * speedMultiplier, slewDirection, -angleCorrection*1);    	    	
    }
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
      if(stopTime != 0)
      {
        if(stopTime <= timer.get() - startTime)
        {
          //Too much time has elapsed.  Stop this command.
          thereYet = true; 		
        }
        else
        {
          if(visionFound)
          {
            if(Math.abs(visionOffset) < offsetTolerance)
            {
              thereYet = true; //We are sufficiently aligned to continue.
            }
          
          }
          else
          {
            thereYet = true; //Vision has been lost.  Stop trying.
          }
        }
      }
      else
      {
        if(visionFound)
        {
          if(Math.abs(visionOffset) < offsetTolerance)
          {
            thereYet = true; //We are sufficiently aligned to continue.
          }
        }
        else
        {
          thereYet = true; //Vision has been lost.  Stop trying.
        }
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