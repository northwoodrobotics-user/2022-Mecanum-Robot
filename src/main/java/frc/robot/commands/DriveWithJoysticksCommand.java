/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.extraClasses.PIDControl;

public class DriveWithJoysticksCommand extends Command {
  
  //Declare class level variables
  boolean assistDriver = false;
  double angleCorrection = 0.0;
  double gyroAngle;
  double robotAngle;
  double joystickDeadband = 0.05;
  double rotationDeadband = .15;
  double lastJoystickX = 0.0;
  double lastJoystickY = 0.0;
  double lastJoystickZ = 0.0;

  PIDControl pidControl;

  //Default constructor
  public DriveWithJoysticksCommand(boolean assist) {
    
    requires(Robot.drivetrain);

    //Set values
    assistDriver = assist;

    //Set up PID control
    pidControl = new PIDControl(RobotMap.kP_Straight, RobotMap.kI_Straight, RobotMap.kD_Straight);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    //Initialize values
    angleCorrection = 0.0;
    lastJoystickX = Robot.oi.rightJoy.getX();
    lastJoystickY = Robot.oi.rightJoy.getY();
    lastJoystickZ = Robot.oi.rightJoy.getZ();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //Determine if driver assistance is engaged
    if (assistDriver == false)
    {

      //Drive normally
      if(Math.abs(Robot.oi.rightJoy.getZ()) > rotationDeadband)
      {
        Robot.drivetrain.drive(-Robot.oi.rightJoy.getY(), Robot.oi.rightJoy.getX(), Robot.oi.rightJoy.getZ() * RobotMap.MECANUM_TURN_MULTIPLIER, false);
      }
      else
      {
        Robot.drivetrain.drive(-Robot.oi.rightJoy.getY(), Robot.oi.rightJoy.getX(), 0, false);
      }
    }
    else
    {

      //If driver is twisting joystick, drive normally
      if (Math.abs(Robot.oi.rightJoy.getZ()) > joystickDeadband)
      {

        //Drive normally
        Robot.drivetrain.drive(-Robot.oi.rightJoy.getY(), Robot.oi.rightJoy.getX(), Robot.oi.rightJoy.getZ(), false);

      }
      else
      {

        //Find new target angle if new joystick positions
        if (Math.abs(Robot.oi.rightJoy.getX() - lastJoystickX) > joystickDeadband || Math.abs(Robot.oi.rightJoy.getX() - lastJoystickX) > joystickDeadband || Math.abs(Robot.oi.rightJoy.getZ() - lastJoystickZ) > joystickDeadband)
        {
          robotAngle = Robot.gyroYaw.getDouble(0);
        }

        //Get the current gyro angle
        gyroAngle = Robot.gyroYaw.getDouble(0);

        //Initialize angle correction
        angleCorrection = 0.0;
      
        //Find angle correction with PID controller
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

        Robot.drivetrain.drive(-Robot.oi.rightJoy.getY(), Robot.oi.rightJoy.getX(), -angleCorrection*0.3, false);

      }
    
    }

    lastJoystickX = Robot.oi.rightJoy.getX();
    lastJoystickY = Robot.oi.rightJoy.getY();
    lastJoystickZ = Robot.oi.rightJoy.getZ();

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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