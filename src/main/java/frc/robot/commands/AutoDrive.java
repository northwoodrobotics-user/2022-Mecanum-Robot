package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.extraClasses.PIDControl;

public class AutoDrive extends Command {

    double driveAngle;  //drive angle
    double stopTime;  //timeout time
    double gyroAngle;
    double angleCorrection;
    double robotAngle;      //angle of the robot with respect to robot front
    double startTime;
    double speedMultiplier;
	
	PIDControl pidControl;

	public Timer timer = new Timer();
	
	//Class constructor
    public AutoDrive(double ang, double orientAng, double time, double speed) {
    	
    	requires(Robot.drivetrain);
    	
    	//Set local variables
        driveAngle = ang;
        robotAngle = orientAng;
        stopTime = time;
        speedMultiplier = speed;
    	    	
    	//Set up PID control
    	pidControl = new PIDControl(RobotMap.kP_Straight, RobotMap.kI_Straight, RobotMap.kD_Straight);
    	
    }

    
    // Called just before this Command runs the first time
    protected void initialize() {
        
        if(stopTime != 0)
        {
            timer.start();
            startTime= timer.get();
        }

        angleCorrection = 0;
        
    }

    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        
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
        
        //possibly substitute driveAngle with driveAngle - gyroAngle to allow for proper slewing
        Robot.drivetrain.autoDrive(RobotMap.AUTO_DRIVE_SPEED * speedMultiplier, driveAngle, -angleCorrection*0.3);    	    	
        
        SmartDashboard.putString("Angle Correction", Double.toString(angleCorrection));
        SmartDashboard.putString("Gyro Yaw", Double.toString(gyroAngle));
        SmartDashboard.putString("Gyro Angle", Double.toString(Robot.driveAngle.getDouble(0)));

    }

    
    // Make this return true when this Command no longer needs to run execute()
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
    protected void end() {

        //Stop the robot
        Robot.drivetrain.robotStop();
        
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {}
    
}