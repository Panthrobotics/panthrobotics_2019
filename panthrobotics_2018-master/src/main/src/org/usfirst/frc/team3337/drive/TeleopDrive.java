//Our package
package main.src.org.usfirst.frc.team3337.drive;

import org.usfirst.frc.team3337.robot.Robot;
//Our other classes
import org.usfirst.frc.team3337.robot.RobotMap;
import main.src.org.usfirst.frc.team3337.drive.ToggleButton;

import com.ctre.phoenix.motorcontrol.ControlMode;
//Cross the Road Electronics packages
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.buttons.Button;
//WPI Library packages
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

//This is the class TeleopDrive. It is a child of Drive. It is abstract.
public abstract class TeleopDrive extends Drive
{

	//Class variables
	ToggleButton driveSwitchButton, speedDecrease, reverseButton;
	//make a RobotMap value for bumpers for auto buttons and triggers for manual buttons
	Button autoRaiseElevator, autoLowerElevator, switchButton, manualRaiseElevator, manualLowerElevator, elevatorMotorOneButton, elevatorMotorTwoButton;
	Timer tempTimer;
	
	double previousVelocity, reverse, velocity, previousAngle;	
	double joyLY, joyLX, joyRY, joyRX, gtaForwardTrigger, gtaBackwardTrigger, elevatorUp, elevatorDown;
	
	public static final double SLOW_SPEED = 0.5;
	
	//Constructor
	public TeleopDrive()
	{
		//Calling Drive's constructor
		super();
		
		driveSwitchButton = new ToggleButton(new JoystickButton(Robot.driveController, RobotMap.DRIVE_SWITCH_TOGGLE));
		speedDecrease = new ToggleButton(new JoystickButton(Robot.driveController, RobotMap.SPEED_DECREASE));
		reverseButton = new ToggleButton(new JoystickButton(Robot.driveController, RobotMap.REVERSE));
		
		elevatorMotorOneButton = new JoystickButton(Robot.auxController, RobotMap.LIFT_MOTOR_1);
		elevatorMotorTwoButton = new JoystickButton(Robot.auxController, RobotMap.LIFT_MOTOR_2);
		
		backwardsStarted = false;
		forwardsStarted = false;
	}
	
	//Arcade Drive
	private void arcadeDrive(boolean reverse)
	{
		vL = (joyRY + joyRX/2) * speedLimit;
		vR = (joyRY - joyRX/2) * speedLimit;
		if (reverse)
		{
			driveLeft(-vL);
			driveRight(-vR);
		}
		else
		{
			driveLeft(vL * speedLimit);
			driveRight(vR * speedLimit);
		}
	}
	
	//Tank Drive
	private void tankDrive(boolean reverse)
	{
		if (reverse)
		{
			driveLeft(-(speedLimit * joyLY));
			driveRight(-(speedLimit * joyRY));
		}
		else
		{
			driveLeft(speedLimit * joyLY);
			driveRight(speedLimit * joyRY);
		}
	}
	
	//This is a function that must be implemented by the child class.
	abstract void updateControls();
	
	//Making function to be called during teleopInit().
	public void init()
	{
		Robot.dynamicAutonomousTimer.reset();
		Robot.dynamicAutonomousTimer.start();
	}
	
	//Making function to be called during teleopPeriodic().
	public void periodic()
	{
		ToggleButton.updateToggleButtons();
		updateControls(); //This gets the values for joystick inputs from the child class.
		
		boolean isReverse = reverseButton.get();
		/*
		 * Priority of drive modes:
		 * 1) GTA Forward
		 * 2) GTA Backward
		 * 3) Arcade and Tank
		 */
		/*if (gtaForwardTrigger > 0 || (gtaBackwardTrigger > 0 && isReverse)) //TODO: add turning (after autonomous)
		{
			if (isReverse)
				driveForwards(gtaBackwardTrigger);
			else
				driveForwards(gtaForwardTrigger);
			SmartDashboard.putString("status", "gtaForward");
			backwardsStarted = false;
		}
		else if (gtaBackwardTrigger > 0 || (gtaForwardTrigger > 0 && isReverse)) //TODO: add turning (after autonomous)
		{
			if (isReverse)
				driveBackwards(gtaForwardTrigger);
			else
				driveBackwards(gtaBackwardTrigger);
			SmartDashboard.putString("status", "gtaBackward");
			forwardsStarted = false;
		}
		else*/ if (driveSwitchButton.get())
		{
			arcadeDrive(isReverse);
			backwardsStarted = false;
			forwardsStarted = false;
		}
		else
		{
			tankDrive(isReverse);
			backwardsStarted = false;
			forwardsStarted = false;
		}
		
		if (speedDecrease.get())
    	{
    		vL *= SLOW_SPEED;
    		vR *= SLOW_SPEED;
    	}
		
		/*
		 * This is where recording of teleop is called.
		 * Since the robot needs to record its data at some point during TeleOp,
		 * and Autonomous only lasts 15 seconds,
		 * the robot will record its teleop after 20 seconds have passed.
		 */
		if (Robot.RECORDING_DYNAMIC_AUTONOMOUS)
		{
			if (Robot.dynamicAutonomousTimer.get() < 20)
			{
				double roundedTime = Robot.dynamicAutonomousTimer.get() - (Robot.dynamicAutonomousTimer.get() % 0.01);
				Robot.addLeftDriveData(roundedTime, Robot.leftFront.getMotorOutputPercent());
				Robot.addRightDriveData(roundedTime, Robot.rightFront.getMotorOutputPercent());
			}
			else
			{
				Robot.recordLeftDriveData(Robot.DYNAMIC_AUTONOMOUS_RECORDING_PATH_FOLDER + Robot.DYNAMIC_AUTONOMOUS_LEFT_DRIVE_FILE);
				Robot.recordRightDriveData(Robot.DYNAMIC_AUTONOMOUS_RECORDING_PATH_FOLDER + Robot.DYNAMIC_AUTONOMOUS_RIGHT_DRIVE_FILE);
				Robot.RECORDING_DYNAMIC_AUTONOMOUS = false;
			}
		}
		
		//previousAngle = Robot.getRawYaw();
	}
	
	double deadZone(double value)
	{
		if (Math.abs(value) < 0.05)
			return 0;
		return value;
	}
	
}
