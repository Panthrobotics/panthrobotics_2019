package main.src.org.usfirst.frc.team3337.drive;

import org.usfirst.frc.team3337.robot.Robot;
import org.usfirst.frc.team3337.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.Solenoid;



public class Actuators
{
	public enum SolenoidStage
	{
		NONE, SUPPORT_PISTON_RISE, MAIN_PISTONS_RISE, MAIN_PISTONS_LOWER;
		
	    public SolenoidStage getNext() {
	        if (ordinal() + 1 == values().length)
	            return values()[0];
	        return values()[ordinal() + 1];
	    }
	}
	
	
	DigitalInput limitSwitch;
	//JoystickButton bButton, xButton, yButton, leftBumper, rightBumper/*triggerButton, thumbButton*/;
	JoystickButton intakeInButton, intakeOutButton, solenoidButton;
	SolenoidStage pneumaticStage;
	ToggleButton aToggle;
	
	private boolean solenoidButtonPressed;
	private double intakeAngleHoldValue;
	public static final double ROTATIONS_TO_REACH_SWITCH = 7000;
	public static final double MAX_ROTATIONS = 1000;
	
	
	public Actuators()
	{
		//aToggle =  new ToggleButton(new JoystickButton(Robot.auxController, 1));
		/*bButton = new JoystickButton(Robot.auxController, 2);
		xButton = new JoystickButton(Robot.auxController, 3);
		yButton = new JoystickButton(Robot.auxController, 4);
		leftBumper = new JoystickButton(Robot.auxController, 5);
		rightBumper = new JoystickButton(Robot.auxController, 6);*/
		
		intakeInButton = new JoystickButton(Robot.auxController, 3);
		intakeOutButton = new JoystickButton(Robot.auxController, 1);
		solenoidButton = new JoystickButton(Robot.auxController, 2);
		
		
		pneumaticStage = SolenoidStage.NONE;
		
		limitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH);
		//Conversion Stuff
		
		solenoidButtonPressed = false;
		
		double inchConversion = SmartDashboard.getNumber("Inches Measured", 0);
		
		intakeAngleHoldValue = 0;
		 
	}
	
	public void setIntake(double velocity)
	{
		Robot.rightArm.set(velocity);
		Robot.leftArm.set(velocity);
	}
	
	/*public void driveLift(double velocity)
	{
		Robot.elevatorMotorOne.set(ControlMode.PercentOutput, -velocity);
		Robot.elevatorMotorTwo.set(ControlMode.PercentOutput, -velocity);
	}*/
	
	public void periodic()
	{
		//double elevatorUpTrigger = Robot.auxController.getRawAxis(RobotMap.ELEVATOR_UP);
		//double elevatorDownTrigger = Robot.auxController.getRawAxis(RobotMap.ELEVATOR_DOWN);
		//double elevatorUpTrigger = Robot.auxController.getRawAxis();
		//double elevatorDownTrigger = Robot.auxController.getRawAxis();
		//int elevatorPosition = Robot.elevatorMotorOne.getSensorCollection().getQuadraturePosition();
		
		if (intakeInButton.get()) //Formerly the B Button
			setIntake(0.6);
		else if (intakeOutButton.get()) //Formerly the X Button
			setIntake(-1);
		else
			setIntake(0);
		
		
		//Robot.intakeAngleMotor.set(ControlMode.PercentOutput, -0.5 * Robot.auxController.getRawAxis(1)); //joyLY
		Robot.intakeAngleMotor.set(0.5 * Robot.auxController.getRawAxis(1));
		//Robot.intakeAngleMotor.set(0.6);// * Robot.auxController.getRawAxis(1));
		
		/*
		 * if (aToggle.justPressed())
		 *     intakeAngleHoldValue = -0.35 * Robot.auxController.getRawAxis(1);
		 * if (aToggle.get())
		 *     Robot.intakeAngleMotor.set(ControlMode.PercentOutput, -0.35 * Robot.auxController.getRawAxis(1));
		 * else
		 *     Robot.intakeAngleMotor.set(ControlMode.PercentOutput, -0.35 * Robot.auxController.getRawAxis(1));S
		 */
		
		/*Single Joystick Controls Potentially, where auxController is a singular joystick.
		 * zScalar = ((Robot.auxController.getRawAxis(2) + 1) / 2);
		 * intakeInput = Robot.auxController.getRawAxis(1);
		 * Robot.intakeAngleMotor.set(ControlMode.PercentOutput, (zScalar * intakeInput));
		 * TODO: Create a loop where it when a button is pressed it locks the value of the motors that are inputted from the joystick.
		 * if (thumbButton.get() = true)
		 * {
		 * 		Robot.rightArm.set(velocity);
		 * 		Robot.lefttArm.set(velocity);
		 * }
		 * else if (thumbButton.get() = true)
		 * {
		 * 		Robot.rightArm.set(-velocity);
		 * 		Robot.lefttArm.set(-velocity);
		 * }	
		 * else
		 * {
		 * 		Robot.rightArm.set(0);
		 * 		Robot.lefttArm.set(0);
		 * }
		 * }
		 */
		
		
		/*if (elevatorUpTrigger > 0)
		{
			if (elevatorPosition < MAX_ROTATIONS)
				driveLift(elevatorUpTrigger);
			else
				driveLift(0);
		}
		else
		{
			if (elevatorPosition > 100)
				driveLift(-elevatorDownTrigger);
			else
				driveLift(0);
		}*/
		/*else if (Robot.auxController.getRawAxis(RobotMap.ELEVATOR_UP) > 0)
			driveLift(Robot.auxController.getRawAxis(RobotMap.ELEVATOR_UP));
		else 
			driveLift(-Robot.auxController.getRawAxis(RobotMap.ELEVATOR_DOWN));*/
		
		//if (yButton.get())
		if (solenoidButton.get())
		{
			if (!solenoidButtonPressed)
				pneumaticStage = pneumaticStage.getNext();
			solenoidButtonPressed = true;
		}
		else
			solenoidButtonPressed = false;
		
		switch(pneumaticStage)
		{
		case NONE:
		default:
			break;
		case SUPPORT_PISTON_RISE:
			Robot.supportPiston.set(true);
			break;
		case MAIN_PISTONS_RISE:
			Robot.mainPistons.set(DoubleSolenoid.Value.kForward); //TODO: Is this value correct????
			break;
		case MAIN_PISTONS_LOWER:
			Robot.mainPistons.set(DoubleSolenoid.Value.kReverse); //TODO: Correct???
			break;
		}
	}
}
