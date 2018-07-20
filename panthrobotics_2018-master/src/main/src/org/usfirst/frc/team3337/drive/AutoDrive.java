//The package this file is in.
package main.src.org.usfirst.frc.team3337.drive;

import java.util.LinkedHashMap;

import org.usfirst.frc.team3337.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
//Importing the classes below to use them.
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//This class is AutoDrive and is a child of Drive.
public class AutoDrive extends Drive
{
	
	AutoChoice autoChoice;
	AutoInitialPosition autoInitialPosition;
	SendableChooser<AutoChoice> autoChooser;
	SendableChooser<AutoInitialPosition> positionChooser;
	LinkedHashMap<Double, Double> leftDriveDynamicAutonomous, rightDriveDynamicAutonomous;
	
	double previousTime;
	boolean ourSwitchIsLeft, scaleIsLeft, otherSwitchIsLeft;
	boolean dataWasRead;
	
	//Constructor for AutoDrive.
	public AutoDrive()
	{
		super();
		
        autoChooser = new SendableChooser<AutoChoice>();
        autoChooser.addDefault(AutoChoice.GO_STRAIGHT.toString(), AutoChoice.GO_STRAIGHT);
        for (AutoChoice d: AutoChoice.possibleOptions())
            autoChooser.addObject(d.toString(), d);
        SmartDashboard.putData("Autonomous Mode autoChoice", autoChooser);	
        
        positionChooser = new SendableChooser<AutoInitialPosition>();
        positionChooser.addDefault(AutoInitialPosition.MIDDLE.toString(), AutoInitialPosition.MIDDLE);
        for (AutoInitialPosition d: AutoInitialPosition.possibleOptions())
            positionChooser.addObject(d.toString(), d);
        SmartDashboard.putData("Autonomous Position autoInitialPosition", positionChooser);
        
        forwardsStarted = false;
        backwardsStarted = false;
        
        //dataWasRead = false;
        
        previousTime = -1;
	}
	
	//Method to be run in autonomousInit() in Robot.java
	public void init()
	{
		Robot.dynamicAutonomousTimer.reset();
		Robot.dynamicAutonomousTimer.start();
		Timer.delay(0.5); //Wait for Driver Station 3-character values to come in.
	}
	
	//Method to be run in autonomousPeriodic() in Robot.java
	public void periodic()
	{
		String dynamicAutonomousPath = "" + Robot.DYNAMIC_AUTONOMOUS_BASE_FOLDER;
		double roundedTime = Robot.dynamicAutonomousTimer.get() - (Robot.dynamicAutonomousTimer.get() % 0.01);
		
		/*The method below returns a string of three characters decided by the competition management system.
		 *The three characters represent where our alliance's switch/scale ports are.
		 *The characters are organized by distance away from alliance players:
		 *	First character = our switch
		 *	Second character = the scale
		 *	Third character = other alliance's switch
		 *If a character is 'L', the component referenced is to the left; if 'R', to the right.
		 */
		
		autoChoice = autoChooser.getSelected();
		autoInitialPosition = positionChooser.getSelected();
		
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		ourSwitchIsLeft = true; //gameData.charAt(0) == 'L';
		scaleIsLeft = true; //gameData.charAt(1) == 'L';
		otherSwitchIsLeft = true; //gameData.charAt(2) == 'L';
		
		switch (autoChoice)
		{
		case GO_STRAIGHT:
		default:
			if (Robot.PLAYING_DYNAMIC_AUTONOMOUS)
			{
				dynamicAutonomousPath += "/gostraight" + Robot.DYNAMIC_AUTONOMOUS_PLAY_VERSION;
				/*
				 * 1) Gather data ONCE
				 * 2) At every time t, get the input from the data and apply it.
				 */
				if (roundedTime != previousTime) //This saves memory consumption from constantly checking the Map.
				{
					driveLeft(leftDriveDynamicAutonomous.get(new Double(roundedTime)));
					driveRight(rightDriveDynamicAutonomous.get(new Double(roundedTime)));
				}
				if (!dataWasRead)
				{
					leftDriveDynamicAutonomous = Robot.readData(dynamicAutonomousPath + Robot.DYNAMIC_AUTONOMOUS_LEFT_DRIVE_FILE);
					rightDriveDynamicAutonomous = Robot.readData(dynamicAutonomousPath + Robot.DYNAMIC_AUTONOMOUS_RIGHT_DRIVE_FILE);
					dataWasRead = true;
				}
			}
			else
			{
				if (Robot.dynamicAutonomousTimer.get() < 5)
				{
					//Competition code
					//driveLeft(0.4);
					//driveRight(0.4);
					
					Robot.rightFront.set(ControlMode.PercentOutput, 0.4);
				}
				else
					Robot.rightFront.set(ControlMode.PercentOutput, 0);
			}
			break;
		case OUR_SWITCH:
			switch (autoInitialPosition)
			{
			case LEFT:
				break;
			case MIDDLE:
				break;
			case RIGHT:
				break;
			}
			break;
		case SCALE:
			switch (autoInitialPosition)
			{
			case LEFT:
				break;
			case MIDDLE:
				break;
			case RIGHT:
				break;
			}
			break;
		case OTHER_SWITCH:
			switch (autoInitialPosition)
			{
			case LEFT:
				break;
			case MIDDLE:
				break;
			case RIGHT:
				break;
			}
			break;
		}
		
		//previousTime = roundedTime;
	}
	
}
