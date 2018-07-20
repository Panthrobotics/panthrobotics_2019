package main.src.org.usfirst.frc.team3337.drive;

import org.usfirst.frc.team3337.robot.Robot;
import org.usfirst.frc.team3337.robot.RobotMap;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Joystick;

public class TeleopGameDrive extends TeleopDrive
{
	
	public TeleopGameDrive()
	{
		super();
	}
	
	void updateControls()
	{
		joyLY = -deadZone(Robot.driveController.getRawAxis(1));
		joyRY = -deadZone(Robot.driveController.getRawAxis(5));
		joyLX = deadZone(Robot.driveController.getRawAxis(0));
		joyRX = deadZone(Robot.driveController.getRawAxis(4));
		gtaForwardTrigger = Robot.driveController.getRawAxis(RobotMap.GTA_FORWARD);
		gtaBackwardTrigger = Robot.driveController.getRawAxis(RobotMap.GTA_BACKWARD);
		elevatorUp = deadZone(Robot.auxController.getRawAxis(RobotMap.ELEVATOR_UP));
		elevatorDown = deadZone(Robot.auxController.getRawAxis(RobotMap.ELEVATOR_DOWN));
	}
	
}
