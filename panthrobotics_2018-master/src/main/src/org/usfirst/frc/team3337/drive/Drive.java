//We're in the package below
package main.src.org.usfirst.frc.team3337.drive;

import org.usfirst.frc.team3337.robot.Robot;
//We're using these other program files below for their functions.
//WPILib imports
import org.usfirst.frc.team3337.robot.RobotMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;

//CANTalon imports
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

//This is our class name. Abstract means that it cannot be instantiated.
public abstract class Drive
{

	//Declaring Variables
	double vL, vR, velocity;
	double originalForwardsAngle, originalBackwardsAngle;
	public static final double speedLimit = 1;
	
	protected boolean forwardsStarted, backwardsStarted;
	public static final double GYRO_COEFFICIENT = 0.01;
	
	//Constructor
	public Drive()
	{
		zeroVelocities();
	}
	
	//oneEncoder = new Encoder(int channelA, int channelB, boolean reverseDirection);
	//oneEncoder = new Encoder(int channelA, int channelB, boolean reverseDirection);
	
	void driveLeft(double leftInput) 
	{
		Robot.leftFront.set(ControlMode.PercentOutput, leftInput*speedLimit);
		Robot.leftBack.set(ControlMode.PercentOutput, leftInput*speedLimit);
	}
	
	//reference temp
	//driveLeft((-gtaBackwardTrigger + scaledAngleDifference) *speedLimit);
	
	void driveRight(double rightInput) 
	{
		Robot.rightFront.set(ControlMode.PercentOutput, -rightInput*speedLimit);
		Robot.rightBack.set(ControlMode.PercentOutput, -rightInput*speedLimit);
	}
	
	void zeroVelocities()
	{
		driveLeft(0);
		driveRight(0);
	}
	
	//GTA Forwards Drive
	/*protected void driveBackwards(double velocity)
	{
		if (!backwardsStarted) //if backwards GTA has not been pressed
		{
			originalBackwardsAngle = Robot.getRawYaw();
			backwardsStarted = true;
		}
        double scaledAngleDifference = (Robot.getRawYaw() - originalBackwardsAngle) * GYRO_COEFFICIENT;
		/* The right side overpowers the left. This fixes that.
		 * Since the right side is stronger than the left, the robot will turn left when going forwards.
		 * As such, the expected change in angle since the robot started will be negative.
		 * To decrease the power of the right side and increase that of the left,
		 * we will subtract the angle difference to the left side and add it to the right side.
		 */
		/*driveLeft((-velocity + scaledAngleDifference) *speedLimit);
		driveRight((-velocity - scaledAngleDifference) * speedLimit);
		/*driveLeft((gtaForwardTrigger * speedLimit) + scaledAngleDifference);
		driveRight((gtaForwardTrigger * speedLimit) - scaledAngleDifference);*/
	/*}
	
	//GTA sdrawkcaB Drive
	protected void driveForwards(double velocity)
	{
		if (!forwardsStarted) //if backwards GTA has not been pressed
		{
			originalForwardsAngle = Robot.getRawYaw();
			forwardsStarted = true;
		}
        double scaledAngleDifference = (Robot.getRawYaw() - originalForwardsAngle) * GYRO_COEFFICIENT;
		/* The right side overpowers the left. This fixes that.
		 * Since the right side is stronger than the left, the robot will turn left when going forwards.
		 * As such, the expected change in angle since the robot started will be positive.
		 * To decrease the power of the right side and increase that of the left,
		 * we will add the angle difference to the left side and subtract it from the right side.
		 */
		/*driveLeft((velocity + scaledAngleDifference) *speedLimit);
		driveRight((velocity - scaledAngleDifference) * speedLimit);
		/*driveLeft(gtaBackwardTrigger * speedLimit * -1);
		driveRight(gtaBackwardTrigger * speedLimit * -1);
	}*/
}