//We're in the package below
package org.usfirst.frc.team3337.robot;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.HashMap;
import java.util.LinkedHashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

//We're using these other program files below for their functions.

//Cross the Road Electronics packages
import com.ctre.phoenix.motorcontrol.can.TalonSRX; //CANTalon class
import com.ctre.phoenix.sensors.PigeonIMU; //Pigeon gyro class

import edu.wpi.cscore.UsbCamera;
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogGyro;

//WPI Library packages
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import main.src.org.usfirst.frc.team3337.drive.Actuators;
import main.src.org.usfirst.frc.team3337.drive.AutoDrive;
//3337 packages
import main.src.org.usfirst.frc.team3337.drive.TeleopGameDrive;

//This is our class name. It is a child of IterativeRobot.
public class Robot extends IterativeRobot {
	
	//Declaring Variables
	//public static AHRS gyro; //Example code for the gyro is at C:\Users\Panthrobotics\navx-mxp\java\examples
	public static Actuators peripherals;
	public static Joystick driveController, auxController;
	//public static PigeonIMU gyro;
	public static TalonSRX leftFront, leftBack, rightFront, rightBack;//, elevatorMotorOne, elevatorMotorTwo;//, intakeAngleMotor;
	public static Timer dynamicAutonomousTimer, time;
	public static Spark rightArm, leftArm, intakeAngleMotor;
	public static Encoder leftEncoder, rightEncoder;
	public static Solenoid supportPiston;
	public static DoubleSolenoid mainPistons;
	
	private static LinkedHashMap<Double, Double> dynamicAutonomousLeftDrive;
	private static LinkedHashMap<Double, Double> dynamicAutonomousRightDrive;
	
	AutoDrive autoDrive;
	TeleopGameDrive teleopDrive;
	
	StringBuilder rbSB, lbSB, lfSB;
	
	int kTimeoutMs = 100;
	int kSlotIdx = 0;
	int kPIDLoopIdx = 1;
	int loops = 0;
	
	
	
	//IMPORTANT CODE!!!! PAY ATTENTION!!!!
	/**************************************************************/
	public static boolean RECORDING_DYNAMIC_AUTONOMOUS = true; //It's not final because this value will change.
	public static final String DYNAMIC_AUTONOMOUS_RECORDING_PATH_FOLDER =
			"/home/lvuser/frc/dynamicauto/gostraight/v1";
	public static final boolean PLAYING_DYNAMIC_AUTONOMOUS = false;
	public static final String DYNAMIC_AUTONOMOUS_BASE_FOLDER = 
			"/home/lvuser/frc/dynamicauto";
	public static final String DYNAMIC_AUTONOMOUS_PLAY_VERSION =
			"v1";
	/**************************************************************/
	//IMPORTANT CODE!!!! PAY ATTENTION!!!!
	
	public static final String DYNAMIC_AUTONOMOUS_LEFT_DRIVE_FILE = "/leftdrive";
	public static final String DYNAMIC_AUTONOMOUS_RIGHT_DRIVE_FILE = "/rightdrive";
	
	
	
    //IterativeRobot has functions like the one below, hence the @Override.
	@Override
	public void robotInit()
	{
		//Initialize motors
		leftFront = new TalonSRX(RobotMap.LEFT_FRONT_TALON_SRX_CAN_DEVICE_ID);
		leftBack = new TalonSRX(RobotMap.LEFT_BACK_TALON_SRX_CAN_DEVICE_ID);
		//leftBack.follow(leftFront); //leftFront has an encoder
		
		//rightFront = new TalonSRX(RobotMap.RIGHT_FRONT_TALON_SRX_CAN_DEVICE_ID);
		//rightBack = new TalonSRX(RobotMap.RIGHT_BACK_TALON_SRX_CAN_DEVICE_ID);
		//rightBack.follow(rightFront); //rightFront has an encoder
		//elevatorMotorOne = new TalonSRX(RobotMap.LIFT_MOTOR_1);
		//elevatorMotorTwo = new TalonSRX(RobotMap.LIFT_MOTOR_2);
		
		/* Practice bot code */
		rightFront = new TalonSRX(RobotMap.RIGHT_FRONT_TALON_SRX_CAN_DEVICE_ID);
	    rightBack = new TalonSRX(RobotMap.RIGHT_BACK_TALON_SRX_CAN_DEVICE_ID);
	    //rightBack.follow(rightFront);
	    //rightFront.setInverted(true);
	    /*Practice bot code*/
	    
		rightArm = new Spark(RobotMap.RIGHT_ARM);
		leftArm = new Spark(RobotMap.LEFT_ARM);
		
		//intakeAngleMotor = new TalonSRX(RobotMap.INTAKE_ANGLE_MOTOR);
		intakeAngleMotor = new Spark(RobotMap.INTAKE_ANGLE_MOTOR);
		
		supportPiston = new Solenoid(RobotMap.PCM_MODULE, RobotMap.SUPPORT_PISTON);
		mainPistons = new DoubleSolenoid(RobotMap.PCM_MODULE, RobotMap.FORWARD_PISTON_PORT, RobotMap.REVERSE_PISTON_PORT);
		Compressor compressor =  new Compressor(RobotMap.PCM_MODULE);
		compressor.start();
		 
		//Initializing joystick
		driveController = new Joystick(RobotMap.DRIVE_STICK_PORT);
		auxController = new Joystick(RobotMap.AUX_STICK_PORT);
		
		//Give pigeonGyro value.
		//gyro = new PigeonIMU(rightBack); //the gyro is plugged into the rightBack motor controller.
		
		
		teleopDrive = new TeleopGameDrive();
		autoDrive = new AutoDrive();
		peripherals = new Actuators();
		
		dynamicAutonomousTimer = new Timer();
		dynamicAutonomousRightDrive = new LinkedHashMap<Double, Double>();
		dynamicAutonomousLeftDrive = new LinkedHashMap<Double, Double>();
		
		time = new Timer();
		
		UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture(0);
		UsbCamera backCamera = CameraServer.getInstance().startAutomaticCapture(1);
		

		rbSB = new StringBuilder();
		lbSB = new StringBuilder();
		lfSB = new StringBuilder();
		
		
		//Motion magic things: works as of Wednesday, 3/14
		/* first choose the sensor */
		//rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
		//kPIDLoopIdx, kTimeoutMs);
		//rightFront.setSensorPhase(true);
		/* Set relevant frame periods to be at least as fast as periodic rate*/
		//rightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
		//kTimeoutMs);
		//rightFront.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10,
		//kTimeoutMs);
		/* set the peak and nominal outputs */
		/*rightFront.configNominalOutputForward(0, kTimeoutMs);
		rightFront.configNominalOutputReverse(0, kTimeoutMs);
		rightFront.configPeakOutputForward(0.5, kTimeoutMs);
		rightFront.configPeakOutputReverse(-0.5, kTimeoutMs);*/
		/* set closed loop gains in slot0 - see documentation */
		/*rightFront.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		rightFront.config_kF(0, 0.2, kTimeoutMs);
		rightFront.config_kP(0, 0.2, kTimeoutMs);
		rightFront.config_kI(0, 0, kTimeoutMs);
		rightFront.config_kD(0, 0, kTimeoutMs);*/
		/* set acceleration and vcruise velocity - see documentation */
		/*rightFront.configMotionCruiseVelocity(15000, kTimeoutMs);
		rightFront.configMotionAcceleration(6000, kTimeoutMs);*/
		/* zero the sensor */
		//rightFront.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		
		
		//Position control things
		/* first choose the sensor */
		//rightFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kPIDLoopIdx, kTimeoutMs);
		//rightFront.setSensorPhase(true);
		/* set the peak and nominal outputs */
		//rightFront.configNominalOutputForward(0, kTimeoutMs);
		//rightFront.configNominalOutputReverse(0, kTimeoutMs);
		//rightFront.configPeakOutputForward(1, kTimeoutMs);
		//rightFront.configPeakOutputReverse(-1, kTimeoutMs);
		
		//rightFront.config_kF(0, 0.1, kTimeoutMs);
		//rightFront.config_kP(0, 0.1, kTimeoutMs);
		//rightFront.config_kI(0, 0, kTimeoutMs);
		//rightFront.config_kD(0, 0, kTimeoutMs);
		/* zero the sensor */
		//rightFront.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		
		
		
		
		
		/* first choose the sensor */
		/*rightBack.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidIdx, timeoutMs); //TODO: MAYBE CHANGE LAST TWO VALUES?
		/rightBack.setSelectedSensorPosition(0, pidIdx, timeoutMs);
		/rightBack.setInverted(false); //TODO: CHANGE?
		/* set the peak and nominal outputs, 12V means full */
		/*rightBack.configNominalOutputForward(0, timeoutMs);
		rightBack.configNominalOutputReverse(0, timeoutMs);
		rightBack.configPeakOutputForward(1, timeoutMs);
		rightBack.configPeakOutputReverse(-1, timeoutMs);
		/* set closed loop gains in slot0 - see documentation */
		/*rightBack.selectProfileSlot(slotIdx, pidIdx);
		rightBack.config_kF(slotIdx, 0, timeoutMs);
		rightBack.config_kP(slotIdx, 0, timeoutMs);
		rightBack.config_kI(slotIdx, 0, timeoutMs);
		rightBack.config_kD(slotIdx, 0, timeoutMs);
		rightBack.configMotionCruiseVelocity(0, timeoutMs);
		rightBack.configMotionAcceleration(0, timeoutMs);
		
		/* first choose the sensor */
		/*leftBack.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidIdx, timeoutMs); //TODO: MAYBE CHANGE LAST TWO VALUES?
		leftBack.setSelectedSensorPosition(0, pidIdx, timeoutMs);
		leftBack.setInverted(false); //TODO: CHANGE?
		/* set the peak and nominal outputs, 12V means full */
		/*leftBack.configNominalOutputForward(0, timeoutMs);
		leftBack.configNominalOutputReverse(0, timeoutMs);
		leftBack.configPeakOutputForward(1, timeoutMs);
		leftBack.configPeakOutputReverse(-1, timeoutMs);
		/* set closed loop gains in slot0 - see documentation */
		/*leftBack.selectProfileSlot(0, 0);
		leftBack.config_kF(slotIdx, 0, timeoutMs);
		leftBack.config_kP(slotIdx, 0, timeoutMs);
		leftBack.config_kI(slotIdx, 0, timeoutMs);
		leftBack.config_kD(slotIdx, 0, timeoutMs);
		leftBack.configMotionCruiseVelocity(0, timeoutMs);
		leftBack.configMotionAcceleration(0, timeoutMs);
		
		/* first choose the sensor */
		/*leftFront.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, pidIdx, timeoutMs); //TODO: MAYBE CHANGE LAST TWO VALUES?
		leftFront.setInverted(false); //TODO: CHANGE?
		/* set the peak and nominal outputs, 12V means full */
		/*leftFront.configNominalOutputForward(0, timeoutMs);
		leftFront.configNominalOutputReverse(0, timeoutMs);
		leftFront.configPeakOutputForward(1, timeoutMs);
		leftFront.configPeakOutputReverse(-1, timeoutMs);
		/* set closed loop gains in slot0 - see documentation */
		/*leftFront.selectProfileSlot(0, 0);
		leftFront.config_kF(slotIdx, 0, timeoutMs);
		leftFront.config_kP(slotIdx, 0, timeoutMs);
		leftFront.config_kI(slotIdx, 0, timeoutMs);
		leftFront.config_kD(slotIdx, 0, timeoutMs);
		leftFront.configMotionCruiseVelocity(0, timeoutMs);
		leftFront.configMotionAcceleration(0, timeoutMs);*/
	}

	@Override
	public void autonomousInit()
	{
		autoDrive.init();
		time.reset();
		time.start();
	}

	@Override
	public void autonomousPeriodic()
	{
		/*if (getYaw() < 100 && getYaw() > 80)
		{
			leftFront.set(ControlMode.PercentOutput, 0);
			leftBack.set(ControlMode.PercentOutput, 0);
			rightFront.set(ControlMode.PercentOutput, 0);
		}
		else
		{
			leftFront.set(ControlMode.PercentOutput, 0.15);
			leftBack.set(ControlMode.PercentOutput, 0.15);
			rightFront.set(ControlMode.PercentOutput, 0.15);
		}*/
		//autoDrive.periodic();
		rightFront.set(ControlMode.PercentOutput, 0.5);
		
		//Competition autonomous
		/*if (time.get() < 1.5)
		{
			leftFront.set(ControlMode.PercentOutput, 0.5);
			leftBack.set(ControlMode.PercentOutput, 0.5);
			rightFront.set(ControlMode.PercentOutput, -0.5);
			rightBack.set(ControlMode.PercentOutput, -0.5);
		}
		else
		{
			leftFront.set(ControlMode.PercentOutput, 0);
			leftBack.set(ControlMode.PercentOutput, 0);
			rightFront.set(ControlMode.PercentOutput, 0);
			rightBack.set(ControlMode.PercentOutput, 0);
		}*/
		
		//Motion magic testing: works as of Wednesday, 3/14
		/*int pos = rightFront.getSensorCollection().getQuadraturePosition();
		if (false)
		{
			if (pos > 1000)
				rightFront.set(ControlMode.PercentOutput, -0.5);
			else if (pos < -1000)
				rightFront.set(ControlMode.PercentOutput, 0.5);
			else
				rightFront.set(ControlMode.PercentOutput, 0);
		}
		else
			rightFront.set(ControlMode.MotionMagic, 10000);
		System.out.println(pos);*/
		
		
		//Position control testing:
		int pos = rightFront.getSensorCollection().getQuadraturePosition();
		if (true)
		{
			if (pos > 1000)
				rightFront.set(ControlMode.PercentOutput, -0.5);
			else if (pos < -1000)
				rightFront.set(ControlMode.PercentOutput, 0.5);
			else
				rightFront.set(ControlMode.PercentOutput, 0);
		}
		else
			rightFront.set(ControlMode.Position, 10000);
		System.out.println(pos);	
		
		
	}

	@Override
	public void teleopInit()
	{
		teleopDrive.init();
	}
	
	@Override
	public void teleopPeriodic()
	{
		teleopDrive.periodic();
		peripherals.periodic();
		
		//SmartDashboard.putNumber("elevatorMotorQuadPos", Robot.elevatorMotorOne.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("rightDriveQuadPos", Robot.rightFront.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("leftDriveQuadPos", Robot.leftFront.getSensorCollection().getQuadraturePosition());
		//SmartDashboard.putNumber("intakeAngleMotorQuadPos", Robot.intakeAngleMotor.getSensorCollection().getQuadraturePosition());
		//SmartDashboard.putNumber("gyro", getRawYaw());
		/*
		double leftYStick = -1.0 * driveController.getRawAxis(1);
		
		double rbMotorOutput = rightBack.getMotorOutputVoltage() / rightBack.getBusVoltage();
		rbSB.append("\tRB_out:");
		rbSB.append(rbMotorOutput);
		rbSB.append("\tRB_spd:");
		rbSB.append(rightBack.getSelectedSensorVelocity(pidIdx));

		double lbMotorOutput = leftBack.getMotorOutputVoltage() / leftBack.getBusVoltage();
		lbSB.append("\tLB_out:");
		lbSB.append(lbMotorOutput);
		lbSB.append("\tLB_spd:");
		lbSB.append(leftBack.getSelectedSensorVelocity(pidIdx));
		
		double lfMotorOutput = leftFront.getMotorOutputVoltage() / leftFront.getBusVoltage();
		lfSB.append("\tLF_out:");
		lfSB.append(lfMotorOutput);
		lfSB.append("\tLF_spd:");
		lfSB.append(leftFront.getSelectedSensorVelocity(pidIdx));
		
		
		if (driveController.getRawButton(1))
		{
			/* Motion Magic */
			//double targetPos = leftYStick * 10.0; /* 10 Rotations in either direction */
			/*
			rightBack.set(ControlMode.MotionMagic, targetPos);
			/* append more signals to print when in speed mode. */
			//rbSB.append("\tRB_err:");
			//rbSB.append(rightBack.getClosedLoopError(pidIdx));
			//rbSB.append("\tRB_trg:");
			//rbSB.append(targetPos);

			//leftBack.set(ControlMode.MotionMagic, targetPos);
			/* append more signals to print when in speed mode. */
			//lbSB.append("\tLB_err:");
			//lbSB.append(leftBack.getClosedLoopError(pidIdx));
			//lbSB.append("\tLB_trg:");
			//lbSB.append(targetPos);
			
			/*leftFront.set(ControlMode.MotionMagic, targetPos);
			/* append more signals to print when in speed mode. */
			/*lfSB.append("\tLF_err:");
			lfSB.append(leftFront.getClosedLoopError(pidIdx));
			lfSB.append("\tLF_trg:");
			lfSB.append(targetPos);*/
		
		//}
		/*
		else
		{
			/* Percent voltage/output mode (normal mode) */
			//rightBack.set(ControlMode.PercentOutput, leftYStick);
			//leftBack.set(ControlMode.PercentOutput, leftYStick);
			//leftFront.set(ControlMode.PercentOutput, leftYStick);
		//}
		
		/* PROCESSING DATA */
		/* smart dash plots */
		/*
	    SmartDashboard.putNumber("RB_RPM", rightBack.getSelectedSensorVelocity(pidIdx));
	    SmartDashboard.putNumber("RB_Pos",  rightBack.getSelectedSensorPosition(pidIdx));
	    SmartDashboard.putNumber("RB_AppliedThrottle",  (rightBack.getMotorOutputVoltage()/rightBack.getBusVoltage())*1023);
	    SmartDashboard.putNumber("RB_ClosedLoopError", rightBack.getClosedLoopError(pidIdx));
	    
	    SmartDashboard.putNumber("LB_RPM", leftBack.getSelectedSensorVelocity(pidIdx));
	    SmartDashboard.putNumber("LB_Pos",  leftBack.getSelectedSensorPosition(pidIdx));
	    SmartDashboard.putNumber("LB_AppliedThrottle",  (leftBack.getMotorOutputVoltage()/leftBack.getBusVoltage())*1023);
	    SmartDashboard.putNumber("LB_ClosedLoopError", leftBack.getClosedLoopError(pidIdx));
	    */
	    /*SmartDashboard.putNumber("LF_RPM", leftFront.getSelectedSensorVelocity(pidIdx));
	    SmartDashboard.putNumber("LF_Pos",  leftFront.getSelectedSensorPosition(pidIdx));
	    SmartDashboard.putNumber("LF_AppliedThrottle",  (leftFront.getMotorOutputVoltage()/leftFront.getBusVoltage())*1023);
	    SmartDashboard.putNumber("LF_ClosedLoopError", leftFront.getClosedLoopError(pidIdx));*/
	    
	    /*if (rightBack.getControlMode() == ControlMode.MotionMagic) {
			//These API calls will be added in our next release.
	    	//SmartDashboard.putNumber("ActTrajVelocity", tal.getMotionMagicActTrajVelocity());
	    	//SmartDashboard.putNumber("ActTrajPosition", tal.getMotionMagicActTrajPosition());
	    }*/
	    
	    
	    /* periodically print to console */
	    /*if(++loops >= 10)
	    {
	        loops = 0;
	        System.out.println(rbSB.toString());
	        System.out.println(lbSB.toString());
	        //System.out.println(lfSB.toString());
	    }
	    /* clear line cache */
	    //rbSB.setLength(0);
	    //lbSB.setLength(0);
	    //lfSB.setLength(0);
		
	}
	
	/*public static double getYaw()
	{
		/*
		 * The yaw returned by the gyro goes negative when turning clockwise (right),
		 * positive when turning counterclockwise (left).
		 * However, the yaw returned does not return a value from -180 to +180.
		 * Rather, the yaw returned continues to build past 180 and -180.
		 * For example, if the robot turns 2 perfect revolutions,
		 * the gyro does not return 0, but rather returns -720.
		 * Not terribly useful.
		 * As such, this function processes the yaw input to return a value
		 * from -180 to +180, with negative being clockwise, and positive being counterclockwise.
		 */
		
		/*double rawYaw = getRawYaw();
		rawYaw %= 360;
		if (Math.abs(rawYaw) <= 180)
			return rawYaw;
		else
		{
			if (rawYaw < 0)
				return 360 + rawYaw;
			else
				return rawYaw - 360;
		}
		
		
			
	}*/
	
	/*public static double getRawYaw()
	{
		double [] ypr = new double[3];
		gyro.getYawPitchRoll(ypr);
		return ypr[0];
	}*/
	
	private static void addData(HashMap<Double, Double> map, double time, double input)
	{
		//TODO: This could mess up (if ifAbsent doesn't work, as the new Double() procedure may cause different addresses with the same value.
		map.putIfAbsent(new Double(time), new Double(input));
	}
	public static void addLeftDriveData(double time, double input) { addData(dynamicAutonomousLeftDrive, time, input); }
	public static void addRightDriveData(double time, double input) { addData(dynamicAutonomousRightDrive, time, input); }
	
	
	public static void recordData(LinkedHashMap<Double, Double> map, String filePath)
	{
	    try 
	    {
	    	FileOutputStream fileOut =
	    	//new FileOutputStream("C:\\Users\\Josue\\eclipse-workspace\\Dynamic Autonomous Testing\\src\\data.ser");
	    	new FileOutputStream(filePath);
	        ObjectOutputStream out = new ObjectOutputStream(fileOut);
	        out.writeObject(map);
	        out.close();
	        fileOut.close();
	        System.out.printf("Serialized data is saved.");
	    }
	    catch (IOException i)  { i.printStackTrace(); }
	}
	public static void recordLeftDriveData(String filePath) { recordData(dynamicAutonomousLeftDrive, filePath); }
	public static void recordRightDriveData(String filePath) { recordData(dynamicAutonomousRightDrive, filePath); }
	
	
	public static LinkedHashMap<Double, Double> readData(String filePath)
	{
		try
		{
			FileInputStream fileIn = //new FileInputStream("C:\\Users\\Josue\\eclipse-workspace\\Dynamic Autonomous Testing\\src\\data.ser");
			new FileInputStream(filePath);
	        ObjectInputStream in = new ObjectInputStream(fileIn);
	        LinkedHashMap<Double, Double> data = (LinkedHashMap<Double, Double>) in.readObject();
	        in.close();
	        fileIn.close();
	        return data;
	    }
		catch (IOException i)
		{
	        i.printStackTrace();
	        return null;
	    } 
		catch (ClassNotFoundException c)
		{
	        System.out.println("Class not found");
	        c.printStackTrace();
	        return null;
	    }
	}

	
	
	@Override
	public void testPeriodic()
	{
	}
}
