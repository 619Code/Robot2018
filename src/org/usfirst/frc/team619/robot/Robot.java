/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	ThreadManager threadManager;
	TeleopThread swerveThread;
	
	AutoThread autoThread;
	
	//talons
	//drive id: 4  turn id: 9
	private WheelDrive backRight, backLeft, frontRight, frontLeft;
	WheelDrive[] wheels;
	
	Lift lift;
	TalonSRX lift1, lift2, lift3;
	final int lowerCurrentLimitLift = 20;
	final int upperCurrentLimitLift = 25;
	
	Intake intake;
	TalonSRX intakeRight, intakeLeft;
	
	Ramp ramp;
	TalonSRX rampRelease;
	
	//Limit switches
	LimitSwitch intakeSwitch;
	
	//auto switches
	LimitSwitch[] autoSwitches = new LimitSwitch[4];
	
	AHRS imu;
	
	AnalogUltrasonic[] ultrasonics = new AnalogUltrasonic[2];
	
	//talon constants
	final int BRDRIVEID = 5;
	final int BRANGLEID = 4;
	final int BLDRIVEID = 1;
	final int BLANGLEID = 0;
	final int FRDRIVEID = 7;
	final int FRANGLEID = 6;
	final int FLDRIVEID = 3;
	final int FLANGLEID = 2;
	final int ROTATIONOFFSET = 45;
	final int FIRSTLIFTID = 11;
	final int SECONDLIFTID = 12;
	final int THIRDLIFTID = 13;
	final int INTAKELEFTID = 10;
	final int INTAKERIGHTID = 14;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() 
	{
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		initializeAll();
	}
	
	public void initializeAll()
	{
		//initialize wheel drives
		backRight = new WheelDrive(BRANGLEID, BRDRIVEID, ROTATIONOFFSET*5);
		backLeft = new WheelDrive(BLANGLEID, BLDRIVEID, ROTATIONOFFSET*3);
		frontRight = new WheelDrive(FRANGLEID, FRDRIVEID, ROTATIONOFFSET*7);
		frontLeft = new WheelDrive(FLANGLEID, FLDRIVEID, ROTATIONOFFSET);
		wheels = new WheelDrive[]{backRight, backLeft, frontRight, frontLeft};
		
		//initialize lift
		lift1 = new TalonSRX(FIRSTLIFTID);
		lift2 = new TalonSRX(SECONDLIFTID);
		lift3 = new TalonSRX(THIRDLIFTID);
		currentLimit(lift1, lowerCurrentLimitLift, upperCurrentLimitLift);
		currentLimit(lift2, lowerCurrentLimitLift, upperCurrentLimitLift);
		currentLimit(lift3, lowerCurrentLimitLift, upperCurrentLimitLift);
		lift = new Lift(lift1, lift2, lift3);
		
		//initialize intakeSwitch
		intakeSwitch = new LimitSwitch(0);
		
		//initialize all four autoSwitch and feed them into the autoSwitches array
		for(int i = 1; i <= 4; i++)
		{
			LimitSwitch autoSwitch = new LimitSwitch(i);
			autoSwitches[i-1] = autoSwitch;
		}
		
		//initialize both ultrasonics
		ultrasonics[0] = new AnalogUltrasonic(0);
		ultrasonics[1] = new AnalogUltrasonic(2);
		
		//initialize imu
		imu = new AHRS(SPI.Port.kMXP);

		//initialize intake
		intakeLeft = new TalonSRX(INTAKELEFTID);
		intakeRight = new TalonSRX(INTAKERIGHTID);
//		currentLimit(intakeLeft, 25, 30);
//		currentLimit(intakeRight, 25, 30);
		intake = new Intake(intakeLeft, intakeRight, intakeSwitch);
		
		//Thread Manager
		threadManager = new ThreadManager();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() 
	{
		threadManager.killAllThreads();
		
		testDrive(frontRight, 0.5);
		testDrive(frontLeft, 0.5);
		testDrive(backRight, 0.5);
		testDrive(backLeft, 0.5);
		
		testRotate(frontRight);
		testRotate(frontLeft);
		testRotate(backRight);
		testRotate(backLeft);
		
		//autoThread = new AutoThread(0, threadManager, backRight, backLeft, frontRight, frontLeft, lift, intake, autoSwitches, ultrasonics);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() 
	{
	}
	
	/**
	 * starts teleop thread
	 */
	@Override
	public void teleopInit()
	{
		threadManager.killAllThreads();
		swerveThread = new TeleopThread(0, threadManager, backRight, backLeft, frontRight, frontLeft, lift, intake, ramp);
	}
	  
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() 
	{
	}

	/**
	 * This function is called initially during test mode.
	 */
	@Override
	public void testInit() 
	{
		threadManager.killAllThreads();
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() 
	{
	}
	
	@Override
	public void disabledInit()
	{
		System.out.println("YOINK");
		threadManager.killAllThreads();
	}
	
    /**
	 * Delays the thread for a certain amount of milliseconds
	 * @param milliseconds - time in milliseconds
	 */
	private void delay(int milliseconds)
    {
    	try {
    		Thread.sleep(milliseconds);
    	} catch(InterruptedException e) {
    		Thread.currentThread().interrupt();
    	}
    }
    
	/**
	 * Limits current to selected motor
	 * @param talon - TalonSRX object
	 * @param continousLimit - variable limit of current
	 * @param currentLimit - variable limit of peak
	 */
	private void currentLimit(TalonSRX talon, int continuousLimit, int currentLimit) 
	{
		talon.configContinuousCurrentLimit(continuousLimit, 0);
		talon.configPeakCurrentLimit(currentLimit, 0);
		talon.configPeakCurrentDuration(100, 0);
		talon.enableCurrentLimit(true);

	}
	
	///--------------------------------------------------
	/// Debug functions
	///--------------------------------------------------
    
    public void testDrive(WheelDrive wheel, double speed)
    {
    	wheel.setDriveSpeed(speed);
    	wheel.drive();
		delay(1000);
		wheel.stop();
		delay(1000);
		wheel.setDriveSpeed(-speed);
		wheel.drive();
		delay(1000);
		wheel.stop();
    }
	    
    public void testRotate(WheelDrive wheel)
    {
    	wheel.setTargetAngle(90);
    	wheel.goToAngle();
    	System.out.println("goto: " + wheel.getTargetAngle());
    	delay(3000);
    	wheel.setTargetAngle(0);
    	wheel.goToAngle();
    	delay(3000);
    }
    
    public void calibrate(WheelDrive wheel)
    {
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, 1);
    	delay(5000);
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, 0);
    	delay(1000);
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, -1);
    	delay(5000);
    	wheel.getRotateTalon().set(ControlMode.PercentOutput, 0);
    }

    
}
