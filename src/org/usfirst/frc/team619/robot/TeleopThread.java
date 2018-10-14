package org.usfirst.frc.team619.robot;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import java.util.ArrayList;

//import org.usfirst.frc.team619.subsystems.GripPipeline;

public class TeleopThread extends RobotThread {
	
	int currentAngle, targetAngle;
	double[] speeds = new double[4];
	
	double steer_value;
	
	public final double L = 6.35;
	public final double W = 9.525;
	
	double targetHeading;
	
	//drive
	private XboxController drive = new XboxController(0);
	
	//secondary
	private XboxController secondary = new XboxController(1);
	
	//Manipulators
	TalonSRX lift1;
	TalonSRX lift2;
	TalonSRX lift3;
	
	TalonSRX intakeLeft;
	TalonSRX intakeRight;
	
	//NAVX
	AHRS imu;
	
	//modes
	boolean isRobotCentric;
	boolean isFieldCentric;
	
	//PID VARS
	//pid values
	double kP;
	double kI;
	double kD;
	
	double theta;
	double mag;
	
	double speed;
	
	SwerveDriveBase driveBase;
	int scalePercent;
	
	boolean alreadyPressed;
	
	//Manipulators
	Lift lift;
	Intake intake;
	Ramp ramp;

	//Deadzone Constants
	final double MOVEDEADZONE = 0.06;
	final double ROTATEDEADZONE = 0.25;
	
	
	/**
	 * Constructor for teleop thread
	 * @param period - threadManager variable
	 * @param threadManager - threadManager object
	 * 
	 * @param backRight - wheelDrive object for back right
	 * @param backLeft - wheelDrive object for back left
	 * @param frontRight - wheelDrive object for front right
	 * @param frontLeft - wheelDrive object for front left
	 * 
	 * @param l1 - variable index for lift 1
	 * @param l2 - variable index for lift 2
	 * @param l3 - variable index for lift 3
	 * 
	 * @param il - variable index for left intake
	 * @param ir - variable index for right intake
	 * 
	 * @param iSwitch LimitSwitch object for limit switch
	 */	
	public TeleopThread(int period, ThreadManager threadManager, WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, 
						WheelDrive frontLeft, Lift l, Intake i, Ramp r) {
		super(period, threadManager);
		
		//initialize manipulators
		lift = l;
		intake = i;
		ramp = r;
		imu = new AHRS(SPI.Port.kMXP);
		driveBase = new SwerveDriveBase(backRight, backLeft, frontRight, frontLeft);
		targetHeading = imu.getAngle();
		
		isRobotCentric = true;
		isFieldCentric = false;
		alreadyPressed = false;
		
		scalePercent = 10;
		
		start();
	}
	
	/**
	 * refreshes inputs continuously
	 */
	@Override
	protected void cycle() {
	
		double xAxis = deadzone(drive.getX(Hand.kRight));
        double yAxis = deadzone(drive.getY(Hand.kRight));
        double zTurn = deadzoneRotate(drive.getX(Hand.kLeft));
        
        XboxController.ButtonID buttonPressedPrimary = drive.getButtonPressed();
        XboxController.ButtonID buttonPressedSecondary = secondary.getButtonPressed();
        
    	/*
		 * Driver Controller
		 * Functions:
		 * Right Bumper - 20% Speed
		 * Both Bumpers - Turbo (100% Speed)
		 * A Button - Robot Centric
		 * X Button - Field Centric
		 * Y Button - Reset Gyro
		 * Right Joystick - Turn
		 * Left Joystick - Drive
		 * Dpad (Up/Down) - Speed Change (10% or -10%)

		 */
        switch(buttonPressedPrimary)
        {
        	case RIGHTBUMPER:
        		xAxis *= 0.2;
            	yAxis *= 0.2;
            	zTurn *= 0.4;
        		break;
        	case LEFTBUMPER:
        		break;
        	case ABUTTON:
        		setRobotCentric();
        		break;
        	case BBUTTON:
        		break;
        	case XBUTTON:
        		setFieldCentric();
        		break;
        	case YBUTTON:
        		imu.zeroYaw();
        		break;
        	case RIGHTSTICKBUTTON:
        		break;
        	case LEFTSTICKBUTTON:
        		break;
        	case STARTBUTTON:
        		break;
        	case BACKBUTTON:
        		break;
        	case POVUP:
        		changeSpeed(1);
        		break;
        	case POVDOWN:
        		changeSpeed(-1);
        		break;
        	default:
        		//add anything if needed
        		break;
        }
        
        /*
         * Secondary Controller
         * Functions:
         * A - Ramp Release
         * B - Intake In
         * X - Set Ramp
         * Y - Intake Out
         * DPad (Up/Down) - Raise/Lower Lift
         */
        switch(buttonPressedSecondary)
        {
        	case RIGHTBUMPER:
        		break;
        	case LEFTBUMPER:
        		break;
        	case ABUTTON:
        		ramp.windDown();
        		break;
        	case BBUTTON:
        		intake.moveIntake(1);
        		break;
        	case XBUTTON:
        		ramp.windUp();
        		break;
        	case YBUTTON:
        		intake.moveIntake(-1);
        		break;
        	case RIGHTSTICKBUTTON:
        		break;
        	case LEFTSTICKBUTTON:
        		break;
        	case STARTBUTTON:
        		break;
        	case BACKBUTTON:
        		break;
        	case POVUP:
        		lift.moveLift(1);
        		break;
        	case POVDOWN:
        		lift.moveLift(-1);
        		break;
        	default:
        		lift.stopLift();
        		intake.stopIntake();
        		break;
        }

        System.out.println("xAxis: " + xAxis);
        //move robot from axis values on primary controller
        move(xAxis * scalePercent*0.1, yAxis * scalePercent*0.1, zTurn*0.7*scalePercent*0.1);
	}
	
	/**
	 * calls driveBase to move swerve modules
	 * @param x1 - double x axis of right joystick
	 * @param y1 - double y axis of right joystick
	 * @param x2 - double x axis of left joystick
	 */
	public void move(double x1, double y1, double x2)
	{
		if(isRobotCentric){
        	driveBase.drive(x1, -y1, x2);
        } else if(isFieldCentric){
        	//System.out.println("isFieldCentric");
            driveBase.getFieldCentric(x1, -y1, x2);
        }
	}
	
	/**
	 * moves the lift up or down
	 * @param dir - 1 up, -1 down
	 */
	public void changeSpeed(int dir) 
	{
		switch(dir) 
		{
			case 1:
				if(!(scalePercent == 10) && !alreadyPressed)
            	{
            		scalePercent += 1;
            		alreadyPressed = true;
            	}
        		else
        		{
        			alreadyPressed = false;
        		}
				break;
			case -1:
        		if(!(scalePercent == 0) && !alreadyPressed)
            	{
            		scalePercent -= 1;
            		alreadyPressed = true;
            	}
        		else
        		{
        			alreadyPressed = false;
        		}
        		break;
		}
	}
	
	//states
	public void setFieldCentric()
	{
		isFieldCentric = true;
		isRobotCentric = false;
	}
	
	public void setRobotCentric()
	{
		isFieldCentric = false;
		isRobotCentric = true;
	}
	
	/**
	 * sets val below MOVEDEADZONE to zero
	 * @param val - variable input
	 * @return
	 */
	private double deadzone(double val) {
		if(Math.abs(val) < MOVEDEADZONE)
			return 0;
		return val;
	}
	
	/**
	 * sets val below ROTATEDEADZONE to zero
	 * @param val - variable input
	 * @return
	 */
	private double deadzoneRotate(double val) {
		if(Math.abs(val) < ROTATEDEADZONE)
			return 0;
		return val;
	}
	
	/**
	 * Delays thread in milliseconds
	 * @param milliseconds - variable time to delay in ms 
	 */
    public void delay(int milliseconds){
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
	public void currentLimit(TalonSRX talon, int continuousLimit, int currentLimit) {
		talon.configContinuousCurrentLimit(continuousLimit, 0);
		talon.configPeakCurrentLimit(currentLimit, 0);
		talon.configPeakCurrentDuration(100, 0);
		talon.enableCurrentLimit(true);

	}
}
