package org.usfirst.frc.team619.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class WheelDrive {
	
	private final double MAX_VOLTS = 4.95;
	
	public TalonSRX angleMotor;
	public TalonSRX speedMotor;
	private PIDController pidController;
	public static final double p=0.3, i=0.0001, d=139;
	private double encoderUnitsPerRotation = 1660; // using old swerve. for new swerve: 22235.4285714 76/14 ratio 4096(encoder ticks per spin)
	private double targetAngle = 0;
	private double driveSpeed = 0;
	
	public int rotAngle;
	
	boolean stopRotating = false;
	public WheelDrive(int angleMotor, int speedMotor, int rangle)
	{
		this.angleMotor = new TalonSRX(angleMotor);
		this.speedMotor = new TalonSRX(speedMotor);
		
		this.angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10); //new swerve has absolute
		
		//NEVER UNCOMMENT THIS LINE UNLESS YOU WANT TO RESET ENCODER POSITION WHERE IT IS WHEN ROBOT TURNS ON
		this.angleMotor.setSelectedSensorPosition(0, 0, 0);
		
		this.angleMotor.configNominalOutputForward(0, 10);
		this.angleMotor.configNominalOutputReverse(0, 10);
		this.angleMotor.configPeakOutputForward(1.0, 10);
		this.angleMotor.configPeakOutputReverse(-1.0, 10);
		
		this.angleMotor.setInverted(false); //used to be false
		this.angleMotor.setSensorPhase(true);
		
		this.speedMotor.setInverted(false);
		
		rotAngle = rangle;
		
		//this.angleMotor.configAllowableClosedloopError(0, 0, 10);
		
		//new serve PID values
//		this.angleMotor.config_kP(0, 0.5, 10);
//		this.angleMotor.config_kI(0, 0, 10);
//		this.angleMotor.config_kD(0, 10, 10);

		this.angleMotor.config_kP(5, 0.0001, 0);
		this.angleMotor.config_kI(5, 0.0001, 0);
		this.angleMotor.config_kD(5, 0.0001, 0);
		
		currentLimit(this.angleMotor, 10, 15);
		currentLimit(this.speedMotor, 35, 40);
	}
	
	//DRIVE METHODS
	public void drive()
	{
		speedMotor.set(ControlMode.PercentOutput, driveSpeed);
	}
	
	public void setDriveSpeed(double speed) {
		driveSpeed = speed;
	}
	
	public void rotate()
	{
		setTargetAngle(rotAngle);
	}
	
	public double getDriveSpeed()
	{
		return driveSpeed;
	}
	
	public void stop()
	{
		speedMotor.set(ControlMode.PercentOutput, 0);
	}

	public void goToAngle(){
		angleMotor.set(ControlMode.Position, angleMotor.getSelectedSensorPosition(0) + angleToEncoderUnit(getDeltaTheta()));
	}
	
	public int angleToEncoderUnit(double angle)
	{
		double deltaEncoder;
		deltaEncoder = angle*(encoderUnitsPerRotation/360.0);
		
		return (int)deltaEncoder;
	}
	
	public int getClosedLoopTarget()
	{
		return angleMotor.getClosedLoopTarget(0);
	}
	
	public int getOutput()
	{
		return encoderUnitToAngle(angleMotor.getClosedLoopError(0));
	}
	
	//ANGLE METHODS
	public int getTargetAngle()
	{
		return (int)(targetAngle);
	}

	public int getCurrentAngle()
	{
		return encoderUnitToAngle(angleMotor.getSelectedSensorPosition(0));
	}
	
	public double getDeltaTheta(){
		double deltaTheta = getTargetAngle() - getCurrentAngle();
		
		while ((deltaTheta < -90) || (deltaTheta > 90)){
//			if ( label.equals("rotateMotor") )
//				System.out.println( "                          --> " + deltaTheta + "/" + speed );
			if(deltaTheta > 90){
				deltaTheta -= 180;
				driveSpeed *= -1;
			}else if(deltaTheta < -90){
				deltaTheta += 180;
				driveSpeed *= -1;
			}
		}
//		if ( label.equals("rotateMotor") )
//			System.out.println( "                          --> " + deltaTheta + "/" + speed );
		
		return deltaTheta;		
	}
	
	private int encoderUnitToAngle(double e)
	{
		double angle = 0;
		if(e >= 0)
		{
			angle = (e * (360.0/encoderUnitsPerRotation));
			angle = angle % 360;
		}
		else if(e < 0) {
			angle = (e * (360.0/encoderUnitsPerRotation));
			angle = angle % 360 + 360;
		}
		return (int)angle;
	}
	
	public void setTargetAngle(double angle)
	{
		if(angle < 0){
			angle += 360;
		}else if(angle >=360){
			angle -= 360;
		}
		targetAngle = (int)angle;
	}

	//get talons
	public TalonSRX getRotateTalon()
	{
		return angleMotor;
	}
	
	public TalonSRX getDriveTalon()
	{
		return speedMotor;
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
