/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6704.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.*;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot implements PIDOutput {
	
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	private Spark trMotor; //Top Right Drive Motor
	private Spark tlMotor; //Top Left Drive Motor
	private Spark brMotor; //Bottom Right Drive Motor
	private Spark blMotor; //Bottom Left Drive Motor

	private SpeedControllerGroup mLeft;
	private SpeedControllerGroup mRight;
	private DifferentialDrive drive;

	private Encoder rEncoder;
	private Encoder lEncoder;
	
	private double kP = 0.00;
	private double kI = 0.00;
	private double kD = 0.00;
	private double kF = 0.00;
	private PIDController rDistanceController;
	private PIDController lDistanceController;

	private double rSpeed;
	private double lSpeed;

	private Victor arm; //Arm Motor
	private Victor rWinch; //Right Winch Motor
	private Victor lWinch; //Left Winch Motor

	private boolean scissor;

	private Solenoid clawOpen; //Solenoid for opening claw
	private Solenoid clawClose; //Solenoid for closing claw
	private Solenoid pusherOpen;
	private Solenoid pusherClose;
	private Solenoid scissorOpen; //Solenoid for opening scissor lift
	private Solenoid scissorClose; //Solenoid for closing scissor lift

	private Hand hand;
	private static XboxController controller;
	private Joystick stick;
	private boolean isClosed;
	private DigitalInput limitOne;
	private DigitalInput limitTwo;
	private int clawSeq;
	private boolean toBePushed;
	private Timer timed;
	private String AutoChoose;
	private String turning;
	private String gameFieldData;
	
	AHRS ahrs;

    PIDController turnController;
    double rotateToAngleRate;
	final double kToleranceDegrees = 2.0f;    
	final double kTargetAngleDegrees = 90.0f;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		rotateToAngleRate = 0;
		
		
		   try {
				/***********************************************************************
				 * navX-MXP:
				 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
				 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * navX-Micro:
				 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
				 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
				 * 
				 * Multiple navX-model devices on a single robot are supported.
				 ************************************************************************/
	            ahrs = new AHRS(SPI.Port.kMXP); 
	        } catch (RuntimeException ex ) {
	            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
	        }
		
		turnController = new PIDController(kP, kI, kD, kF, ahrs,this);
		turnController.setInputRange(-180.0f,180.0f);
		turnController.setOutputRange(-1.0,1.0);
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.disable();
		
		
		
		

		new Thread(() -> {
            UsbCamera usbCam = CameraServer.getInstance().startAutomaticCapture();
            usbCam.setResolution(640, 480);
            //Kinect class -> not done yet. Will get too it.
            //Kinect kinect = new Kinect();
        }).start();

		timed = new Timer();

		trMotor = new Spark(0); //Top right motor
		brMotor = new Spark(2); //Back right motor
		mRight = new SpeedControllerGroup(trMotor, brMotor); //Matches values for right speed controllers

		tlMotor = new Spark(1); //Top left motor
		blMotor = new Spark(3); //Back left motor
		mLeft = new SpeedControllerGroup(tlMotor, blMotor); //Matches values for left speed controllers

		drive = new DifferentialDrive(mLeft, mRight); //Tank drive object

		rEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X); //Right encoder
		rEncoder.setDistancePerPulse(Math.PI/(4096)); //Sets distance in feet per pulse using 2048 pulses per revolution scale factor
		rEncoder.setPIDSourceType(PIDSourceType.kDisplacement); //Sets PIDSource as displacement for PID loop
		
		lEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X); //Left encoder
		lEncoder.setDistancePerPulse(Math.PI/(4096)); //Sets distance in feet per pulse using 2048 pulses  per revolution scale factor
		lEncoder.setPIDSourceType(PIDSourceType.kDisplacement); //Sets PIDSource as displacement for PID loop

		arm = new Victor(4); //Arm motor
		rWinch= new Victor(5); //Right winch motor
		lWinch=new Victor(6); //Left winch motor

		pusherOpen = new Solenoid(0);
		pusherClose = new Solenoid(1);
		clawOpen = new Solenoid(2);
		clawClose = new Solenoid(3);
		scissorOpen = new Solenoid(4);
		scissorClose = new Solenoid(5);

		limitOne = new DigitalInput(0);
		limitTwo = new DigitalInput(1);

		stick = new Joystick(1);
		controller = new XboxController(0);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = 0;

		gameFieldData = "";
		
		scissorOpen.set(true);
		scissorClose.set(false);
		AutoChoose = "";
		turning = "";
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
	public void disabledInit() {
		timed.stop();
		timed.reset();
		lEncoder.reset();
		rEncoder.reset();
		scissorOpen.set(true);
		scissorClose.set(false);
		pusherOpen.set(false);
		pusherClose.set(true);
		clawOpen.set(false);
		clawClose.set(true);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = 0;
	}
	
	@Override
	public void disabledPeriodic() {
//		SmartDashboard.putString("Value of the thingy", gameFieldData.charAt(0) + " ");
		if(stick.getTrigger()) {
			AutoChoose = "Middle";
		}
		if(stick.getRawButton(3)) {
			AutoChoose = "Left";
		}
		if(stick.getRawButton(4)) {
			AutoChoose = "Right";
		}
		SmartDashboard.putString("Autonomous Selecting", AutoChoose);
		if(controller.getBackButton()) {
			SmartDashboard.putBoolean("Start Button", controller.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		}else if(controller.getStartButton()) {
 			SmartDashboard.putBoolean("Start Button", controller.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		}
 		else {
 			SmartDashboard.putBoolean("Start Button", controller.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		}
		SmartDashboard.updateValues();
	}
	@Override
	public void autonomousInit() {
		lEncoder.reset();
		rEncoder.reset();
		scissorOpen.set(true);
		scissorClose.set(false);
//		pusherOpen.set(false);
//		pusherClose.set(true);
//		clawOpen.set(false);
//		clawClose.set(true);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = 0;
		
		rEncoder.reset();
		lEncoder.reset();
		rDistanceController = new PIDController(kP, kI, kD, kF, rEncoder, mRight);
		rDistanceController.setPercentTolerance(0.02);
		lDistanceController = new PIDController(kP, kI, kD, kF, lEncoder, mLeft);
		lDistanceController.setPercentTolerance(0.02);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		gameFieldData = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putNumber("Right Distance", rEncoder.get());
		SmartDashboard.putNumber("Left Encoder", lEncoder.get());
		SmartDashboard.putString("Value of the thingy", gameFieldData.charAt(0) + " ");
		
//		rDistanceController.disable(); //Enable when ready. Disabled because of crash
//		lDistanceController.disable(); //Enable when ready. Disabled because of crash
//		//Change PID values in shuffleboard. DO NOT GO ABOVE 0.1 ON ANY VALUE
//		rDistanceController.setSetpoint(1);
//		lDistanceController.setSetpoint(1);
//		SmartDashboard.putData("Right PID Loop", rDistanceController);
//		SmartDashboard.putData("Left PID Loop", lDistanceController);
//		drive.tankDrive(rDistanceController.get(), lDistanceController.get()); //Use drive to operate robot because instance was already created in RobotInit
		//Use encoder.getDistance instead of get raw. The values are set in feet, and the robot will over shoot if no PID algorithm is implemented
		switch (AutoChoose) {
			case "Left":
				if(gameFieldData.charAt(0) == 'L') {
				if(rEncoder.get() < 5000) {
					mRight.set(0.35);
					mLeft.set(-0.35);
				}else if(rEncoder.get() > 5000) {
					mRight.set(0);
					mLeft.set(0);
					AutoChoose = "";
				}else {
					mRight.set(0);
					mLeft.set(0);
				}
			}else {
				if(rEncoder.get() < 5000) {
					mRight.set(0.35);
					mLeft.set(-0.35);
				}else if(rEncoder.get() > 5000) {
					mRight.set(0);
					mLeft.set(0);
					AutoChoose = "";
				}else {
					mRight.set(0);
					mLeft.set(0);
				}

				}
				break;
			case "Right":
				if(gameFieldData.charAt(0) == 'L') {
				if(rEncoder.get() < 5000) {
					mRight.set(0.35);
					mLeft.set(-0.35);
				}else if(rEncoder.get() > 5000) {
					mRight.set(0);
					mLeft.set(0);
					AutoChoose = "";
				}else {
					mRight.set(0);
					mLeft.set(0);
				}
			}else {
				if(rEncoder.get() < 5000) {
					mRight.set(0.35);
					mLeft.set(-0.35);
				}else if(rEncoder.get() > 5000) {
					mRight.set(0);
					mLeft.set(0);
					AutoChoose = "";
				}else {
					mRight.set(0);
					mLeft.set(0);
				}
//
				}
				break;
			case "Middle":
				if(gameFieldData.charAt(0) == 'L') {
					if(rEncoder.get() < 5000) {
						mRight.set(0.35);
						mLeft.set(-0.35);
					}else if(rEncoder.get() > 5000) {
						mRight.set(0);
						mLeft.set(0);
						AutoChoose = "";
					}else {
						mRight.set(0);
						mLeft.set(0);
					}
				}else {
					if(rEncoder.get() < 5000) {
						mRight.set(0.35);
						mLeft.set(-0.35);
						SmartDashboard.putBoolean("if encoder stopped", rEncoder.getStopped());
					}else if(rEncoder.get() > 5000) {
						mRight.set(0);
						mLeft.set(0);
						AutoChoose = "";
						turning = "Left";
						SmartDashboard.putBoolean("if encoder stopped", rEncoder.getStopped());
					}else {
						mRight.set(0);
						mLeft.set(0);
					}

				}
				break;

		}
		
		
		switch(turning) {
		case "Left":
			if(ahrs.getYaw() < 90) {
				mRight.set(0.35);
				mLeft.set(0.35);
			}else if(ahrs.getYaw() <90){
				mRight.set(0);
				mLeft.set(0);
				turning = "";
			}else {
				mRight.set(0);
				mLeft.set(0);
			}
			break;
		
		}
		SmartDashboard.updateValues();
	}
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopInit() {
		timed.stop();
		timed.reset();
		scissorOpen.set(true);
		scissorClose.set(false);
		pusherOpen.set(false);
		pusherClose.set(true);
		clawOpen.set(false);
		clawClose.set(true);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = 0;
	}

	@Override
	public void teleopPeriodic() {


		lSpeed = controller.getY(hand.kLeft);
		rSpeed = controller.getY(hand.kRight);

		drive.tankDrive(rSpeed,lSpeed);
		SmartDashboard.putData("Tank Drive", drive);
		SmartDashboard.putNumber("Right Distance", rEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", lEncoder.getDistance());

		if( limitOne.get() && limitTwo.get()&& !(isClosed)){
			clawOpen.set(false);
			clawClose.set(true);
			isClosed = true;
			toBePushed = true;
			clawSeq = 0;
		}

		if(controller.getBumper(hand.kRight) && isClosed){
			clawOpen.set(true);
			clawClose.set(false);
			isClosed = false;
		}

		SmartDashboard.putBoolean("IS CLOSED", isClosed);
		SmartDashboard.putBoolean("To be pushed", toBePushed);
		SmartDashboard.putBoolean("Start Button", controller.getStartButton());
 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());

		if(controller.getBumper(hand.kLeft)&& toBePushed) {
			clawSeq = 0;
			timed.reset();
			timed.start();
		}

		if(controller.getStartButton()) {
			clawOpen.set(false);
			clawClose.set(true);
			isClosed = true;
			scissor = false;
			toBePushed = false;
//
		}
		if(controller.getBackButton()) {
			isClosed = true;

		}
		
		clawSeq = (int)timed.get();
		SmartDashboard.putNumber("claw seq", clawSeq);
		switch(clawSeq) {
			case 0:
				clawOpen.set(true);
				clawClose.set(false);
				pusherOpen.set(true);
				pusherClose.set(false);
				break;
			
			case 1:
				pusherOpen.set(false);
				pusherClose.set(true);
				break;
			case 2:
				toBePushed = false;
				isClosed = false;
				timed.stop();
				break;
		}

		if(controller.getBButton()) {
			scissorOpen.set(false);
			scissorClose.set(true);
		}

		SmartDashboard.putBoolean("LimitSwitchLeft", limitOne.get());
		SmartDashboard.putBoolean("LimitSwitchRight", limitTwo.get());


		if(controller.getTriggerAxis(hand.kLeft)>= 0.01) {
			SmartDashboard.putNumber("ArmL",controller.getTriggerAxis(hand.kLeft));
			arm.set(controller.getTriggerAxis(hand.kLeft) * -1);
		}
		if(controller.getTriggerAxis(hand.kRight)>= 0.01) {
			arm.set(controller.getTriggerAxis(hand.kRight ));
		}

			SmartDashboard.updateValues();

	}
	
	

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testInit() {
		ahrs.zeroYaw();
		timed.stop();
		timed.reset();
		scissorOpen.set(true);
		scissorClose.set(false);
		if(controller.getBackButton()) {
			SmartDashboard.putBoolean("Start Button", controller.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		}else if(controller.getStartButton()) {
 			SmartDashboard.putBoolean("Start Button", controller.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		}
 		else {
 			SmartDashboard.putBoolean("Start Button", controller.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		}
		SmartDashboard.updateValues();
	}

	




	
	 @Override
 	public void testPeriodic() {
		 

 		lSpeed = controller.getY(hand.kLeft);
 		rSpeed = controller.getY(hand.kRight);

// 		drive.tankDrive(rSpeed,lSpeed);

 		if(controller.getBButtonPressed() && !(scissor)) {
 			scissorOpen.set(true);
 			scissorClose.set(false);
 			scissor = true;
 			
 		}

 		if(controller.getXButtonPressed() && scissor) {
 			scissorOpen.set(false);
 			scissorClose.set(true);
 			scissor = false;
 			
	 }

 		 if(controller.getBumper(hand.kRight) && isClosed){
 			clawOpen.set(true);
 			clawClose.set(false);
 			isClosed = false;
 		}
 		if(controller.getBumper(hand.kLeft) && !(isClosed)){
 			clawOpen.set(false);
 			clawClose.set(true);
 			isClosed = true;
 		}

 		SmartDashboard.putBoolean("Start Button", controller.getStartButton());
 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		
 		if(stick.getRawButton(3)) {
 			lWinch.set(-1);
 		}else {
 			lWinch.set(0);
 		}
 		
 		if(stick.getRawButton(4)) {
 			rWinch.set(1);
 		}else {
 			rWinch.set(0);
 		}
 		
 		if(controller.getTriggerAxis(hand.kLeft)>= 0.01) {
			SmartDashboard.putNumber("ArmL",controller.getTriggerAxis(hand.kLeft));
			arm.set(controller.getTriggerAxis(hand.kLeft) * -1);
		}
		if(controller.getTriggerAxis(hand.kRight)>= 0.01) {
			arm.set(controller.getTriggerAxis(hand.kRight ));
		}
 		if(controller.getYButton()) {
 			timed.start();
 		}
 		
 		if(lEncoder.get()< 2000) {
 			SmartDashboard.putBoolean("Here i am", true);
 		} else {
			SmartDashboard.putBoolean("Here i am", false);
 		}
 		
	 
 	
//	 if (stick.getRawButton(5)) {
//		
//		 pusherOpen.set(true);
//		 pusherClose.set(false);
//	 
//	  }else {
//		  
//		  pusherOpen.set(false);
//	 	  pusherClose.set(true);
//	 	  
//	  	}
//	 
	 if(stick.getRawButton(2)) {
		 clawOpen.set(false);
		 clawClose.set(true);
	 }else {
		 clawOpen.set(true);
		 clawClose.set(false);
	 }
	 
	 SmartDashboard.putBoolean("turnController Enabled", turnController.isEnabled());
	 SmartDashboard.putNumber("navX Yaw", ahrs.getYaw());
    
     	if ( stick.getRawButton(5)) {
     		if(ahrs.getYaw() <90) {
     			drive.tankDrive(1,-1);
     		}
//     		if (!turnController.isEnabled()) {
//     			turnController.setSetpoint(kTargetAngleDegrees);
//     			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
//     			turnController.enable();
//     		}
//     		
//     		double leftStickValue = rotateToAngleRate;
//     		double rightStickValue = rotateToAngleRate;
//     		SmartDashboard.putNumber("Turn Value Left", leftStickValue);
//     		SmartDashboard.putNumber("Turn Value Right", rightStickValue);
//     		drive.tankDrive(leftStickValue,  rightStickValue);
     		
     	} else if (stick.getRawButton(6)) {
     		ahrs.zeroYaw();
	 
     	}else {
    		if(turnController.isEnabled()) {
    			turnController.disable();
    		}
    		drive.tankDrive(rSpeed,lSpeed);
    	}
     	
     		
//	 double magnitude = (stick.getY() + stick.getY());
//		double leftStickValue = magnitude + rotateToAngleRate;
//		double rightStickValue = magnitude - rotateToAngleRate;
//		drive.tankDrive(leftStickValue,rightStickValue);
//	 
	 }
	 
	 @Override
	 public void pidWrite(double output) {
	 	rotateToAngleRate = output;
	 } 
  }


	 
  
	 