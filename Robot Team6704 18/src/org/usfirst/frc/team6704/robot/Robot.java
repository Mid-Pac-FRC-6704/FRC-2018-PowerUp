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
	
	private double kP = 1.1;
	private double kI = 0.15;
	private double kD = 2.3;
	private double kF = 0.00;
	private PIDController rDistanceController;
	private PIDController lDistanceController;

	private double rSpeed; // speed of right motors
	private double lSpeed; // speed of left motors

	private Victor arm; //Arm Motor
	private Victor rWinch; //Right Winch Motor
	private Victor lWinch; //Left Winch Motor

	private boolean scissor;

	private Solenoid clawOpen; //Solenoid for opening claw
	private Solenoid clawClose; //Solenoid for closing claw
	private Solenoid pusherOpen; // Solenoid for opening the pusher
	private Solenoid pusherClose; // Solenoid for closing the pusher
	private Solenoid scissorOpen; //Solenoid for opening scissor lift
	private Solenoid scissorClose; //Solenoid for closing scissor lift

	private Hand hand;
	private static XboxController controllerDrive;// Xbox controller variable 
	private static XboxController controllerClimb;
	private boolean isClosed; 
	private DigitalInput limitOne;
	private DigitalInput limitTwo;
	private int clawSeq;
	private boolean toBePushed;
	private Timer timed;
	private String AutoChoose;
	private String turning;
	private String gameFieldData;
	
	private boolean clawThingy;
	
	private boolean driveBy;
	
	AHRS ahrs;

	double krP = 0.10;
	double krI = 0.10;
	double krD = 0.10;
	double krF = 0.00;
    PIDController turnController; // PID controller variable 
    double rotateToAngleRate;
	final double kToleranceDegrees = 2.0f;    
	final double kTargetAngleDegrees = 0.0f;
	
	boolean testingThis;
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
		
		turnController = new PIDController(krP, krI, krD, krF, ahrs,this); // tells the PID controller what values to use
		turnController.setInputRange(-180.0f,180.0f);// max & min rotation degrees 
		turnController.setOutputRange(-0.2,0.2); // speed at which the robor rotates
		turnController.setAbsoluteTolerance(kToleranceDegrees);
		turnController.setContinuous(true);
		turnController.disable();
		
		

		new Thread(() -> {
            UsbCamera usbCam = CameraServer.getInstance().startAutomaticCapture("USB cam", "/dev/video0"); // USB camera feed activation
            usbCam.setResolution(640, 480); // what resolution the camera broadcasts its feed "USB cam", "/dev/video0"
            
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
		
		rDistanceController = new PIDController(kP, kI, kD, kF, rEncoder, mRight);
		rDistanceController.setPercentTolerance(0.05);
		rDistanceController.setOutputRange(-0.5, 0.5);
		rDistanceController.disable();
		lDistanceController = new PIDController(kP, kI, kD, kF, lEncoder, mLeft);
		lDistanceController.setPercentTolerance(0.1);
		lDistanceController.setOutputRange(-0.5, 0.5);
		lDistanceController.disable();

		arm = new Victor(4); //Arm motor
		rWinch= new Victor(5); //Right winch motor
		lWinch=new Victor(6); //Left winch motor

		clawOpen = new Solenoid(0);
		clawClose = new Solenoid(1);
		pusherOpen = new Solenoid(2);
		pusherClose = new Solenoid(3);
		scissorOpen = new Solenoid(4);
		scissorClose = new Solenoid(5);

		limitOne = new DigitalInput(0);
		limitTwo = new DigitalInput(1);

//		stick = new Joystick(1);
		controllerDrive = new XboxController(0);
		controllerClimb = new XboxController(1);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = -1;
		
		clawThingy = true;

		gameFieldData = "";
		scissorOpen.set(true);
		scissorClose.set(false);
		AutoChoose = "";
		turning = "";
		ahrs.zeroYaw();
		 
		testingThis=false;
		driveBy = false;
	}

	/**
	 * to be called it initialize the disabled periodic
	 */
	
	@Override
	public void disabledInit() {
		timed.stop();
		timed.reset();
		lEncoder.reset();
		rEncoder.reset();
//		scissorOpen.set(true);
//		scissorClose.set(false);
		pusherOpen.set(false);
		pusherClose.set(true);
//		clawOpen.set(false);
//		clawClose.set(true);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = -1;
	}
	
	/**
	 * to be called within the disabled period ..... basically to be called while disabled.
	 */
	
	@Override
	public void disabledPeriodic() {
//		SmartDashboard.putString("Value of the thingy", gameFieldData.charAt(0) + " ");
		 SmartDashboard.putNumber("navX Yaw", ahrs.getYaw());
		
		if(controllerClimb.getAButtonPressed()) {
			AutoChoose = "Middle";
		}
		if(controllerClimb.getXButtonPressed()) {
			AutoChoose = "Left";
		}
		if(controllerClimb.getBButtonPressed()) {
			AutoChoose = "Right";
		}
		SmartDashboard.putString("Autonomous Selecting", AutoChoose);
		if(controllerDrive.getBackButton()) {
			SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());
 		}else if(controllerDrive.getStartButton()) {
 			SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());
 		}
 		else {
 			SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());
 		}
		
		if(controllerClimb.getStartButton()) {
			ahrs.zeroYaw();
		}
		SmartDashboard.putData("Rotate PID Loop", turnController);
		SmartDashboard.putNumber("Angle Error", turnController.getError());
		SmartDashboard.putData("Tank Drive", drive);
		SmartDashboard.putNumber("Rotate PID Output", rotateToAngleRate);
		SmartDashboard.putNumber("Right Distance", rEncoder.get());
		SmartDashboard.putNumber("Left Encoder", lEncoder.get());
		SmartDashboard.putBoolean("IS CLOSED", isClosed);
		SmartDashboard.putBoolean("To be pushed", toBePushed);
		SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());

		SmartDashboard.updateValues();
	}
	
	/**
	 * initialization of the autonomous period
	 */
	
	@Override
	public void autonomousInit() {
		lEncoder.reset();
		rEncoder.reset();
		scissorOpen.set(true);
		scissorClose.set(false);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = -1;
		turning = "";
//		mLeft.setInverted(true);
//		rEncoder.reset();
//		lEncoder.reset();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		gameFieldData = DriverStation.getInstance().getGameSpecificMessage();
		SmartDashboard.putNumber("Right Distance", rEncoder.get());
		SmartDashboard.putNumber("Left Encoder", lEncoder.get());
		SmartDashboard.putString("Game Field Data", gameFieldData.charAt(0) + " ");
//		
//		rDistanceController.disable(); //Enable when ready. Disabled because of crash
//		lDistanceController.disable(); //Enable when ready. Disabled because of crash
		/*
		//Change PID values in shuffleboard. DO NOT GO ABOVE 0.1 ON ANY VALUE
		rDistanceController.setSetpoint(10);
		lDistanceController.setSetpoint(10);
		SmartDashboard.putData("Right PID Loop", rDistanceController);
		SmartDashboard.putData("Left PID Loop", lDistanceController);
		SmartDashboard.putData("Rotate PID Loop", turnController);
		
		*/
		//Use encoder.getDistance instead of get raw. The values are set in feet, and the robot will over shoot if no PID algorithm is implemented

		
		switch (AutoChoose) {
			case "Left":
				if(gameFieldData.charAt(0) == 'L') {
					if(rEncoder.get() < 16000 && lEncoder.get() < 16000) {
						mRight.set(0.45);
						mLeft.set(-0.45);
					}else {
						mRight.set(0);
						mLeft.set(0);
						ahrs.zeroYaw();
						AutoChoose = "";
						turning = "Left";
					}
				}else {
					if(rEncoder.get() < 16000 && lEncoder.get() < 16000) {
						mRight.set(0.45);
						mLeft.set(-0.45);
					}else {
						mRight.set(0);
						mLeft.set(0);
						AutoChoose = "";
					}
				}
				break;
			case "Right":
				if(gameFieldData.charAt(0) == 'L') {
					if(rEncoder.get() < 16000 && lEncoder.get() < 16000) {
						mRight.set(0.45);
						mLeft.set(-0.45);
					}else {
						mRight.set(0);
						mLeft.set(0);
						AutoChoose = "";
					}
				}else {
					if(rEncoder.get() < 16000 && lEncoder.get() < 16000) {
						mRight.set(0.45);
						mLeft.set(-0.45);
					}else{
						mRight.set(0);
						mLeft.set(0);
						ahrs.zeroYaw();
						AutoChoose = "";
						turning = "Right";
					}
				}
				break;
//			case "Middle":
//				if(gameFieldData.charAt(0) == 'L') {
//					if(rEncoder.get() < 5000) {
//						mRight.set(0.35);
//						mLeft.set(-0.35);
//					}else if(rEncoder.get() > 5000) {
//						mRight.set(0);
//						mLeft.set(0);
//						AutoChoose = "";
//					}else {
//						mRight.set(0);
//						mLeft.set(0);
//					}
//				}else {
//					if(rEncoder.get() < 5000) {
//						mRight.set(0.35);
//						mLeft.set(-0.35);
//						SmartDashboard.putBoolean("if encoder stopped", rEncoder.getStopped());
//					}else if(rEncoder.get() > 5000) {
//						mRight.set(0);
//						mLeft.set(0);
//						AutoChoose = "";
//						turning = "Left";
//						SmartDashboard.putBoolean("if encoder stopped", rEncoder.getStopped());
//					}else {
//						mRight.set(0);
//						mLeft.set(0);
//					}
//
//				}
//				break;

		}
		
		
		switch(turning) {
			case "Left":
				if(ahrs.getYaw() < 130) {
					mRight.set(-0.55);
					mLeft.set(-0.55);
				}else{
					mRight.set(0);
					mLeft.set(0);
					lEncoder.reset();
					rEncoder.reset();
					driveBy = true;
					turning = "";
				}
			break;
		
		case "Right":
			if(ahrs.getYaw() > -130) {
				mRight.set(0.55);
				mLeft.set(0.55);
			}else{
				mRight.set(0);
				mLeft.set(0);
				lEncoder.reset();
				rEncoder.reset();
				driveBy = true;
				turning = "";
			}
			break;
		
		}
		
		if(driveBy) {
			if(rEncoder.get() < 1000) {
				mRight.set(0.45);
				mLeft.set(0.-45);
			}else if(rEncoder.get() > 1000) {
				mRight.set(0);
				mLeft.set(0);
				pusherOpen.set(false);
				pusherClose.set(true);
				clawOpen.set(false);
				clawClose.set(true);
				driveBy = false;
			}
		}
		
		SmartDashboard.putNumber("navX Yaw", ahrs.getYaw());
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
		pusherOpen.set(true);
		pusherClose.set(false);
		clawOpen.set(true);
		clawClose.set(false);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = 0;
		
		mLeft.setInverted(false);
		lDistanceController.disable();
		rDistanceController.disable();
	}

	@Override
	public void teleopPeriodic() {

		SmartDashboard.putNumber("navX Yaw", ahrs.getYaw());
		
//		Main controller controls
		lSpeed = controllerDrive.getY(hand.kLeft);
		rSpeed = controllerDrive.getY(hand.kRight);

		drive.tankDrive(rSpeed,lSpeed);
		SmartDashboard.putData("Tank Drive", drive); // sends tank drive data to the shuffle board
		SmartDashboard.putNumber("Right Distance", rEncoder.getDistance()); // gets the distance reading from the right encoder and puts it on the shuffleboard
		SmartDashboard.putNumber("Left Encoder", lEncoder.getDistance());// gets the distance reading from the left encoder and puts it on the shuffleboard

		if(clawThingy && controllerDrive.getBumperPressed(hand.kRight)) {
			pusherOpen.set(true);
			pusherClose.set(false);
			clawThingy = false;
		}else if(!clawThingy && controllerDrive.getBumperPressed(hand.kRight)) {
			pusherOpen.set(false);
			pusherClose.set(true);
			clawThingy = true;
		}
		

		SmartDashboard.putBoolean("IS CLOSED", isClosed);
		SmartDashboard.putBoolean("To be pushed", toBePushed);
		SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());

		if(controllerDrive.getBumperPressed(hand.kLeft)&& !clawThingy) {
			clawSeq = 0;
			timed.reset();
			timed.start();
			clawThingy = true;
		}

		if(controllerDrive.getStartButton()) {
			pusherOpen.set(true);
			pusherClose.set(false);
			isClosed = true;
			scissor = false;
			toBePushed = false;
		}
		
		if(controllerDrive.getBackButton()) {
			isClosed = true;
			pusherOpen.set(false);
			pusherClose.set(true);
		}
		
		clawSeq = (int)timed.get();
		SmartDashboard.putNumber("claw seq", clawSeq);
		switch(clawSeq) {
			case 1:
				pusherOpen.set(false);
				pusherClose.set(true);
				clawOpen.set(false);
				clawClose.set(true);
				break;
			
			case 2:
				clawOpen.set(true);
				clawClose.set(false);
				break;
			case 3:
				toBePushed = false;
				isClosed = false;
				timed.stop();
				timed.reset();
				break;
		}

		SmartDashboard.putBoolean("Arm limit switch", limitOne.get());
		
//		 What are we using this limit switch for?
		SmartDashboard.putBoolean("LimitSwitchRight", limitTwo.get());

		
		if(controllerDrive.getTriggerAxis(hand.kLeft)> 0.1) {
			arm.set(controllerDrive.getTriggerAxis(hand.kLeft) * -1);
		} else if(controllerDrive.getTriggerAxis(hand.kRight)> 0.01 && !limitOne.get()) {
			arm.set(controllerDrive.getTriggerAxis(hand.kRight ));
		}else if(!limitOne.get() && controllerDrive.getTriggerAxis(hand.kLeft)< 0.01){
			arm.set(0);
		}else{
			arm.set(0);
		}
		 
		
		// Second controller controls
		if(controllerClimb.getBButton()) {
			scissorOpen.set(false);
			scissorClose.set(true);
		}
		
		lWinch.set(controllerClimb.getTriggerAxis(hand.kLeft));
		rWinch.set(controllerClimb.getTriggerAxis(hand.kRight));
			SmartDashboard.updateValues();

	}
	
	

	/**
	 * This function is called initially before the test periodic. Usually to initialize stuff before starting.
	 */
	@Override
	public void testInit() {
		ahrs.zeroYaw();
		timed.stop();
		timed.reset();
		scissorOpen.set(true);
		scissorClose.set(false);
		if(controllerDrive.getBackButton()) {
			SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());
 		}else if(controllerDrive.getStartButton()) {
 			SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());
 		}
 		else {
 			SmartDashboard.putBoolean("Start Button", controllerDrive.getStartButton());
	 		SmartDashboard.putBoolean("Back Button", controllerDrive.getBackButton());
 		}
		ahrs.zeroYaw();
		SmartDashboard.updateValues();
	}
	
	
	/**
	 * This class is a test. Only to be used when testing things. once the things are tested, you are able to put them into the teleop periodic
	 * otherwise, don't put them into the teleop
	 */
	 @Override
 	public void testPeriodic() {
		 SmartDashboard.putNumber("navX Yaw", ahrs.getYaw());
		 
		 if(ahrs.getYaw() > -90) {
			mRight.set(0.5);
			mLeft.set(0.5);
		 }else {
			mRight.set(0);
			mLeft.set(0);
		 }
		 
//		|| (ahrs.getYaw() < -90 && ahrs.getYaw() > -110)

//			 if(ahrs.getYaw() > -100  ) {
//				mRight.set(-0.4);
//				mLeft.set(-0.4);
//			 }else {
//				mRight.set(0);
//				mLeft.set(0);
//			 }
		 
//			turnController.setSetpoint(15.0f);
//			
//			SmartDashboard.putData("Rotate PID Loop", turnController);
//			SmartDashboard.putNumber("Angle Error", turnController.getError());
//			SmartDashboard.putData("Tank Drive", drive);
//			SmartDashboard.putNumber("Rotate PID Output", rotateToAngleRate);
//			turnController.enable();
//			double left = rotateToAngleRate;
//			double right = rotateToAngleRate;
//			mRight.set(right);
//			mLeft.set(left);
//			drive.tankDrive(left, right);
//			
//			
//			SmartDashboard.updateValues();
		 /*

 		lSpeed = controller.getY(hand.kLeft); // sets left motor speed based off of y value
 		rSpeed = controller.getY(hand.kRight); // sets right motor speed based off of y value

// 		drive.tankDrive(rSpeed,lSpeed);

 		if(controller.getBButtonPressed() && !(scissor)) { // opens scissor lift when B button pressed
 			scissorOpen.set(true);
 			scissorClose.set(false);
 			scissor = true;
 			
 		}

 		if(controller.getXButtonPressed() && scissor) { // closes scissor lift when X button pressed
 			scissorOpen.set(false);
 			scissorClose.set(true);
 			scissor = false;
 			
	 }

 		 if(controller.getBumper(hand.kRight) && isClosed){ // opens claw when right Bumper is pressed 
 			clawOpen.set(true);
 			clawClose.set(false);
 			isClosed = false;
 		}
 		if(controller.getBumper(hand.kLeft) && !(isClosed)){ // closes claw when left bumper is pressed
 			clawOpen.set(false);
 			clawClose.set(true);
 			isClosed = true;
 		}

 		SmartDashboard.putBoolean("Start Button", controller.getStartButton());
 		SmartDashboard.putBoolean("Back Button", controller.getBackButton());
 		
 		if(stick.getRawButton(3)) { // runs left winch backwards when button 3 is pressed
 			lWinch.set(-1);
 		}else { // if button 3 is not pressed winch motors will not activate
 			lWinch.set(0);
 		}
 		
 		if(stick.getRawButton(4)) { // runs winch forward if button 4 is pressed 
 			rWinch.set(1);
 		}else {// if button 3 is not pressed winch will not run
 			rWinch.set(0);
 		}
 		
 		if(controller.getTriggerAxis(hand.kLeft)>= 0.01) {
			SmartDashboard.putNumber("ArmL",controller.getTriggerAxis(hand.kLeft));
			arm.set(controller.getTriggerAxis(hand.kLeft) * -1);
		}else if(controller.getTriggerAxis(hand.kRight)>= 0.01) {
			arm.set(controller.getTriggerAxis(hand.kRight ));
		}else if (controller.getTriggerAxis(hand.kRight)< 0.01 && controller.getTriggerAxis(hand.kLeft)< 0.01){
			arm.set(0);
		}
		else {
			arm.set(0);
		}
 		if(controller.getYButton()) {
 			timed.start();
 		}
 		
 		if(lEncoder.get()< 2000) {
 			SmartDashboard.putBoolean("Here i am", true);
 		} else {
			SmartDashboard.putBoolean("Here i am", false);
 		}
 		
	 
 	
//	 if (stick.getRawButton(5)) { // opens pusher if button 5 is pressed
//		
//		 pusherOpen.set(true);
//		 pusherClose.set(false);
//	 
//	  }else { // closes pusher is button 5 is not pressed
//		  
//		  pusherOpen.set(false);
//	 	  pusherClose.set(true);
//	 	  
//	  	}
//	 
	 if(stick.getRawButton(2)) { // opens claw if button 2 is pressed 
		 pusherOpen.set(false);
		 pusherClose.set(true);
	 }else { // closes claw if button 2 is not pressed 
		 pusherOpen.set(true);
		 pusherClose.set(false);
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
     		
     	} else if (stick.getRawButton(6)) { // zeros Yaw if button 6 is pressed 
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
     	
	 */
	 }
	 
	 
	 @Override
	 public void pidWrite(double output) {
	 	rotateToAngleRate = output;
	 }
	  
  }


	 
  
	 