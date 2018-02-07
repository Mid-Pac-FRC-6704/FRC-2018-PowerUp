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

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.DigitalInput;
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
public class Robot extends IterativeRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();

	public Timer timer;

	private Spark trMotor; //Top Right Drive Motor
	private Spark tlMotor; //Top Left Drive Motor
	private Spark brMotor; //Bottom Right Drive Motor
	private Spark blMotor; //Bottom Left Drive Motor

	private SpeedControllerGroup mLeft;
	private SpeedControllerGroup mRight;
	private DifferentialDrive drive;

	private Encoder rEncoder;
	private Encoder lEncoder;

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

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);

		new Thread(() -> {
            UsbCamera usbCam = CameraServer.getInstance().startAutomaticCapture();
            usbCam.setResolution(640, 480);
            //Kinect class -> not done yet. Will get too it.
            //Kinect kinect = new Kinect();
        }).start();


		timer = new Timer();
		timed = new Timer();

		trMotor = new Spark(0);
		brMotor = new Spark(2);
		mRight = new SpeedControllerGroup(trMotor, brMotor);

		tlMotor = new Spark(1);
		blMotor = new Spark(3);
		mLeft = new SpeedControllerGroup(tlMotor, blMotor);

		drive = new DifferentialDrive(mLeft, mRight);

		rEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
		lEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
		//Still need to dial in encoder settings and reverse left or right encoder (forgot which one)
		//2048 pulses per revolution

		arm = new Victor(4);
		rWinch= new Victor(5); //Right Winch Motor
		lWinch=new Victor(6);

		clawOpen = new Solenoid(0);
		clawClose = new Solenoid(1);
		pusherOpen = new Solenoid(2);
		pusherClose = new Solenoid(3);
		scissorOpen = new Solenoid(4);
		scissorClose = new Solenoid(5);



		limitOne = new DigitalInput(0);
		limitTwo = new DigitalInput(1);

		stick = new Joystick(0);
		controller = new XboxController(0);
		isClosed = true;
		scissor = false;
		toBePushed = false;
		clawSeq = 0;

		scissorOpen.set(true);
		scissorClose.set(false);
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
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
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
		SmartDashboard.putNumber("Right Distance", rEncoder.get());
		SmartDashboard.putNumber("Left Encoder", lEncoder.get());

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

		if(controller.getBumper(hand.kLeft)&& toBePushed) {
//			clawSeq = 1;
			timed.reset();
			timed.start();
			toBePushed = false;
			isClosed = true;
		}

		if(controller.getStartButton()) {
			clawOpen.set(false);
			clawClose.set(true);
			isClosed = true;
			scissor = false;
			toBePushed = false;
//
		}

		SmartDashboard.putNumber("claw seq", clawSeq);
		

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
//			arm.set(controller.getTriggerAxis(hand.kLeft) * -1);
			arm.set(controller.getTriggerAxis(hand.kRight ));
		}

			SmartDashboard.updateValues();

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testInit() {
		timed.stop();
		timed.reset();
	}
	 @Override
 	public void testPeriodic() {

 		lSpeed = controller.getY(hand.kLeft);
 		rSpeed = controller.getY(hand.kRight);

 		drive.tankDrive(rSpeed,lSpeed);

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

// 		pusherOpen.set(controller.getYButton());
// 		pusherClose.set(!(controller.getYButton()));

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

 		
 		if(controller.getStartButton()) {
 			rWinch.set(1.0);
 			lWinch.set(1.0);
 		}else {
 			rWinch.set(0);
 			lWinch.set(0);
 		}
 		
 		if(controller.getBackButton()) {
 			rWinch.set(-1.0);
 			lWinch.set(-1.0);
 		}else {
 			rWinch.set(0);
 			lWinch.set(0);
 		}
 		if(controller.getTriggerAxis(hand.kLeft)>= 0.01) {
 			SmartDashboard.putNumber("ArmL",controller.getTriggerAxis(hand.kLeft));
 			arm.set(controller.getTriggerAxis(hand.kLeft) * -1);
 		}
 		if(controller.getTriggerAxis(hand.kRight)>= 0.01) {
 			arm.set(controller.getTriggerAxis(hand.kLeft) * -1);
 			arm.set(controller.getTriggerAxis(hand.kRight ));
 		}
 		
 		if(controller.getYButton()) {
 			timed.start();
 		}
 		clawSeq = (int)timed.get();
 		SmartDashboard.putNumber("Timer values", clawSeq);
 		
 		switch(clawSeq) {
 		case 1:
 			SmartDashboard.putBoolean("Here i am", true);
 			break;
 		case 2:
 			SmartDashboard.putBoolean("Here i am", false);
 			break;
 		case 3:
 			SmartDashboard.putBoolean("Here i am", true);
 			break;
 		}
 		
 	}
	 
	 public int punching(int Seq) {
		 int ThisSeq = Seq; 
			switch(ThisSeq) {
			case 1:
				clawOpen.set(true);
				clawClose.set(false);
				ThisSeq = 2;
				break;
			case 2:
				ThisSeq = 3;
				break;
			case 3:
				ThisSeq = 4;
				break;
			case 4:
				ThisSeq = 5;
				break;
			case 5:
				ThisSeq = 6;
				break;
			case 6:
				pusherOpen.set(true);
				pusherClose.set(false);
				ThisSeq = 7;
				break;
			case 7:
				ThisSeq = 8;
				break;
			case 8:
				ThisSeq = 9;
				break;
			case 9:
				ThisSeq = 10;
				break;
			case 10:
				ThisSeq = 11;
				break;
			case 11:
				pusherOpen.set(false);
				pusherClose.set(true);
				ThisSeq = 12;
				break;
			case 12:
				ThisSeq = 13;
				break;
			case 13:
				isClosed = true;
				ThisSeq = 14;
				break;
			case 14:
				ThisSeq = 15;
				break;
			case 15:
				ThisSeq = 16;
				break;
			case 16:
				ThisSeq = 17;
				break;
			case 17:
				ThisSeq = 18;
				break;
			case 18:
				ThisSeq = 19;
				break;
			case 19:
				ThisSeq = 21;
				break;
			case 21:
				ThisSeq = 22;
				break;
			case 22:
				ThisSeq = 23;
				break;
			case 23:
				ThisSeq = 24;
				break;
			case 24:
				ThisSeq = 25;
				break;
			case 25:
				ThisSeq = 26;
				break;
			case 26:
				ThisSeq = 27;
				break;
			case 27:
				isClosed = false;
				ThisSeq = 0;
				break;
			}
			return 0;

	 }
}
