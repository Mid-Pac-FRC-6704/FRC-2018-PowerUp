/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6704.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj.GenericHID.*;
import edu.wpi.first.wpilibj.XboxController;

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
	
	private double rSpeed;
	private double lSpeed;
	
	private Victor arm; //Arm Motor
	private Victor rWinch; //Right Winch Motor
	private Victor lWinch; //Left Winch Motor
	
	private Solenoid clawOpen; //Solenoid for opening claw
	private Solenoid clawClose; //Solenoid for closing claw
	private Solenoid scissorOpen; //Solenoid for opening scissor lift
	private Solenoid scissorClose; //Solenoid for closing scissor lift
	
	private Hand hand;
	private static XboxController controller;
	private Joystick stick;
	private int counterClaw;
	private int counterPush;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		timer = new Timer();
		
		trMotor = new Spark(0);
		tlMotor = new Spark(1);
		brMotor = new Spark(2);
		blMotor = new Spark(3);
		
		clawOpen = new Solenoid(0);
		clawClose = new Solenoid(1);
		scissorOpen = new Solenoid(2);
		scissorClose = new Solenoid(3);
		
		stick = new Joystick(0);
		controller = new XboxController(0);
		counterClaw = 1;
		counterPush = 1;
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
	public void teleopPeriodic() {
		
		lSpeed = controller.getY(hand.kLeft);
		trMotor.set(lSpeed * -1);
		brMotor.set(lSpeed* -1);
		
		
		rSpeed = controller.getY(hand.kRight);
		tlMotor.set(rSpeed);
		blMotor.set(rSpeed);
//		if(controller.getBumper(hand.kRight)) {
//			arm.set(0.25);
//		}else {
//			arm.set(0);
//		}
//		
//		if(controller.getBumper(hand.kLeft)) {
			clawOpen.set(controller.getBumper(hand.kLeft));
			clawClose.set(!(controller.getBumper(hand.kLeft)));
			
			scissorOpen.set(controller.getBumper(hand.kRight));
			scissorClose.set(!(controller.getBumper(hand.kRight)));
//			counterClaw++;
//		}else {
//			clawOpen.set(false);
//			clawClose.set(true);
//			counterClaw++;
//		}
		
//		if(counterPush%2 !=0 && controller.getBumper(hand.kRight)) {
//			clawOpen.set(true);
//			clawClose.set(false);
//			counterPush++;
//		}else {
//			clawOpen.set(false);
//			clawClose.set(true);
//			counterPush++;
//		}
		
		
		
//		
//		if(controller.getBumper(hand.kLeft)) {
//			controller.setRumble(RumbleType.kLeftRumble, 1);
//			controller.setRumble(RumbleType.kRightRumble, 1);
//		}else {
//			controller.setRumble(RumbleType.kLeftRumble, 0);
//			controller.setRumble(RumbleType.kRightRumble, 0);
//		}
//		
//		if(stick.getTrigger()) {
//			rWinch.set(1.0);
//			lWinch.set(1.0);
//		}else {
//			rWinch.set(0);
//			lWinch.set(0);
//		}
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}