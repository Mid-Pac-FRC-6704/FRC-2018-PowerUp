/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6704.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.GamepadBase;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

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

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
//	private SmartDashboard Dashboard;
	private Hand left;
	private Spark SparkR;
	private Spark SparkL;
	private Victor VictorR; 
	private Victor VictorL;
	private Solenoid Solenoid0;
	private Solenoid Solenoid1;
	private Solenoid Solenoid2;
	private Solenoid Solenoid3;
	private Joystick m_LeftStick;
	private XboxController BumperOpen;
	private XboxController BumperClose;
	public Timer timer;
	double speedUp = 0.0;
	public Left Left;
	public Right Right;
	public None None;
	
	
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		timer = new Timer();
		SparkR = new Spark(0);
		SparkL = new Spark(1);
		VictorR = new Victor(2);
		VictorL = new Victor(3);
		Solenoid0 = new Solenoid(0);
		Solenoid1 = new Solenoid(1);
		Solenoid2 = new Solenoid(2);
		Solenoid3 = new Solenoid(3);
		m_LeftStick= new Joystick(0);
		Controller = new  XboxController(0);
		
//		Dashboard = new SmartDashboard();
		
		
		
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
		timer.start();
		m_autoSelected = m_chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		Solenoid0.set(false);
		Solenoid1.set(true);
		System.out.println("Auto selected: " + m_autoSelected);
		SmartDashboard.putNumber("Robot Timer", timer.get());
		
		String sendable = "";
        sendable = DriverStation.getInstance().getGameSpecificMessage();
        
        switch (sendable.charAt(0)) {
            case 'L':
            	SmartDashboard.putNumber("Left", Left);
                // Put autonomous code here for the switch being on the left side
                break;
            case 'R':
            	SmartDashboard.putNumber("Right", Right);
                // Put autonomous code here for the switch being on the right side.
                break;
            default:
            	SmartDashboard.putNumber("None", None);
                // Put default auto code here
                break;
        }
		
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		
	
//		timer.start();
//		SmartDashboard.putString("DB/String 0",timer.get());
		SmartDashboard.putNumber("Robot timer",timer.get());
		if (speedUp >= 1) {
			speedUp = 1;
		}
		SparkR.set(speedUp);
		SparkL.set(speedUp);
		if (timer.get() > 5) {
			Solenoid0.set(true);
			Solenoid1.set(false);
		} else if (timer.get() >= 10) {
			Solenoid0.set(false);
			Solenoid1.set(true);
		}
		/*switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
		*/
		speedUp += 0.01;
		SmartDashboard.putNumber("speedUp Timer", speedUp);
		SmartDashboard.updateValues();
		
	}

	/**
	 * This function is called periodically during operator control.
	 */
	
		
	@Override
	public void teleopPeriodic() {
		
		SparkR.set(BumperOpen.getX(left.kLeft));
		SparkL.set(BumperOpen.getX(left.kLeft));
		VictorR.set(BumperOpen.getX(left.kRight));
		VictorL.set(BumperOpen.getX(left.kRight));
		Solenoid0.set(BumperOpen.getBumper(left.kLeft));
		Solenoid1.set(!(BumperOpen.getBumper(left.kLeft)));
		Solenoid2.set(BumperOpen.getBumper(left.kRight));
		Solenoid3.set(!(BumperOpen.getBumper(left.kRight)));
		SmartDashboard.putNumber("Controller speed", BumperOpen.getX(left.kLeft));
	}
	
//	@Override
	public void diabledPeriodic() {
		SparkR.set(0);
		SparkL.set(0);
		VictorR.set(0);
		VictorL.set(0);
		Solenoid0.set(false);
		Solenoid1.set(true);
		SmartDashboard.putString("DB/String 0"," testing ");
		timer.stop();
		timer.reset();
		speedUp=0;
		SmartDashboard.updateValues();
	}
	
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		SparkR.set(1.0);
		SparkL.set(-1.0);
		VictorR.set(1.0);
		VictorL.set(-1.0);
		Solenoid0.set(true);
		Solenoid1.set(false);
	}
}
