// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String goBackward = "forward";
  private static final String goForward = "backward";
  private final Joystick m_joystick=new Joystick(0);
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();


  private final RomiDrivetrain m_drivetrain = new RomiDrivetrain();
  private SendableChooser<Command> m_testChooser = buildTestChooser();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("goBackwards", goBackward);
    m_chooser.addOption("goForward", goForward);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    m_drivetrain.resetEncoders();


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case goForward:
        m_drivetrain.arcadeDrive(0.9, 0.0);
        break;
      case goBackward:
      default:
        m_drivetrain.arcadeDrive(-0.9, 0.0);
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println ("teleop innit");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double teleopSpeed = m_joystick.getRawAxis(1) ;
    double leftNRight = m_joystick.getRawAxis(0) ;
    m_drivetrain.arcadeDrive(teleopSpeed, leftNRight);
    // System.out.print(".");
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println ("disabled innit");
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println ("test innit");CommandScheduler.getInstance().cancelAll();

		Command command = m_testChooser.getSelected();
		if (command != null) {
			command.schedule();
		}
  }
  protected SendableChooser<Command> buildTestChooser() {
		SysIdRoutine sysidFactory = new SysIdRoutine(new SysIdRoutine.Config(),
				new SysIdRoutine.Mechanism(
						m_drivetrain::setVoltage,
						m_drivetrain::logEntry,
						m_drivetrain));

		SendableChooser<Command> chooser = new SendableChooser<>();

		chooser.setDefaultOption("Quasistatic, Forward",
				sysidFactory.quasistatic(Direction.kForward));
		chooser.addOption("Quasistatic, Reverse",
				sysidFactory.quasistatic(Direction.kReverse));
		chooser.addOption("Dynamic, Forward",
				sysidFactory.dynamic(Direction.kForward));
		chooser.addOption("Dynamic, Reverse",
				sysidFactory.dynamic(Direction.kReverse));

		//// SmartDashboard.putData(chooser);
		SmartDashboard.putData("Test Mode Commands", chooser);
		return chooser;
	}


  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_drivetrain.arcadeDrive(0.9, 0.0);
  }
}

