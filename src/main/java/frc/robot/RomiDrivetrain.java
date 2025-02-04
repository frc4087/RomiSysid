// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

public class RomiDrivetrain implements SysidDrivable {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private final MutVoltage _dummyVoltage = Units.Volts.mutable(0);
	private final MutDistance _dummyDistance = Units.Meters.mutable(0);
	private final MutLinearVelocity _dummyVelocity = Units.MetersPerSecond.mutable(0);

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }
  public void setVoltage(Voltage volts) {
		m_leftMotor.setVoltage(volts);
		m_rightMotor.setVoltage(volts);
	}

	@Override
	public void logEntry(SysIdRoutineLog log) {
		log.motor("drive-left")
				.voltage(_dummyVoltage.mut_replace(
						m_leftMotor.get() * RobotController.getBatteryVoltage(),
						Units.Volts))
				.linearPosition(
						_dummyDistance.mut_replace(m_leftEncoder.getDistance(),
								Units.Meters))
				.linearVelocity(
						_dummyVelocity.mut_replace(m_leftEncoder.getRate(),
								Units.MetersPerSecond));
		log.motor("drive-right")
				.voltage(_dummyVoltage
						.mut_replace(m_rightMotor.get() * RobotController
								.getBatteryVoltage(), Units.Volts))
				.linearPosition(
						_dummyDistance.mut_replace(m_rightEncoder.getDistance(),
								Units.Meters))
				.linearVelocity(
						_dummyVelocity.mut_replace(m_rightEncoder.getRate(),
								Units.MetersPerSecond));
	}
}
