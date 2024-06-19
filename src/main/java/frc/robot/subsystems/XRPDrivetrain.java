// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj.xrp.XRPRangefinder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveForTimeCommand;

public class XRPDrivetrain extends SubsystemBase {
  private static final double K_GEAR_RATIO =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double COUNTS_PER_MOTOR_SHAFT_REV = 12.0;
  private static final double COUNTS_PER_REVOLUTION = COUNTS_PER_MOTOR_SHAFT_REV * K_GEAR_RATIO; // 585.0
  private static final double WHEEL_DIAMETER_INCH = 2.3622; // 60 mm

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  //PID
  private final PIDController m_PID = new PIDController(0,0,0);

  // Set up the XRPGyro
  private final XRPGyro m_gyro = new XRPGyro();

  // Set up the XRPRangeFinder
  private final XRPRangefinder m_rangefinder = new XRPRangefinder();

  // Create a timer
  private final Timer m_timer = new Timer();

  /** Creates a new XRPDrivetrain. */
  public XRPDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
    m_rightEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
    resetEncoders();

    m_PID.setTolerance(7.5);
    m_PID.enableContinuousInput(0,360);

    m_timer.reset();

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

  public double getGyroHeading() {
    return m_gyro.getAngle();
  }
  public double getRangeInches() {
    return m_rangefinder.getDistanceInches();
  }
  public Command avoidWallsFactory() {
    return new ConditionalCommand(
            new DriveForTimeCommand(this, 1.0, 1.0, 0.0).withTimeout(0.2),
            turnForDegreesFactory(90.0),
            () -> getRangeInches() >= 25.0
    ).repeatedly();

  }

  public Command turnForDegreesFactory(double degrees) {
    final double setPoint = degrees + getGyroHeading();
    return run(() -> {
      double output = m_PID.calculate(getGyroHeading(),setPoint);
      arcadeDrive(0.0, output);
    }).until(m_PID::atSetpoint);
  }
  // Neither of these periodic methods are needed, but you can add things to them
  // to tinker

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Robot Heading", getGyroHeading());
    SmartDashboard.putNumber("Distance (in.)", getRangeInches());
    SmartDashboard.putNumber("PID Setpoint", m_PID.getSetpoint());
    SmartDashboard.putNumber("Measurement", getGyroHeading());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  }

