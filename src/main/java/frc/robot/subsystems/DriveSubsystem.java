// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The robot's drive
  private final DifferentialDrive m_drive;

  private final CANSparkMax m_leftSpark1;
  private final CANSparkMax m_leftSpark2;
  private final CANSparkMax m_rightSpark1;
  private final CANSparkMax m_rightSpark2;
  private final CANEncoder m_leftEncoder;
  private final CANEncoder m_rightEncoder;

  private void initSparkMax(CANSparkMax sm) {
    sm.restoreFactoryDefaults();
    sm.setIdleMode(IdleMode.kCoast);
    sm.setSmartCurrentLimit(40);
  }

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_leftSpark1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    m_leftSpark2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    m_rightSpark1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    m_rightSpark2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

    initSparkMax(m_leftSpark1);
    initSparkMax(m_leftSpark2);
    initSparkMax(m_rightSpark1);
    initSparkMax(m_rightSpark2);

    m_leftSpark2.follow(m_leftSpark1);
    m_rightSpark2.follow(m_rightSpark2);

    m_leftEncoder = m_leftSpark1.getEncoder();
    m_rightEncoder = m_rightSpark1.getEncoder();

    m_drive = new DifferentialDrive(m_leftSpark1, m_rightSpark1);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
}
