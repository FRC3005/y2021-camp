// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.RollerConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A robot arm subsystem that moves with a motion profile. */
public class RollerSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(RollerConstants.kMotorPort, MotorType.kBrushless);

  /** Create a new ArmSubsystem. */
  public RollerSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(40);
    m_motor.burnFlash();
  }

  /**
   * Set the roller output percent
   * @param pct Speed in percent to set the arm, positive = in
   */
  public void setOutput(double pct) {
    m_motor.set(pct);
  }

}
