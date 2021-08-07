// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.BatterySim;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushed);
  private final CANEncoder m_encoder = m_motor.getEncoder();
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kCosVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  // Simulation classes help us simulate what's going on, including gravity.
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getBag(1),
          ArmConstants.kReduction,
          SingleJointedArmSim.estimateMOI(ArmConstants.kLength, ArmConstants.kMass),
          ArmConstants.kLength,
          ArmConstants.kMinAngle,
          ArmConstants.kMaxAngle,
          ArmConstants.kMass,
          true,
          VecBuilder.fill(Units.degreesToRadians(0.5)) // Add noise with a std-dev of 0.5 degrees
      );

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    m_motor.restoreFactoryDefaults();
    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.burnFlash();

    BatterySim.addCurrentSource(m_armSim::getCurrentDrawAmps);
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    // TODO: Set the CANSparkMax encoder value
    //m_encoderSim.setDistance(m_armSim.getAngleRads());
  }

  /**
   * Set the arm output percent
   * @param pct Speed in percent to set the arm, positive = up
   */
  public void setOutput(double pct) {
    m_motor.set(pct);
  }

}
