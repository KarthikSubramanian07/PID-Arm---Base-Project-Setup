// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** A robot arm subsystem that moves with a motion profile. */
public class ArmSubsystem extends ProfiledPIDSubsystem {
  private final WPI_TalonFX m_motor = new WPI_TalonFX(ArmConstants.kMotorPort);
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kCosVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);

    TalonFXConfiguration configs = new TalonFXConfiguration();
		/* select integ-sensor for PID0 (it doesn't matter if PID is actually used) */
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		/* config all the settings */
    m_motor.configAllSettings(configs);
    m_motor.setSelectedSensorPosition(0);
    // Start arm at rest in neutral position
    setGoal(ArmConstants.kArmOffsetRads);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    return m_motor.getSelectedSensorPosition() * ArmConstants.kEncoderDistancePerPulse + ArmConstants.kArmOffsetRads;
  }  

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
    double encoder_pos = m_motor.getSelectedSensorPosition();
    double measurement = getMeasurement();
    // System.out.println("Set point: " + getController().getGoal().position);
  }
}
