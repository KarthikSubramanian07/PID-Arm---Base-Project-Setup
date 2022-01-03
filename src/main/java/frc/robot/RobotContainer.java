// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystem
  private final ArmSubsystem m_robotArm = new ArmSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * JoystickButton}.
   */
  private void configureButtonBindings() {

    // Move the arm to 2 radians above horizontal when the 'A' button is pressed.
    new JoystickButton(m_driverController, 3)
        .whenPressed(
            () -> {
              m_robotArm.setGoal(Math.PI/2);
              m_robotArm.enable();
            },
            m_robotArm);

    // Move the arm to neutral position when the 'B' button is pressed.
    new JoystickButton(m_driverController, 4)
        .whenPressed(
            () -> {
              m_robotArm.setGoal(Constants.ArmConstants.kArmOffsetRads);
              m_robotArm.enable();
            },
            m_robotArm);

    // Disable the arm controller when Y is pressed.
    new JoystickButton(m_driverController, 5).whenPressed(m_robotArm::disable);
  }

  /**
   * Disables all ProfiledPIDSubsystem and PIDSubsystem instances. This should be called on robot
   * disable to prevent integral windup.
   */
  public void disablePIDSubsystems() {
    m_robotArm.disable();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
