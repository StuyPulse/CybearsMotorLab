// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize the Drivetrain with the name m_robotDrive

  // Create a new XboxController with the name driverController and a port of 1

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -driverController.getLeftY(), -driverController.getLeftX()*0.65),
            m_robotDrive));
  }

  // No buttons to bind
  private void configureButtonBindings() {}

}
