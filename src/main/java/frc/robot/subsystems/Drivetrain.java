// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {

  private final PWMMotorController leftMotors;
  private final PWMMotorController rightMotors;
  private final Encoder left;
  private final Encoder right;
  private final DifferentialDrive driveTrain;
  private final DifferentialDriveOdometry odometry;


  /** Creates a new ExampleSubsystem. */
  @SuppressWarnings("removal")

  public Drivetrain() {
    // leftMotors = new PWM[] {
    //   new PWM(Constants.Ports.LEFT_TOP),
    //   new PWM(Constants.Ports.LEFT_MIDDLE),
    //   new PWM(Constants.Ports.LEFT_BOTTOM)
    // };

    // rightMotors = new PWM[] {
    //   new PWM(Constants.Ports.RIGHT_TOP),
    //   new PWM(Constants.Ports.RIGHT_MIDDLE),
    //   new PWM(Constants.Ports.RIGHT_BOTTOM)
    // };

    left = new Encoder(Constants.Ports.LEFT_A, Constants.Ports.LEFT_B);
    right = new Encoder(Constants.Ports.RIGHT_A, Constants.Ports.RIGHT_B);

    PWMMotorController leftTopController = new PWMMotorController("left top", Constants.Ports.LEFT_TOP) {};
    PWMMotorController leftMiddleController = new PWMMotorController("left middle", Constants.Ports.LEFT_MIDDLE) {};
    PWMMotorController leftBottomController = new PWMMotorController("left bottom", Constants.Ports.LEFT_BOTTOM) {};

    PWMMotorController rightTopController = new PWMMotorController("right top", Constants.Ports.RIGHT_TOP) {};
    PWMMotorController rightMiddleController = new PWMMotorController("right middle", Constants.Ports.RIGHT_MIDDLE) {};
    PWMMotorController rightBottomController = new PWMMotorController("right bottom", Constants.Ports.RIGHT_BOTTOM) {};


    leftMiddleController.addFollower(leftBottomController);
    leftTopController.addFollower(leftMiddleController);

    rightMiddleController.addFollower(rightBottomController);
    rightTopController.addFollower(rightMiddleController);

    driveTrain = new DifferentialDrive(leftTopController, leftMiddleController);

    
    odometry = new DifferentialDriveOdometry(getRotation2d()    );

  }

    


    



  /**
   * Example command factory method.
   *
   * @return a command
   */


  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */


  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
