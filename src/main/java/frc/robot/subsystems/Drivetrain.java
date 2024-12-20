// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/* TODO: 
 * Add Field2d related stuff
 * Check if we have all needed methods
 * Motor Config?
 * Drive Commands?
 */


package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;


public class Drivetrain extends SubsystemBase {

  private final PWMSparkMax[] leftMotors;
  private final PWMSparkMax[] rightMotors;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  private final DifferentialDrive driveTrain;

  private final ADIS16448_IMU gyro;
  private final DifferentialDriveOdometry odometry;
  private final Field2d field;

  //Sim stuff
  private final DifferentialDrivetrainSim driveTrainSim;
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;

  private final LinearSystemSim<N2, N1, N2> driveSim;
  private final LinearSystemSim<N2, N1, N1> turnSim;
  
  private final Joystick joystick;

  public Drivetrain() {

    leftMotors = new PWMSparkMax[] {
      new PWMSparkMax(Constants.Ports.LEFT_TOP),
      new PWMSparkMax(Constants.Ports.LEFT_BOTTOM)
    };

    rightMotors = new PWMSparkMax[] {
      new PWMSparkMax(Constants.Ports.RIGHT_TOP),
      new PWMSparkMax(Constants.Ports.RIGHT_BOTTOM)
    };

    rightMotors[0].setInverted(true);
    rightMotors[1].setInverted(true);

    leftEncoder = new Encoder(Constants.Ports.LEFT_A, Constants.Ports.LEFT_B);
    rightEncoder = new Encoder(Constants.Ports.RIGHT_A, Constants.Ports.RIGHT_B);

    
    leftMotors[0].addFollower(leftMotors[1]);
    rightMotors[0].addFollower(rightMotors[1]);

    joystick = new Joystick(0);


    driveTrain = new DifferentialDrive(leftMotors[0], rightMotors[0]);
    leftEncoder.setDistancePerPulse(Constants.Drivetrain.DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.Drivetrain.DISTANCE_PER_PULSE);

    gyro = new ADIS16448_IMU();
    odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance()); //Idk if this is right

    field = new Field2d();

    driveTrainSim = new DifferentialDrivetrainSim( // constant numbers are arbitrary
      DCMotor.getNEO(2),
      Constants.Drivetrain.GEAR_RATIO, 
      10, //Intertia
      50, //Weight KG 
      Units.inchesToMeters(4), //Wheel Radus
      Units.inchesToMeters(24), //Circumference
      null
    );
  
  }

  public void drive() {
    double speed = -joystick.getRawAxis(1) * 0.6;
    double turn = joystick.getRawAxis(4) * 0.3;

    double left = speed + turn;
    double right = speed - turn;

    leftMotors[0].set(left);
    leftMotors[1].set(left);
    
    rightMotors[0].set(-right);
    rightMotors[1].set(-right); 
  }

  //Encoder
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
   }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }

  //Odometry
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();
    odometry.resetPosition(
      Rotation2d.fromDegrees(getHeading()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance(),
      pose
    );
  }

  @Override
  public void periodic() {
    odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );
  }

  @Override
  public void simulationPeriodic() {
    driveTrainSim.setInputs(
      leftMotors[0].get() * 12, //Convert to voltage?
      rightMotors[0].get() * 12
    );

    driveTrainSim.update(0.02);

    leftEncoderSim.setDistance();
    leftEncoderSim.setRate();

    rightEncoderSim.setDistance();
    rightEncoderSim.setRate();
  }

  public void arcadeDrive(double spd, double rot) {
    driveTrain.arcadeDrive(spd, rot);
  }

  public void stop() {
    driveTrain.stopMotor();
  }
}