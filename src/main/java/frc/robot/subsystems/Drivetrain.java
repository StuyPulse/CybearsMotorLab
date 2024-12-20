package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;


public class Drivetrain extends SubsystemBase {

  //Declare fields for rightMotors and rightEncoder underneath
  private final PWMSparkMax[] leftMotors;

  private final Encoder leftEncoder;

  private final DifferentialDrive driveTrain;
  private final ADIS16448_IMU gyro;
  private final DifferentialDriveOdometry odometry;

  private final Field2d field;
  private final FieldObject2d robotPose;
  private final DifferentialDrivetrainSim driveTrainSim;
  private final EncoderSim leftEncoderSim;
  private final EncoderSim rightEncoderSim;
  private final ADIS16448_IMUSim gyroSim;
  
  private final Joystick joystick;

  public Drivetrain() {

    // Instantiate rightMotors
    leftMotors = new PWMSparkMax[] {
      new PWMSparkMax(Constants.Ports.LEFT_TOP),
      new PWMSparkMax(Constants.Ports.LEFT_BOTTOM)
    };

    // Uncomment the code below when rightMotors is coded
    // rightMotors[0].setInverted(true);
    // rightMotors[1].setInverted(true);

    // Instantiate rightEncoder
    leftEncoder = new Encoder(Constants.Ports.LEFT_A, Constants.Ports.LEFT_B);
    // setDistancePerPulse for rightEncoder
    leftEncoder.setDistancePerPulse(Constants.Drivetrain.DISTANCE_PER_PULSE);
    
    //addFollower for rightMotors
    leftMotors[0].addFollower(leftMotors[1]);

    // Uncomment this code below when rightMotors is instantiated
    // DriveTrain = new DifferentialDrive(leftMotors[0], rightMotors[0]);

    // Code for simulation, you can ignore if you want
    gyro = new ADIS16448_IMU();
    odometry = new DifferentialDriveOdometry(getRotation2d(), getLeftDistance(), getRightDistance()); 
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    robotPose = field.getObject("Robot Pose");

    driveTrainSim = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2),
      Constants.Drivetrain.GEAR_RATIO,
      Constants.Drivetrain.INERTIA,
      Constants.Drivetrain.MASS,
      Units.inchesToMeters(3),
      Units.inchesToMeters(24),
      null);

    leftEncoderSim = new EncoderSim(leftEncoder);
    rightEncoderSim = new EncoderSim(rightEncoder);
    gyroSim = new ADIS16448_IMUSim(gyro);

    joystick = new Joystick(0);
  }

  public void arcadeDrive(double fwd, double rot) {
    // Fill in with one line, search WPILib Documentation for a method in DifferentialDrive that completes the task
  }

  //Encoders
  public double getLeftDistance() {
    // Fill in with one line, Encoder has a method getDistance()
  }

  // Code getRightDistance()

  public double getDistance() {
    // Find the average of the left and right measurements from the two encoders
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
   }

  // Code getRightVelocity()

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

    SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled (m)", getDistance());
    SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled Left (m)", getLeftDistance());
    SmartDashboard.putNumber("Debug/Drivetrain/Distance Traveled Right (m)", getRightDistance());
    SmartDashboard.putNumber("Debug/Drivetrain/Velocity (m per s)", getVelocity());
    SmartDashboard.putNumber("Debug/Drivetrain/Velocity Left (m per s)", getLeftVelocity());
    SmartDashboard.putNumber("Debug/Drivetrain/Velocity Right (m per s)", getRightVelocity());
  }

  @Override
  public void simulationPeriodic() {
    double joystickX = joystick.getX();
    double joystickY = joystick.getY();
    if (Math.abs(joystickX) >= 0.01) {
      driveTrainSim.setInputs(
        joystickX * 0.5,
        -joystickX * 0.5 
      );
    }
    if (Math.abs(joystickY) >= 0.01) {
        driveTrainSim.setInputs(
          -joystickY,
          -joystickY 
        );

      }

    robotPose.setPose(driveTrainSim.getPose());
    field.setRobotPose(driveTrainSim.getPose());
    SmartDashboard.putData(field);
    driveTrainSim.update(0.02);
    
    leftEncoderSim.setDistance(driveTrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveTrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveTrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveTrainSim.getRightVelocityMetersPerSecond());
    gyroSim.setGyroAngleZ(-driveTrainSim.getHeading().getDegrees());
  }

}