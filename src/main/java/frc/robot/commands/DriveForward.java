package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveForward extends Command {
    private final Drivetrain driveSubsystem;
    private final double distance;

    public DriveForward(Drivetrain driveSubsystem, double distance) {
        // Instantiate driveSubsystem here

        // We add distance to getDistance() because we want to terminate when the encoders' getDistance()
        // increases by "distance" amount- this would mean the wheels have traveled "distance" units
        this.distance = driveSubsystem.getDistance() + distance;
        
        // addRequirements() here
    }

    @Override
    public void initialize() {
        System.out.println("DriveForward Command started!");
    }

    @Override
    public void execute() {
        // What would the speeds be for the right motors?
        driveSubsystem.setMotors(Constants.Drivetrain.autoSpeed, );
    }

    
    @Override
    public void end(boolean interrupted) {
        // set the motor speeds to 0 here
        System.out.println("DriveForward Command ended!");
    }

    @Override
    public boolean isFinished() {
        return driveSubsystem.getDistance() > distance;
    }
}
