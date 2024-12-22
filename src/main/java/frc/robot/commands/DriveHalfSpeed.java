package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

// What does DriveHalfSpeed extend?
public class DriveHalfSpeed extends {
    // We declare the subsystem we use here
    private final Drivetrain driveSubsystem;
    
    public DriveHalfSpeed(Drivetrain driveSubsystem) {
    // Instantiate our subsystem in the constructor (remember to use this.driveSubsystem)
    // Prevent a conflict if this command is run in tandem with another using driveSubsystem
    
    }

    public void execute() {
    // Call the arcadeDrive method from Drivetrain.java with arguments of 0.5

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
