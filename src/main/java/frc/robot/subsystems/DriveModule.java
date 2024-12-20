package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DriveModule extends SubsystemBase {
    
    public final String id;
    public final Translation2d offset;

    public DriveModule(String id, Translation2d offset){
        this.id = id;
        this.offset = offset;

    }

    public final String getId() {
            return this.id;
    }

    public abstract Rotation2d getAngle();

    public abstract double getVelocity();


}
