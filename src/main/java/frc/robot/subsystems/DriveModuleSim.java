package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveModuleSim extends DriveModule {

    public DriveModuleSim(String id, Translation2d offset) {
        super(id, offset);
    }

    public Rotation2d getAngle(){
        return null;
    }

	@Override
	public double getVelocity() {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'getVelocity'");
	}

}



