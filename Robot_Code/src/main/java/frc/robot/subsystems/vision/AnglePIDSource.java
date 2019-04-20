package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.subsystems.Drive;

public class AnglePIDSource implements PIDSource {
    private PIDSourceType type = PIDSourceType.kDisplacement;

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        type = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return type;
    }

    @Override
    public double pidGet() {
        return Drive.getInstance().getHeading().getDegrees();
    }
}
