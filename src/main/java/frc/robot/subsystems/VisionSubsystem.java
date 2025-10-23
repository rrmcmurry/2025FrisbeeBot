package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;

    public VisionSubsystem() {
        camera = new PhotonCamera("FrontLeftCamera");
    }

    public void robotInit() {
        camera.getLatestResult(); // warm-up
    }
}
