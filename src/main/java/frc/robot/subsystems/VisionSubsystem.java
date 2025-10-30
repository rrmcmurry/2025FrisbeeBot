package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.photonvision.PhotonCamera;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    boolean targetVisible;
    double targetYaw;
    double targetArea;
    double targetPitch;
    boolean aligned;
    double deadband = 2.0;

    public VisionSubsystem() {
        camera = new PhotonCamera("FrontLeftCamera");
        aligned = false;
    }


    public boolean isAligned() {
        return aligned;
    }

    public Command reset() {
        return Commands.runOnce(() -> aligned = false);
    }

    @Override
    public void periodic() {
        UpdateVision();
        aligned = Math.abs(targetYaw) < deadband;

    }


    public void UpdateVision() {
        targetVisible = false;
        targetYaw = 360.0;
        targetArea = 0.0;
        targetPitch = 0.0;

        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            var result = results.get(results.size() -1);
            if (result.hasTargets()){
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 1) {
                        targetVisible = true;
                        targetYaw = target.getYaw();
                        targetArea = target.getArea();
                        targetPitch = target.getPitch();
                    }
                }
            }
        }
    }

    public double CalculateDriveRotation() {

        if (!targetVisible || aligned) return 0;
        
        // Clamp yaw and convert to rotation (power) 
        double maxYaw = 30.0;
        double maxPower = 0.25;
        double clampedYaw = Math.max(-maxYaw, Math.min(maxYaw, targetYaw));
        double rotation = (clampedYaw / maxYaw) * maxPower;        

        return rotation;
    }

    public Command align(DriveSubsystem drive) {
        return Commands.run(() -> drive.drive(0,0,CalculateDriveRotation(),false),drive);
    }

    public void robotInit() {        
        camera.getAllUnreadResults(); // warm-up
    }
}
