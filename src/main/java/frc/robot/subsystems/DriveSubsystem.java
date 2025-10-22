// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  
    public static final class DriveConstants {

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kFrontRightDrivingCanId = 2;
        public static final int kRearRightDrivingCanId = 3;
        public static final int kRearLeftDrivingCanId = 4;    

        public static final int kFrontLeftTurningCanId = 5;    
        public static final int kFrontRightTurningCanId = 6;
        public static final int kRearRightTurningCanId = 7;
        public static final int kRearLeftTurningCanId = 8;  

        // Chassis configuration
        public static final double kWheelBase = Units.inchesToMeters(26.5);
        public static final double kTrackWidth = Units.inchesToMeters(26.5);

        // Driving Parameters 
        public static final double kMaxSpeedMetersPerSecond = 4.8; // Default is 4.8 meters per second     
        public static final double kMaxAngularSpeed = 2 * Math.PI; // Default is 2 PI radians (one full rotation) per second 

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;
      
    }  

    // Create 4 instances of SwerveModules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDrivingCanId,
        DriveConstants.kFrontLeftTurningCanId,
        DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.kFrontRightDrivingCanId,
        DriveConstants.kFrontRightTurningCanId,
        DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule m_rearLeft = new SwerveModule(
        DriveConstants.kRearLeftDrivingCanId,
        DriveConstants.kRearLeftTurningCanId,
        DriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule m_rearRight = new SwerveModule(
        DriveConstants.kRearRightDrivingCanId,
        DriveConstants.kRearRightTurningCanId,
        DriveConstants.kBackRightChassisAngularOffset);

    // Create NavX AHRS Gyroscope
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);


    /**
     * Method to drive the robot using joystick info.
     *
     * @param forward       Speed of the robot in the forward, forward positive.
     * @param strafe        Speed of the robot in the sideways direction, left negative.
     * @param rotation      Rate of the robot rotation, left negative.
     * @param fieldRelative Whether the provided speeds are relative to the field.   
     */
    public void drive(double forward, double strafe, double rotation, boolean fieldRelative) {

        // Convert the commanded speeds into the correct units for the drivetrain, make strafe and rotation left positive numbers as expected for swerve
        double forwardDelivered = forward * DriveConstants.kMaxSpeedMetersPerSecond;
        double strafeDelivered = -strafe * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = -rotation * DriveConstants.kMaxAngularSpeed;
        double currentangle = -m_gyro.getAngle() % 360;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(forwardDelivered, strafeDelivered, rotDelivered, Rotation2d.fromDegrees(currentangle))
                : new ChassisSpeeds(forwardDelivered, strafeDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);

    }

    public void reset(){
        m_gyro.zeroYaw();
    }

    public float getHeading() {// Assuming this returns yaw in degrees
        return m_gyro.getYaw();
    }

    public void stop(){
        this.drive(0,0,0,true);
    }

    public Command driveCommand(XboxController controller, boolean fieldRelative){
        return Commands.run(
            () -> {
                double forward = MathUtil.applyDeadband(-controller.getLeftY() * 0.2, 0.020);
                double strafe = MathUtil.applyDeadband(controller.getLeftX() * 0.2, 0.02);
                double rotate = MathUtil.applyDeadband(controller.getRightX() * 0.2, 0.02);
                this.drive(forward, strafe, rotate, fieldRelative);
            }
            , this);
    } 

}
