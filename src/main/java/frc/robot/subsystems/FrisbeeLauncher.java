package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class FrisbeeLauncher extends SubsystemBase {

    public static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax FrisbeeLaunchMotor;    
    private SparkMax FrisbeeLoadingMotor;
    public static final int kFrisbeeLaunchMotorCanID = 10;
    public static final int kFrisbeeLoadingMotorCanID = 9;
    private SparkAbsoluteEncoder flywheelencoder;
    public boolean loaded;

    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(1.0);
        DefaultConfig.inverted(true);
    }

    public FrisbeeLauncher() {
        FrisbeeLaunchMotor = new SparkMax(kFrisbeeLaunchMotorCanID, MotorType.kBrushless);
        FrisbeeLoadingMotor = new SparkMax(kFrisbeeLoadingMotorCanID, MotorType.kBrushless);
        FrisbeeLaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        FrisbeeLoadingMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);        
        flywheelencoder = FrisbeeLaunchMotor.getAbsoluteEncoder();
        loaded = true;
    }

    public Command reload() {
        return Commands.runOnce(() -> loaded = true);
    }

    public Command fire() {
        if (!loaded) return Commands.none();
        
        return Commands.sequence(
            Commands.runOnce(() -> FrisbeeLaunchMotor.set(1)), // Set Frisbee launch motor to full speed
            // Commands.waitUntil(() -> flywheelencoder.getVelocity() > 100), // Wait until we're at 5000 RPM
            Commands.waitSeconds(1.0), // Wait another half a second
            Commands.runOnce(() -> FrisbeeLoadingMotor.set(1)), // Load the frisbee
            Commands.runOnce(() -> loaded = false),
            Commands.waitSeconds(1.0) // Fire 
        ).finallyDo(() -> {
            FrisbeeLaunchMotor.stopMotor(); // let the motors coast
            FrisbeeLoadingMotor.stopMotor();
        });
    }
}
