package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class FrisbeeLauncher extends SubsystemBase {

    public static SparkMaxConfig DefaultConfig = new SparkMaxConfig();    
    private SparkMax FrisbeeLaunchMotor;
    public static final int kFrisbeeLaunchMotorCanID = 9;
    private SparkMax FrisbeeLoadingMotor;
    public static final int kFrisbeeLoadingMotorCanID = 10;
    
    static {
        DefaultConfig.smartCurrentLimit(50);
        DefaultConfig.idleMode(IdleMode.kCoast);
        DefaultConfig.openLoopRampRate(2.0);
        DefaultConfig.inverted(true);
    }

    public FrisbeeLauncher() {
        FrisbeeLaunchMotor = new SparkMax(kFrisbeeLaunchMotorCanID, MotorType.kBrushless);
        FrisbeeLoadingMotor = new SparkMax(kFrisbeeLoadingMotorCanID, MotorType.kBrushless);
        FrisbeeLaunchMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        FrisbeeLoadingMotor.configure(DefaultConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command fire() {
        return Commands.sequence(
            Commands.runOnce(() -> FrisbeeLaunchMotor.set(1)),
            Commands.waitSeconds(3.0),
            Commands.runOnce(() -> FrisbeeLoadingMotor.set(1)),
            Commands.waitSeconds(2.0)
        ).finallyDo(() -> {
            FrisbeeLaunchMotor.stopMotor();
            FrisbeeLoadingMotor.stopMotor();
        });
    }
}
