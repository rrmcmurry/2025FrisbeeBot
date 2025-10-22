// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FrisbeeLauncher;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  
  private final DriveSubsystem drive = new DriveSubsystem();
  private final XboxController controller = new XboxController(0);
  private final FrisbeeLauncher frisbeelauncher = new FrisbeeLauncher();

 
  public RobotContainer() {
    configureBindings();
    CommandScheduler.getInstance().setDefaultCommand(drive, drive.driveCommand(controller, false));
  }


  private void configureBindings() {
    new Trigger(() -> controller.getAButton()).whileTrue(frisbeelauncher.fire());
  }

  public void thisdoesnothing() {}
}
