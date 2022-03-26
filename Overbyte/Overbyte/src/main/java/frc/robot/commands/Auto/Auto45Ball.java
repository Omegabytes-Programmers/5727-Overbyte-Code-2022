// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto45Ball extends SequentialCommandGroup {
  public Auto45Ball(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new Auto3Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),
      new ParallelCommandGroup(
        new DriveAutonomouslyCommand(
          driveSubsystem,
          Constants.autoPoseBall45,
          7
        ),
        new IntakeAutonomouslyCommand(
          intakeSubsystem,
          storageSubsystem,
          false,
          4.0
        )
      ),
      new DriveAutonomouslyCommand(
        driveSubsystem,
        Constants.autoPoseShoot2,
        3.0
      ),
      new AimToShootCommand(
        driveSubsystem, 
        visionSubsystem, 
        0.75
      ),
      new ShootAutonomouslyCommand(
        visionSubsystem,
        pneumaticsSubsystem,
        shooterSubsystem,
        storageSubsystem,
        intakeSubsystem
      )
    );
  }
}
