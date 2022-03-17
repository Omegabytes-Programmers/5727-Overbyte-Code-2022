// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto3Ball extends SequentialCommandGroup {
  public Auto3Ball(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new Auto2Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem),
      new DriveAutonomouslyCommand(
        driveSubsystem,
        Constants.autoPoseBall3
      ),
      new ParallelCommandGroup(
        new AutoCommand(
          driveSubsystem,
          1.3
        ),
        new IntakeAutonomouslyCommand(
          intakeSubsystem,
          storageSubsystem,
          2.3
        )
      ),
      new DriveAutonomouslyCommand(
        driveSubsystem,
        Constants.autoPoseShoot3
      ),
      new AimToShootCommand(
        driveSubsystem,
        visionSubsystem,
        0.5
      ),
      new ShootAutonomouslyCommand(
        visionSubsystem,
        pneumaticsSubsystem,
        shooterSubsystem,
        storageSubsystem,
        intakeSubsystem
      ),
      new DriveAutonomouslyCommand(
        driveSubsystem,
        Constants.autoPoseBall4
      )
    );
  }
}
