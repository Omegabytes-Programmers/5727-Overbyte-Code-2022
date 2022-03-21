// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Auto2BallLeft extends SequentialCommandGroup {
  public Auto2BallLeft(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new InstantCommand(()->driveSubsystem.zeroGyroscope(0)),
      new PrintCommand("DEBUG: Ready"),
      new DriveAutonomouslyCommand(
        driveSubsystem,
        Constants.autoPoseBall2Left
      ),
      new PrintCommand("DEBUG: In autoPoseBall2"),
      new ParallelCommandGroup(
        new AutoCommand(
          driveSubsystem,
          0.5
        ),
        new IntakeAutonomouslyCommand(
          intakeSubsystem,
          storageSubsystem,
          4.0
        )
      ),
      new AimToShootCommand(
        driveSubsystem,
        visionSubsystem,
        1.0
      ),
      new PrintCommand("DEBUG: Moved to shoot"),
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