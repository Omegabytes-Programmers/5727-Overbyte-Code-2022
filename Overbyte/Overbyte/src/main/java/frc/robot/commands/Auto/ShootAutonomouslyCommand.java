// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootAutonomouslyCommand extends SequentialCommandGroup {
  public ShootAutonomouslyCommand(DriveSubsystem drive, VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake){
    this(drive, vision, pneumatics, shooter, storage, intake, -1.0);
  }

  public ShootAutonomouslyCommand(DriveSubsystem drive, VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake, double distance){
    this(drive, vision, pneumatics, shooter, storage, intake, distance, false);
  }

  public ShootAutonomouslyCommand(DriveSubsystem drive, VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake, boolean extendIntake){
    this(drive, vision, pneumatics, shooter, storage, intake, -1.0, extendIntake);
  }

  public ShootAutonomouslyCommand(DriveSubsystem drive, VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake, double distance, boolean extendIntake){
    addCommands(
      new InstantCommand(() -> drive.enableAutoDrive()),
      new ParallelRaceGroup(
        new ShootCommand(vision, pneumatics, shooter, storage, intake, distance, extendIntake),
        new ConditionalCommand(
          new DriveManuallyCommand(drive, intake, vision, true),
          new WaitCommand(15.0),
          () -> vision.getVisionAuto())
      ),
      new InstantCommand(() -> drive.disableAutoDrive())
    );
  }
}