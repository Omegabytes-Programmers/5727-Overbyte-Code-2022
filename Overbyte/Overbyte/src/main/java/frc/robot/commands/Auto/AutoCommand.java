// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoCommand extends CommandBase {
  private DriveSubsystem drive;
  private VisionSubsystem vision;
  private PneumaticsSubsystem pneumatics;
  private ShooterSubsystem shooter;
  private StorageSubsystem storage;
  private IntakeSubsystem intake;
  private Timer autoTimer;

  /** Creates a new AutoCommand. */
  public AutoCommand(DriveSubsystem drive, VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake) {
    this.drive = drive;
    this.vision = vision;
    this.pneumatics = pneumatics;
    this.shooter = shooter;
    this.storage = storage;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(vision);
    addRequirements(pneumatics);
    addRequirements(shooter);
    addRequirements(storage);
    addRequirements(intake);
    
    autoTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.start();
    autoTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationXPercent = 0.0;
    double translationYPercent = 0.0;
    double rotationPercent = 0.0;

    System.out.println(autoTimer.get());
    if (autoTimer.get() < 0.35){
      translationXPercent = 0.4;
    }

    if (autoTimer.get() < 1.7){
      intake.runIntake();
      intake.extend();
    }

    if (autoTimer.get() > 1.7 && autoTimer.get() < 1.8){
      intake.stopIntake();
      intake.retract();
    }

    if (autoTimer.get() > 1.8 && autoTimer.get() < 3.0){
      double x = vision.getPosition();
      
      if (Math.abs(x) >= 3.0){
        rotationPercent = .1 * -Math.signum(x);
      }
    }

    if (autoTimer.get() > 3.0 && autoTimer.get() < 6.0){
      shooter.shoot(Constants.vsConversion.getValuesFromAngle(vision.getAngle(), shooter.isHoodUp()));
      intake.runIntake();
      storage.wheelntake();
      storage.beltIntake();
    }

    if (autoTimer.get() > 6.0 && autoTimer.get() < 15.0){
      shooter.stop();
      intake.stop();
      storage.stop();

      //TODO: Fix this to rotate to the proper angle

    }



    drive.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
          translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
          translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
          rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
          drive.getRotation()
      ), true
  );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
