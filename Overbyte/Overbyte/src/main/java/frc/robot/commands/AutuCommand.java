// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutuCommand extends CommandBase {
  private final DriveSubsystem drivetrain;
  private final VisionSubsystem vision;
  private final IntakeSubsystem intake;
  private final StorageSubsystem storage;
  private final ShooterSubsystem shooter;
  /** Creates a new AutuCommand. */
  public AutuCommand(DriveSubsystem drivetrain, VisionSubsystem vision, IntakeSubsystem intake, StorageSubsystem storage, ShooterSubsystem shooter) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.intake = intake;
    this.storage = storage;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    //addRequirements(vision);
    //addRequirements(intake);
    //addRequirements(shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {






      double translationXPercent = .25;
      double translationYPercent = 0;
      double rotationPercent = 0;

      //translationXPercent *= .65;
      //translationYPercent *= .65;
      //rotationPercent *= .5;

      if (Timer.getMatchTime() > 13.8){
        intake.runIntake();
        intake.extend();
      }else{
        translationXPercent = 0;
      }
      System.out.println(Timer.getMatchTime());


      if (Timer.getMatchTime() < 12 && Timer.getMatchTime() > 10){
        intake.stop();
        double x = vision.getPosition();


        //System.out.println("It is hitting");


        if (Math.abs(x) >= 3.0){
            rotationPercent = .15 * -Math.signum(x);
        }
      }

      if (Timer.getMatchTime() < 10 && Timer.getMatchTime() > 6){
        double y = vision.getAngle();
        shooter.shoot(Constants.vsConversion.getValuesFromAngle(y, shooter.isHoodUp()));

        if (Timer.getMatchTime() < 8){
          storage.beltFeed();
          storage.wheelFeed();
        }else{
          storage.beltStop();
          storage.wheelStop();
        }
        
      }else{
        shooter.stop();
        storage.stop();
      }

      System.out.println(translationXPercent);
      drivetrain.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
              translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
              rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
              drivetrain.getRotation()
          ), true
      );
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
