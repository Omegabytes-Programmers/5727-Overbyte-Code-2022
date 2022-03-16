// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootAutonomouslyCommand extends CommandBase {
  private VisionSubsystem vision;
  private PneumaticsSubsystem pneumatics;
  private ShooterSubsystem shooter;
  private StorageSubsystem storage;
  private IntakeSubsystem intake;
  private Timer shootTimer;
  private Timer storageTimer;
  private Timer timeoutTimer;
  private Double timeoutThreshold;
  /** Creates a new ShootAutonomouslyCommand. */
  public ShootAutonomouslyCommand(VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake){
    this.vision = vision;
    this.pneumatics = pneumatics;
    this.shooter = shooter;
    this.storage = storage;
    this.intake = intake;

    shootTimer = new Timer();
    storageTimer = new Timer();
    timeoutTimer = new Timer();
    timeoutThreshold = 1.0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(shooter);
    addRequirements(storage);
    addRequirements(intake);
  }
   
  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    shootTimer.reset();
    storageTimer.reset();
    timeoutTimer.reset();

    shootTimer.start();
    storageTimer.start();
    timeoutTimer.start();

    timeoutThreshold = .5;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isTest()){
      boolean hasTarget = shooter.shoot(Constants.vsConversion.getValuesFromAngle(vision.getAngle(), shooter.isHoodUp()));

      if (hasTarget){
        pneumatics.stop();

        if (storage.getTopProxSensor()){
          storageTimer.reset();
          timeoutTimer.reset();
        }else{
          if (storage.getBottomProxSensor() || intake.getProxSensor()){
            timeoutTimer.reset();
          }
        }

        intake.runIntake();

        if (shootTimer.get() >= 0.1){
          storage.wheelFeed();
          
          if (storageTimer.get() > 0.2){
            storage.beltFeed();
          }else{
            storage.beltReverse();
          }
        }else{
          storage.wheelStop();
          storage.beltStop();
        }


      }else{
        shootTimer.reset();
        storageTimer.reset();

        shooter.stop();
        storage.stop();
        intake.stop();
        pneumatics.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isTest()){
      shooter.stop();
      storage.stop();
      intake.stop();
      pneumatics.start();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return timeoutTimer.get() > timeoutThreshold;
  }
}
