// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.omegabytes.ShooterConfiguration;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootCommand extends CommandBase {
  private VisionSubsystem vision;
  private PneumaticsSubsystem pneumatics;
  private ShooterSubsystem shooter;
  private StorageSubsystem storage;
  private IntakeSubsystem intake;
  private double distance;
  private boolean extendIntake;
  private Timer shootTimer;
  private Timer storageTimer;
  private Timer timeoutTimer;
  private Double timeoutThreshold;
  private boolean linedUp;
  private boolean tookSnapshot;
  private boolean useVision;

  /** Creates a new ShootCommand. */
  public ShootCommand(VisionSubsystem visionSubsystem, PneumaticsSubsystem pneumaticsSubsystem, ShooterSubsystem shooterSubsystem, StorageSubsystem storageSubsystem, IntakeSubsystem intakeSubsystem){
    this(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 0.0);
  }

  public ShootCommand(VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake, double distance){
    this(vision, pneumatics, shooter, storage, intake, 0.0, false);
  }

  public ShootCommand(VisionSubsystem vision, PneumaticsSubsystem pneumatics, ShooterSubsystem shooter, StorageSubsystem storage, IntakeSubsystem intake, double distance, boolean extendIntake){
    this.vision = vision;
    this.pneumatics = pneumatics;
    this.shooter = shooter;
    this.storage = storage;
    this.intake = intake;
    this.distance = distance;
    this.extendIntake = extendIntake;

    shootTimer = new Timer();
    storageTimer = new Timer();
    timeoutTimer = new Timer();
    timeoutThreshold = RobotState.isAutonomous() ? 0.5 : 1.0;
    linedUp = false;
    tookSnapshot = false;
    useVision = (distance <= 0.0);

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

    timeoutThreshold = RobotState.isAutonomous() ? 0.5 : 1.0;
    linedUp = false;
    tookSnapshot = false;
    DataLogManager.log("init: use vision before?" + useVision);
    useVision = (useVision || (RobotState.isAutonomous() && vision.getVisionAuto()));
    DataLogManager.log("init: use vision?" + useVision);

    if (useVision) {
      vision.takeSnapshot();
    } else {
      linedUp = true; // Pretend already aligned when vision is not to be used
      tookSnapshot = true; // And pretend already took a snapshot
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isTest()){
      double x = vision.getPosition();
      ShooterConfiguration config;

      boolean hasTarget;
      if (useVision) {
        DataLogManager.log("Config from vision");
        config = Constants.vsConversion.getValuesFromAngle(vision.getAngle(), shooter.isHoodUp());
      }else{
        DataLogManager.log("Config from distance: " + distance);
        config = Constants.vsConversion.getValuesFromDistance(distance, shooter.isHoodUp());
      }

      SmartDashboard.putNumber("Distance", config.getDistance());
      hasTarget = shooter.shoot(config);

      DataLogManager.log("Lined up?" + linedUp + "; X = " + x);
      if (useVision && Math.abs(x) >= 3.0 && !linedUp){
        storageTimer.reset();
        if(RobotState.isAutonomous() || Constants.driveController.getRawButton(Constants.readyToShootButton)){
          timeoutTimer.reset();
        }

        storage.stop();
        intake.stop();
        pneumatics.start();
      }else{
        if (!linedUp){
          linedUp = true;
          vision.resetSnapshot();
        }

        if (hasTarget){
          // Turn off pneumatics in preparation to shoot
          pneumatics.stop();

          // Check if we have a ball ready
          if (storage.getTopProxSensor()){
            // If we do, don't time out
            storageTimer.reset();
            timeoutTimer.reset();
          }else{
            // We must have shot the ball already (or we never had it?)

            // Take a snapshot after shooting the first ball
            if (!tookSnapshot && useVision){
              vision.takeSnapshot();
              tookSnapshot = true;
            }

            // Check if we have another ball waiting
            if (storage.getBottomProxSensor() || intake.getBeamBreakSensor()){
              // If we do, don't time out
              timeoutTimer.reset();
            }
          }

          // Ensure that all balls in the intake can make it into the robot
          intake.runIntake();

          // Ensure that we have given adequate time for the shooter wheel to spin up to speed
          if (shootTimer.get() >= 0.75){ // TODO Tune -- was 0.25 -- or specifically wait for velocity to be in range, perhaps using getSelectedSensorVelocity?
            storage.wheelFeed();

            // If chosen, extend intake (used to pick up a third ball)
            if (extendIntake){
              intake.extend();
            }
  
            // TODO: Dustin, why do we run the belt backwards first?  Does this timer even work?
            if (storageTimer.get() > 0.1){
              storage.beltFeed();
            }else{
              storage.beltReverse();
            }
          }else{
            // Not yet ready to feed shooter wheels
            storage.wheelStop();

            if (RobotState.isAutonomous()) {
              storage.beltStop();
              intake.retract();
            }
          }


        }else{
          DataLogManager.log("SHOOT: Don't have a target!");
          shootTimer.reset();
          storageTimer.reset();

          shooter.stop();
          storage.stop();
          intake.stop();
          if (RobotState.isAutonomous()) {
            intake.retract();
          }
          pneumatics.start();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //DataLogManager.log("Shooting is over; int = " + interrupted);
    if (!RobotState.isTest()){
      shooter.stop();
      storage.stop();
      intake.stop();
      if (RobotState.isAutonomous()) {
        intake.retract();
      }
      pneumatics.start();
    }

    vision.resetSnapshot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotState.isTeleop() && !Constants.driveController.getRawButton(Constants.readyToShootButton) && !Constants.manipController.getRawButton(Constants.overwriteShootCloseButton) && !Constants.manipController.getRawButton(Constants.overwriteShootFarButton)){
      timeoutThreshold = 0.25;
    }

    if (RobotState.isTest()){
      timeoutThreshold = -0.1;
    }
    
    DataLogManager.log("Timer:" + timeoutTimer.get());
    DataLogManager.log("Threshold:" + timeoutThreshold);
    return timeoutTimer.get() > timeoutThreshold;
  }
}
