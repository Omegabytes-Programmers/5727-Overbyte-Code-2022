// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class IntakeAutonomouslyCommand extends CommandBase {
  private IntakeSubsystem intake;
  private StorageSubsystem storage;
  private Timer timeoutTimer;
  private Double timeoutThreshold;
  
  /** Creates a new IntakeAutonomouslyCommand. */
  public IntakeAutonomouslyCommand(IntakeSubsystem intake, StorageSubsystem storage, double time) {
    this.intake = intake;
    this.storage = storage;

    this.timeoutThreshold = time;
    timeoutTimer = new Timer();
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeoutTimer.reset();
    timeoutTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    intake.runIntake();
    if (!(storage.getTopProxSensor() && (storage.getBottomProxSensor() || intake.getProxSensor()))){
      if (!intake.isExtended()){
        intake.extend();
        
      }
    }else{
      if (intake.isExtended()){
        intake.retract();
      }
    }
    
    if (!storage.getTopProxSensor()){
      storage.wheelntake();
    }else{
      storage.wheelStop();
    }

    if (!storage.getBottomProxSensor()){
      storage.beltIntake();
    }else{
      if (storage.getTopProxSensor()){
        storage.beltStop();
      }else{
        storage.beltIntake();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    
    intake.retract();
    intake.stopIntake();
    storage.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){

    if((storage.getTopProxSensor() && (storage.getBottomProxSensor() || intake.getProxSensor()))){
      timeoutThreshold = -0.1;
    }


    return timeoutTimer.get() >= timeoutThreshold;
  }
}
