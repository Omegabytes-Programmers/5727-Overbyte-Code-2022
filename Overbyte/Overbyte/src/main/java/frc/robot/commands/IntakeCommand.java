// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StorageSubsystem;

public class IntakeCommand extends CommandBase {
  private IntakeSubsystem intake;
  private StorageSubsystem storage;
  private Timer timeoutTimer;
  private Double timeoutTime;
  /** Creates a new IntakeCommand. */
  public IntakeCommand(IntakeSubsystem intake, StorageSubsystem storage) {
    this.intake = intake;
    this.storage = storage;

    timeoutTimer = new Timer();
    timeoutTime = 3.5;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!RobotState.isTest()){
      if (!(storage.getTopProxSensor() && (storage.getBottomProxSensor() || intake.getProxSensor()))){
        if (!intake.isExtended()){
          intake.extend();
          intake.runIntake();
        }
      }else{
        if (intake.isExtended()){
          intake.retract();
          intake.stopIntake();
        }
      }
    
      if (!Constants.driveController.getRawButton(Constants.readyToShootButton)){
        if (!storage.getTopProxSensor()){
          storage.wheelntake();
        }else{
          storage.wheelStop();
        }

        if (!storage.getBottomProxSensor()){
          storage.beltIntake();
        }else{
          storage.beltStop();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isTest()){
      if (!Constants.driveController.getRawButton(Constants.readyToShootButton)){
        if (intake.isExtended()){
          intake.retract();
          intake.stopIntake();
        }
        storage.stop();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!Constants.driveController.getRawButton(Constants.intakeButton)){
      timeoutTimer.start();
      if (intake.isExtended()){
        intake.retract();
        intake.stopIntake();
      }
    }

    if(((storage.getTopProxSensor() && (storage.getBottomProxSensor() || intake.getProxSensor()))) || RobotState.isTest()){
      timeoutTime = 0.0;
    }

    return timeoutTimer.get() >= timeoutTime;
  }
}
