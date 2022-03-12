// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private StorageSubsystem storage;
  private boolean canIntake = true;
  private boolean intaking = false;
  private boolean stopped = true;
  private TalonFX intakeMotor;
  private DoubleSolenoid intakeSolenoids;
  private Timer storageTimer;
  
  public void start(){
    canIntake = true;
  }

  public void stop(){
    if (!stopped){
      storageTimer.start();
      intakeMotor.set(TalonFXControlMode.PercentOutput, 0);  
      intakeSolenoids.set(Value.kReverse);
      intaking = false;
      if (storageTimer.hasElapsed(1.5)){
        storage.stop();
        stopped = true;
      }else{
        storage.intake();
      }
    }
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(DoubleSolenoid intakeSolenoids, StorageSubsystem storage) {
    intakeMotor = new TalonFX(Constants.intakeMotorPort);
    this.intakeSolenoids = intakeSolenoids;
    this.storage = storage;
    storageTimer = new Timer();
  }

  public boolean isIntaking(){
    return intaking;
  }

  public void intake(){
    intakeMotor.set(TalonFXControlMode.PercentOutput, -.75);
    intakeSolenoids.set(Value.kForward);
    intaking = true;
    stopped = false;
    storage.intake();
    storageTimer.stop();
    storageTimer.reset();
  }

  @Override
  public void periodic() {
    if (canIntake){
      if (Constants.driveController.getRawButton(Constants.intakeButton)){
        intake();
      }else{
        stop();
      }
    }
  }
}
