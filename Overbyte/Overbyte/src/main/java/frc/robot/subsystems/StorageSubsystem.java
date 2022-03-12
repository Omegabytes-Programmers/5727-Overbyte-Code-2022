// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class StorageSubsystem extends SubsystemBase {
  private TalonFX feederWheel;
  private TalonSRX feederBelt;
  private DigitalInput stopLidar;
  private Timer shootTimer;
  /** Creates a new StorageSubsystem. */
  public StorageSubsystem() {
    feederWheel = new TalonFX(Constants.storageWheelMotorPort);
    feederBelt = new TalonSRX(Constants.storageConveyorPort);
    stopLidar = new DigitalInput(Constants.stopLidarPort);
    shootTimer = new Timer();
  }


  public void feedShooter(){
    if (!stopLidar.get()){
      shootTimer.start();
      if (shootTimer.hasElapsed(.25)){
        feederWheel.set(TalonFXControlMode.PercentOutput, -0.25);
      }else{
        feederWheel.set(TalonFXControlMode.PercentOutput, 0.0);
      }
    }else{
      feederWheel.set(TalonFXControlMode.PercentOutput, -0.25);
    }
    feederBelt.set(TalonSRXControlMode.PercentOutput, -0.25);
  }

  public void stop(){
    feederWheel.set(TalonFXControlMode.PercentOutput, 0.0);
    feederBelt.set(TalonSRXControlMode.PercentOutput, 0.0);
    shootTimer.stop();
    shootTimer.reset();
  }

  public void intake(){
    if (stopLidar.get()){
      feederWheel.set(TalonFXControlMode.PercentOutput, -0.1);
    }else{
      feederWheel.set(TalonFXControlMode.PercentOutput, 0.0);
    }
    feederBelt.set(TalonSRXControlMode.PercentOutput, -0.5);
  }
// Coconut.jpg
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if (Constants.driveController.getRawButton(Constants.readyToShootButton)){
      shootTimer.start();
      if (shootTimer.get() > 0.5){
        feedShooter();
      }
    }else{
      stop();
    }*/
  }
}
