// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private StorageSubsystem storage;
  private boolean intaking = false;
  private boolean stopped = true;
  private TalonFX intakeMotor;
  private DoubleSolenoid intakeSolenoids;
  private DigitalInput rightProxSensor;
  private DigitalInput leftProxSensor;
  private Timer storageTimer;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(DoubleSolenoid intakeSolenoids, StorageSubsystem storage) {
    intakeMotor = new TalonFX(Constants.intakeMotorPort);
    this.intakeSolenoids = intakeSolenoids;
    this.storage = storage;
    rightProxSensor = new DigitalInput(Constants.intakeRightProxSensorPort);
    leftProxSensor = new DigitalInput(Constants.intakeLeftProxSensorPort);
    storageTimer = new Timer();
  }

  public boolean getProxSensor(){
    return !rightProxSensor.get() || !leftProxSensor.get(); // Inverting the value as the sensor returns true when no ball is in the way 
  }

  public boolean isExtended(){
    return (intakeSolenoids.get() == Value.kForward) ? true : false;
  }

  public void runIntake(){
    setIntakeSpeed(-0.75);
  }

  public void stopIntake(){
    setIntakeSpeed(0.0);
  }

  public void setIntakeSpeed(double speedPercent){
    intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void extend(){
    intakeSolenoids.set(Value.kForward);
  }

  public void retract(){
    intakeSolenoids.set(Value.kForward);
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
  }
}
