// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  

  private boolean intaking = false;
  private TalonFX intakeMotor;
  private DoubleSolenoid intakeSolenoids;
  private DigitalInput rightProxSensor;
  private DigitalInput leftProxSensor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(DoubleSolenoid intakeSolenoids) {
    intakeMotor = new TalonFX(Constants.intakeMotorPort);
    this.intakeSolenoids = intakeSolenoids;
    rightProxSensor = new DigitalInput(Constants.intakeRightProxSensorPort);
    leftProxSensor = new DigitalInput(Constants.intakeLeftProxSensorPort);

    retract();
    stopIntake();
  }

  public boolean getProxSensor(){
    return !rightProxSensor.get() || !leftProxSensor.get(); // Inverting the value as the sensor returns true when no ball is in the way 
  }

  public boolean isExtended(){
    return (intakeSolenoids.get() == Value.kForward) ? true : false;
  }

  public void runIntake(){
    setIntakeSpeed(-0.75);
    intaking = true;
  }

  public void reverseIntake(){
    setIntakeSpeed(1.0);
  }

  public void stopIntake(){
    setIntakeSpeed(0.0);
    intaking = false;
  }

  public void setIntakeSpeed(double speedPercent){
    intakeMotor.set(TalonFXControlMode.PercentOutput, speedPercent);
  }

  public void extend(){
    intakeSolenoids.set(Value.kForward);
  }

  public void retract(){
    intakeSolenoids.set(Value.kReverse);
  }

  public void stop(){
    retract();
    stopIntake();
  }

  public boolean isIntaking(){
    return intaking;
  }


  @Override
  public void periodic() {
    //System.out.println("Intake: " + (getProxSensor() ? "Has " : "No ") + "Ball");
  }
}
