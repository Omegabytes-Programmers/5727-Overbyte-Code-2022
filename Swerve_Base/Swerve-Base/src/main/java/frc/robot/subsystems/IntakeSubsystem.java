// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private boolean canIntake;
  private TalonFX intakeMotor;
  private DoubleSolenoid intakeSolenoids;
  
  public void start(){
    canIntake = true;
  }

  public void stop(){
    canIntake = false;
  }

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(DoubleSolenoid intakeSolenoids) {
    intakeMotor = new TalonFX(Constants.intakeMotorPort);
    this.intakeSolenoids = intakeSolenoids;
  }

  @Override
  public void periodic() {
    if (true){
      if (Constants.driveController.getRawButton(Constants.IntakeButton)){
        intakeMotor.set(TalonFXControlMode.PercentOutput, 1);
        intakeSolenoids.set(Value.kForward);
      }else{
        intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
        intakeSolenoids.set(Value.kReverse);
      }
    }
  }
}
