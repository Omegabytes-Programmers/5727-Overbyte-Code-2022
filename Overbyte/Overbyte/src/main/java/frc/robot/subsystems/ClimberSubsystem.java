// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private TalonFX leftClimberMotor;
  private TalonFX rightClimberMotor;

  private boolean leftClimberExtended = false;
  private boolean leftClimberRetracted = false;
  private boolean rightClimberExtended = false;
  private boolean rightClimberRetracted = false;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftClimberMotor = new TalonFX(Constants.leftClimberMotorPort);
    rightClimberMotor = new TalonFX(Constants.rightClimberMotorPort);
  }

  public void extend(double speed){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 1.0);
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, speed * -1.0);
    leftClimberMotor.getStatorCurrent();
    rightClimberMotor.getStatorCurrent();
  }

  public void retract(double speed){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, speed * -0.8);
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, speed * 0.8);
    leftClimberMotor.getStatorCurrent();
    rightClimberMotor.getStatorCurrent();
  }

  public void stop(){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, 0.0);
  }

  public void resetLeft(){
    leftClimberMotor.set(TalonFXControlMode.PercentOutput, -0.15);
  }  

  public void resetRight(){
    rightClimberMotor.set(TalonFXControlMode.PercentOutput, 0.15);
  }  

  @Override
  public void periodic() {
    if (Constants.driveController.getRawAxis(Constants.extendLiftAxis) > 0.1){
      extend(Constants.driveController.getRawAxis(Constants.extendLiftAxis));

    } else if (Constants.driveController.getRawAxis(Constants.retractLiftAxis) > 0.1){
      retract(Constants.driveController.getRawAxis(Constants.retractLiftAxis));

    }else if (Constants.resetController.getRawButton(Constants.resetLeftButton)){
      resetLeft();
    }else if (Constants.resetController.getRawButton(Constants.resetRightButton)){
      resetRight();
    }else{
      stop();
    }
    // This method will be called once per scheduler run
  }
}
