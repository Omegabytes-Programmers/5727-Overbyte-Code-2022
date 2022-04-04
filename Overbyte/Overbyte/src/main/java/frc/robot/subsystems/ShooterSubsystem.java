// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.omegabytes.ShooterConfiguration;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private DoubleSolenoid shooterSolenoids;

  private TalonFX topShooterMotor;
  private TalonFX bottomShooterMotor;


  
  private double topValue;
  private double bottomValue;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(DoubleSolenoid shooterSolenoids) {
    this.shooterSolenoids = shooterSolenoids;
    topShooterMotor = new TalonFX(Constants.topShooterMotorPort);
    bottomShooterMotor = new TalonFX(Constants.bottomShooterMotorPort);
  }
 
  public boolean shoot(ShooterConfiguration shooterConfig){



    if (shooterConfig.getDistance() != 0.0){

      
      topShooterMotor.set(TalonFXControlMode.PercentOutput, shooterConfig.getTopMotorSpeed());
      bottomShooterMotor.set(TalonFXControlMode.PercentOutput, shooterConfig.getBottomMotorSpeed());

      // For testing purposes.

      topShooterMotor.set(TalonFXControlMode.Velocity, shooterConfig.getTopMotorSpeed());
      bottomShooterMotor.set(TalonFXControlMode.Velocity, shooterConfig.getBottomMotorSpeed());
      

      shooterSolenoids.set(shooterConfig.isHoodUp() ? Value.kForward : Value.kReverse); //
    }else{
      topShooterMotor.set(TalonFXControlMode.PercentOutput, 0.0);
      bottomShooterMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    return shooterConfig.getDistance() != 0.0;
  }

  public void stop(){
    topShooterMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    bottomShooterMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    
  }

  public boolean isHoodUp(){
    return (shooterSolenoids.get() == Value.kForward) ? true : false;
  }

  @Override
  public void periodic() {
    topValue = (1 - ((1 + Constants.topController.getRawAxis(3)) / 2)) * 6380;
    bottomValue = (-(1 - ((1 + Constants.bottomController.getRawAxis(3)) / 2))) * 6380;
  
    SmartDashboard.putNumber("Top Shooter", topValue);
    SmartDashboard.putNumber("Bottom Shooter", bottomValue);
  }
}
