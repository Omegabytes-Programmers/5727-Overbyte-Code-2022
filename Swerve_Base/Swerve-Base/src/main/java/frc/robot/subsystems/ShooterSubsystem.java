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

public class ShooterSubsystem extends SubsystemBase {
  private DoubleSolenoid shooterSolenoids;
  private TalonFX topShooterMotor;
  private TalonFX bottomShooterMotor;
  private boolean canShoot;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(DoubleSolenoid shooterSolenoids) {
    this.shooterSolenoids = shooterSolenoids;
    topShooterMotor = new TalonFX(Constants.topShooterMotorPort);
    bottomShooterMotor = new TalonFX(Constants.bottomShooterMotorPort);
    canShoot = true;
  }

  @Override
  public void periodic() {
    if (canShoot){
      if (Constants.driveController.getRawButton(Constants.readyToShootButton)){
        topShooterMotor.set(TalonFXControlMode.PercentOutput, 1 - ((1 + Constants.topController.getRawAxis(3)) / 2));
        bottomShooterMotor.set(TalonFXControlMode.PercentOutput, -(1 - ((1 + Constants.bottomController.getRawAxis(3)) / 2)));
        shooterSolenoids.set(Value.kForward);
      }else{
        topShooterMotor.set(TalonFXControlMode.PercentOutput, 0);
        bottomShooterMotor.set(TalonFXControlMode.PercentOutput, 0);
        shooterSolenoids.set(Value.kReverse);
      }
    }
  }
}
