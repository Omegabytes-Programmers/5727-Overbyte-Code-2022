// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveManuallyCommand extends CommandBase {
  private final DriveSubsystem drivetrain;

  public DriveManuallyCommand(DriveSubsystem drivetrain){
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);

      
  }

    @Override
    public void execute() {
        double translationXPercent = Constants.driveController.getRawAxis(1);
        double translationYPercent = Constants.driveController.getRawAxis(0);
        double rotationPercent = Constants.driveController.getRawAxis(4);

        if (Math.abs(translationXPercent) < Constants.deadzone){
        translationXPercent = 0.0;
        }
        if (Math.abs(translationYPercent) < Constants.deadzone){
            translationYPercent = 0.0;
        }
        if (Math.abs(rotationPercent) < Constants.deadzone){
            rotationPercent = 0.0;
        }

        translationXPercent *= .65;
        translationYPercent *= .65;
        rotationPercent *= .5;

        drivetrain.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
                translationYPercent * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
                rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
                drivetrain.getRotation()
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}

