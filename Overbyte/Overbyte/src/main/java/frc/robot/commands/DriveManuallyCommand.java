// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// conection terminated
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveManuallyCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final IntakeSubsystem intake;
  private final VisionSubsystem vision;
  private final Timer locateTimer;
  private double translationXPercent;
  private double translationYPercent;
  private double rotationPercent;

  public DriveManuallyCommand(DriveSubsystem drive, IntakeSubsystem intake, VisionSubsystem vision){
      this.drive = drive;
      this.intake = intake;
      this.vision = vision;
      addRequirements(drive);

      locateTimer = new Timer();
      locateTimer.start();
      
  }

    @Override
    public void execute() {
        translationXPercent = Constants.driveController.getRawAxis(1);
        translationYPercent = Constants.driveController.getRawAxis(0);
        rotationPercent = -Constants.driveController.getRawAxis(4);



        if (Math.abs(translationXPercent) < Constants.deadzone){
            translationXPercent = 0.0;
        }
        
        if (Math.abs(translationYPercent) < Constants.deadzone){
            translationYPercent = 0.0;
        }
        
        if (Math.abs(rotationPercent) < Constants.deadzone){
            rotationPercent = 0.0;
        }

        translationXPercent *= .8;
        translationYPercent *= .8;
        rotationPercent *= .4;

        if (rotationPercent != 0.0 || (intake.isIntaking() && locateTimer.get() < 1.5)){
            locateTimer.reset();
        }

        if ((Constants.driveController.getRawButton(Constants.readyToShootButton))) { // || (rotationPercent == 0.0 && locateTimer.get() >= 1.5)){
            double x = vision.getPosition();

            if (Math.abs(x) >= 3.0){
                rotationPercent = .1 * -Math.signum(x);
            }
        }

         
        drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXPercent * Constants.maxVelocity, 
                translationYPercent * Constants.maxVelocity,
                rotationPercent * Constants.maxAngularVelocity,
                (Constants.driveController.getRawButton(Constants.robotOrientButton) ? Rotation2d.fromDegrees(0.0) : drive.getGyroscopeRotation())
            )
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive
        drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}

