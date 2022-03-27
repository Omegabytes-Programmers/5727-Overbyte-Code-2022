// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAutonomouslyCommand extends CommandBase {
  private DriveSubsystem drive;

  
  
  private PIDController translationXController = new PIDController(0.9, 0, 0); //10
  private PIDController translationYController = new PIDController(0.9, 0, 0);
  private PIDController rotationController = new PIDController(0.5, 0, 0); // 1.2

  private double targetX;
  private double targetY;
  private double targetRotation;

  private double currentX;
  private double currentY;
  private double currentRotation;

  private Pose2d currentPose;

  private double translationXPercent;
  private double translationYPercent;
  private double rotationPercent;

  private double translationXError = 1000;
  private double translationYError = 1000;
  private double rotationError = 1000;

  private Timer timeoutTimer = new Timer();
  private double expectedTime = 0.0;

  /** Creates a new DriveAutonomouslyCommand. */
  public DriveAutonomouslyCommand(DriveSubsystem drive, Pose2d endPose, double expectedTime) {
    this.drive = drive;
    this.expectedTime = expectedTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    targetX = endPose.getTranslation().getX();
    targetY = endPose.getTranslation().getY();
    targetRotation = endPose.getRotation().getRadians();

    translationXController.setSetpoint(targetX); 
    translationYController.setSetpoint(targetY);
    rotationController.setSetpoint(targetRotation);
       
    rotationController.enableContinuousInput(0, 2 * Math.PI);

    currentPose = drive.getPose();

    currentX = currentPose.getTranslation().getX();
    currentY = currentPose.getTranslation().getY();
    currentRotation = currentPose.getRotation().getRadians();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeoutTimer.start();
    timeoutTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ////System.out.println(targetX);
    ////System.out.println(targetY);

    currentPose = drive.getPose();

    currentX = currentPose.getTranslation().getX();
    currentY = currentPose.getTranslation().getY();

    currentRotation = currentPose.getRotation().getRadians();

    translationXError = (currentX - targetX);
    translationYError = (currentY - targetY);
    rotationError = (currentRotation - targetRotation);
    
    translationXPercent = (Math.signum(translationXError) * Constants.translationFeedForward) + translationXController.calculate(currentX);
    translationYPercent = (Math.signum(translationYError) * Constants.translationFeedForward) + translationYController.calculate(currentY);
    rotationPercent = (Math.signum(rotationError) * Constants.rotationFeedForward) + rotationController.calculate(currentRotation);

    //System.out.println(translationXPercent);
    //System.out.println(translationYPercent);
    //System.out.println(rotationPercent);

    /*double rotationPercentMin = 0.10;           // With the feedforward value this may not be needed
    if (Math.abs(rotationPercent) < rotationPercentMin) {
      rotationPercent = rotationPercentMin * Math.signum(rotationPercent);
    }*/

    //System.out.println("Angle Error: " + Math.abs(currentRotation - targetRotation) + "; Rotation percent = " + rotationPercent);

    drive.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
          translationXPercent * Constants.maxVelocity, 
          translationYPercent * Constants.maxVelocity, 
          rotationPercent * Constants.maxAngularVelocity, 
          drive.getRotation()
      )
    );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(0, 0, 0));
    //System.out.println("Reached end of auto drive");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(translationXPercent) <= 0.1 && Math.abs(translationYPercent) <= 0.1 && Math.abs(rotationPercent) <= 0.1) || (timeoutTimer.get() > expectedTime));
  }
}