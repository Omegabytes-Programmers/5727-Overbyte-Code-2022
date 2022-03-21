// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveAutonomouslyCommand extends CommandBase {
  private DriveSubsystem drive;

  
  private PIDController translationXController = new PIDController(1.2, 0, 0); //10
  private PIDController translationYController = new PIDController(1.2, 0, 0);
  private PIDController rotationController = new PIDController(1.2, 0, 0); // 1.2

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

  private Timer timeoutTimer;
  private int inPlace = 0;

  /** Creates a new DriveAutonomouslyCommand. */
  public DriveAutonomouslyCommand(DriveSubsystem drive, Pose2d endPose) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    timeoutTimer = new Timer();

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
    //System.out.println(targetX);
    //System.out.println(targetY);

    currentPose = drive.getPose();

    currentX = currentPose.getTranslation().getX();
    currentY = currentPose.getTranslation().getY();

    currentRotation = currentPose.getRotation().getRadians();
    
    translationXPercent = 0.1 + translationXController.calculate(currentX);
    translationYPercent = 0.1 + translationYController.calculate(currentY);
    rotationPercent = 0.1 + rotationController.calculate(currentRotation);
    double rotationPercentMin = 0.10;
    if (Math.abs(rotationPercent) < rotationPercentMin) {
      rotationPercent = rotationPercentMin * Math.signum(rotationPercent);
    }

    System.out.println("Angle Error: " + Math.abs(currentRotation - targetRotation) + "; Rotation percent = " + rotationPercent);

    drive.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(
          0 * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
          0 * DriveSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
          rotationPercent * DriveSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 
          drive.getRotation()
      ), false
    );


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(0, 0, 0), false);
    System.out.println("Reached end of auto drive");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Current Angle: " + currentRotation + "; Target = " + targetRotation);
    if (Math.abs(currentRotation - targetRotation) < 0.05){
      System.out.println("In place");
      inPlace++;
    }else{
      inPlace = 0;
    }
    return inPlace > 5 || timeoutTimer.get() > 2.5;
  }
}