// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimToShootCommand extends CommandBase {
  private final DriveSubsystem drive;
  private final VisionSubsystem vision;

  private double time;
  private Timer timer;

  private double x;

  /** Creates a new AimToShootCommand. */
  public AimToShootCommand(DriveSubsystem drive, VisionSubsystem vision, double time) {
    this.drive = drive;
    this.vision = vision;
    this.time = time;
    x = 0.0;
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = vision.getPosition();

    double rotationPercent = 0;
    if (Math.abs(x) >= 3.0){
        rotationPercent = .1 * -Math.signum(x);
    }

    drive.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                0 * Constants.maxVelocity, 
                0 * Constants.maxVelocity, 
                rotationPercent * Constants.maxAngularVelocity, 
                drive.getRotation()
            )
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
