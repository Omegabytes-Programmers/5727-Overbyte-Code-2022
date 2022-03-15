// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ResetRightClimberCommand extends CommandBase {
  private ClimberSubsystem climber;
  /** Creates a new ResetLeftClimberCommand. */
  public ResetRightClimberCommand(ClimberSubsystem climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    if (!RobotState.isTest()){
      climber.resetRight();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!RobotState.isTest()){
      climber.stopRight();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!Constants.driveController.getRawButton(Constants.resetRightButton) || RobotState.isTest());
  }
}
