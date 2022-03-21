// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import frc.robot.commands.ExampleCommand;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Auto.Auto2Ball;
import frc.robot.commands.Auto.Auto2BallLeft;
import frc.robot.commands.Auto.Auto2BallRight;
import frc.robot.commands.Auto.Auto3Ball;
import frc.robot.commands.Climber.ClimberMoveCommand;
import frc.robot.commands.Climber.ResetLeftClimberCommand;
import frc.robot.commands.Climber.ResetRightClimberCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StorageSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {  
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final StorageSubsystem storageSubsystem = new StorageSubsystem();

  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(pneumaticsSubsystem.getIntakeSolenoids());
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(pneumaticsSubsystem.getShooterSolenoids());
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(driveSubsystem, intakeSubsystem, visionSubsystem);
  private final ClimberMoveCommand climberMoveCommand = new ClimberMoveCommand(climberSubsystem);
  private final Auto2Ball auto2BallCommand = new Auto2Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallLeft auto2BallLeft = new Auto2BallLeft(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto2BallRight auto2BallRight = new Auto2BallRight(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);
  private final Auto3Ball auto3BallCommand = new Auto3Ball(driveSubsystem, intakeSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, visionSubsystem);

  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    climberSubsystem.setDefaultCommand(climberMoveCommand);
    configureButtonBindings();

    // Send auto commands to SmartDashboard
    chooser.setDefaultOption("3 Ball Auto", auto3BallCommand);
    chooser.addOption("2 Ball Left", auto2BallLeft);
    chooser.addOption("2 Ball Center", auto2BallRight);
    chooser.addOption("2 Ball Right", auto2BallCommand);

    SmartDashboard.putData(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(Constants.driveController, Constants.readyToShootButton).whenPressed(new ShootCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 0.0));
    new JoystickButton(Constants.driveController, Constants.intakeButton).whenPressed(new IntakeCommand(intakeSubsystem, storageSubsystem));
    new JoystickButton(Constants.manipController, Constants.resetLeftButton).whenPressed(new ResetLeftClimberCommand(climberSubsystem));
    new JoystickButton(Constants.manipController, Constants.resetRightButton).whenPressed(new ResetRightClimberCommand(climberSubsystem));
    new JoystickButton(Constants.manipController, Constants.overwriteShootCloseButton).whenPressed(new ShootCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, Constants.closeShootDistance));
    new JoystickButton(Constants.manipController, Constants.overwriteShootFarButton).whenPressed(new ShootCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, Constants.farShootDistance));

    //new JoystickButton(Constants.driveController, Constants.readyToShootButton).and(new JoystickButton(Constants.driveController, Constants.intakeButton)).whenPressed(new ShootAndIntakeCommand(visionSubsystem, pneumaticsSubsystem, shooterSubsystem, storageSubsystem, intakeSubsystem, 0.0));
    new JoystickButton(Constants.manipController, Constants.resetGyroButton).whenReleased(()->driveSubsystem.zeroGyroscope());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return auto2BallRight;
    return chooser.getSelected();
  }

  public DriveSubsystem getDriveTrain(){
    return driveSubsystem;
  }
}
