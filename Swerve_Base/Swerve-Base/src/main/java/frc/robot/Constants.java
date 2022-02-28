// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.omegabytes.VisionConfiguration;
import frc.robot.subsystems.VisionSubsystem;
import frc.omegabytes.ShooterConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drive Train
    public static int fldmPort = 0;
    public static int flsmPort = 1;

    public static int frdmPort = 2;
    public static int frsmPort = 3;

    public static int rrdmPort = 4;
    public static int rrsmPort = 5;

    public static int rldmPort = 6;
    public static int rlsmPort = 7;

    public static int intakeMotorPort = 8;
    
    public static int storageConveyorPort = 13; // Talon.
    public static int storageWheelMotorPort = 9;

    public static int topShooterMotorPort = 10;
    public static int bottomShooterMotorPort = 11;

    public static int leftClimberMotorPort = 14;
    public static int rightClimberMotorPort = 15;

    public static int flePort = 0;
    public static int frePort = 1;
    public static int rrePort = 2;
    public static int rlePort = 3;
    //265.869140625
    public static double fleo = Math.toRadians(-258.66);
    public static double freo = Math.toRadians(-33.05);
    public static double rreo = Math.toRadians(-253.65);
    public static double rleo = Math.toRadians(-60.30);

    public static int talonCount = 12;

    public static double wheelBase = 14.0;


    public static XboxController driveController = new XboxController(0);
    public static XboxController manipController = new XboxController(1);
    public static XboxController topController = new XboxController(3);
    public static XboxController bottomController = new XboxController(4);
    //public static XboxController autoController = new XboxController(2); //If we run out of buttons we can use another arcade cabnet controller

    public static int translateYAxis = 0;
    public static int translateXAxis = 1;
    public static int rotationAxis = 4;
    public static int halfSpeedButton = 1;
    public static int lockWheelButton = 2;

    public static int intakeButton = 6;
    public static int readyToShootButton = 5;
    public static int expelBallButton = 3;
    public static int extendLiftButton = 4;
    public static int retractLiftButton = 5;
    
    public static int ball1Toggle = 6; // If we run out of buttons 1
    public static int ball2Toggle = 7; // If we run out of buttons 2
    public static int ball3Toggle = 8; // If we run out of buttons 3
    public static int ball4Toggle = 9; // If we run out of buttons 4
    public static int ball5Toggle = 10; // If we run out of buttons 5

    public static int intakeExtendPort = 6;
    public static int intakeRetractPort = 7;
    public static int shooterExtendPort = 1;
    public static int shooterRetractPort = 0;
    public static int motorCoolerPort = 4;

    public static int stopLidarPort = 0;

    public static double deadzone = 0.05;

    public static VisionConfiguration[] visionTable = {
        new VisionConfiguration(40.48, 6.0),
        new VisionConfiguration(33.93, 7.0),
        new VisionConfiguration(28.88, 8.0),
        new VisionConfiguration(24.33, 9.0),
        new VisionConfiguration(20.31, 10.0),
        new VisionConfiguration(17.10, 11.0),
        new VisionConfiguration(14.01, 12.0),
        new VisionConfiguration(12.06, 13.0),
        new VisionConfiguration(9.86, 14.0),
        new VisionConfiguration(7.30, 15.0),
        new VisionConfiguration(5.65, 16.0),
        new VisionConfiguration(4.05, 17.0),
        new VisionConfiguration(2.33, 18.0),
        new VisionConfiguration(0.85, 19.0),
        new VisionConfiguration(0, 20.0)
    };
 
    public static ShooterConfiguration[] shootingTable = {
        new ShooterConfiguration(4.0, 1.0, -0.25984251499176025, false),
        new ShooterConfiguration(30.17, 1.0, -0.35433071851730347 , false),
        new ShooterConfiguration(25.32, 0.6614173054695129, -0.7952755689620972, false),
        new ShooterConfiguration(25.32, 0.5826771855354309, -0.7952755689620972, true),
        new ShooterConfiguration(20.99, 0.5826771855354309, -0.7952755689620972, true),
        new ShooterConfiguration(17.25, 0.5984252095222473, -0.8582677245140076, true),
        new ShooterConfiguration(14.90, 0.5354330539703369, -0.8110235929489136, true),
        new ShooterConfiguration(12.48, 0.6692913174629211, -0.7086614370346069, true),
        new ShooterConfiguration(10.37, 0.6771653294563293, -0.7716535329818726, true),
        new ShooterConfiguration(8.18, 0.7086614370346069, -0.8110235929489136, true),
        new ShooterConfiguration(5.91, 0.7086614370346069, -0.8110235929489136, true)
        

  
    };
}

