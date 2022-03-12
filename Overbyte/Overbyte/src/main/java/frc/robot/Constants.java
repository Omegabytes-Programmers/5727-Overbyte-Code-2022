// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.omegabytes.ShooterConfiguration;
import frc.omegabytes.VisionConfiguration;
import frc.omegabytes.VisionShooterConversion;

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
    
    public static int storageConveyorPort = 14; // Talon.
    public static int storageWheelMotorPort = 9;

    public static int topShooterMotorPort = 10;
    public static int bottomShooterMotorPort = 11;

    public static int leftClimberMotorPort = 12;
    public static int rightClimberMotorPort = 13;

    public static int flePort = 0;
    public static int frePort = 1;
    public static int rrePort = 2;
    public static int rlePort = 3;

    public static double fleo = Math.toRadians(-37.6171875);
    public static double freo = Math.toRadians(-38.232421875);
    public static double rreo = Math.toRadians(-16.083984375);
    public static double rleo = Math.toRadians(-194.677734375);

    public static int talonCount = 14;

    public static double wheelBase = 14.0;


    public static XboxController driveController = new XboxController(0);
    public static XboxController manipController = new XboxController(1);
    public static XboxController topController = new XboxController(3);
    public static XboxController bottomController = new XboxController(4);
    public static XboxController resetController = new XboxController(5);
    //public static XboxController autoController = new XboxController(2); //If we run out of buttons we can use another arcade cabnet controller

    public static int translateYAxis = 0;
    public static int translateXAxis = 1;
    public static int rotationAxis = 4;
    public static int halfSpeedButton = 1;
    public static int lockWheelButton = 2;
    public static int robotOrientButton = 3;
    public static int intakeButton = 6;
    public static int readyToShootButton = 5;
    

    public static int overwriteShootCloseButton = 5;
    public static int overwriteShootFarButton = 6;
    public static int resetGyroButton = 1;
    public static int expelBallButton = 3;
    public static int extendLiftAxis = 2;
    public static int retractLiftAxis = 3;

    public static int resetLeftButton = 5;
    public static int resetRightButton = 6;
    
    public static int ball1Toggle = 6; // If we run out of buttons 1
    public static int ball2Toggle = 7; // If we run out of buttons 2
    public static int ball3Toggle = 8; // If we run out of buttons 3
    public static int ball4Toggle = 9; // If we run out of buttons 4
    public static int ball5Toggle = 10; // If we run out of buttons 5

    public static int intakeExtendPort = 13;
    public static int intakeRetractPort = 15;
    public static int shooterExtendPort = 0;
    public static int shooterRetractPort = 1;
    public static int motorCoolerPort = 4;

    public static int stopLidarPort = 9;

    public static double deadzone = 0.05;

    public static VisionConfiguration[] visionTable = {
        new VisionConfiguration(39.6, 6.0),
        new VisionConfiguration(33.41, 7.0),
        new VisionConfiguration(27.76, 8.0),
        new VisionConfiguration(23.91, 9.0),
        new VisionConfiguration(20.10, 10.0),
        new VisionConfiguration(17.04, 11.0),
        new VisionConfiguration(14.24, 12.0),
        new VisionConfiguration(11.26, 13.0),
        new VisionConfiguration(9.54, 14.0),
        new VisionConfiguration(7.27, 15.0),
        new VisionConfiguration(5.38, 16.0),
        new VisionConfiguration(4.54, 17.0),
        new VisionConfiguration(2.08, 18.0),
        new VisionConfiguration(1.07, 19.0),
        new VisionConfiguration(0.0, 20.0)
    };
// Coconut.jpg
    public static ShooterConfiguration[] shootingTable = {
        new ShooterConfiguration(6.0, 1.0, -0.22, false),
        new ShooterConfiguration(7.0, 1.0, -0.27, false),
        new ShooterConfiguration(8.0, 1.0, -0.37, false),
        new ShooterConfiguration(9.0, 1.0, -0.53, false),
        new ShooterConfiguration(10.0, 1.0, -0.60, false),
        new ShooterConfiguration(9.0, 0.65, -0.65, true),
        new ShooterConfiguration(10.0, 0.67, -0.67, true),
        new ShooterConfiguration(11.0, 0.68, -0.68, true),
        new ShooterConfiguration(12.0, 0.70, -0.70, true),
        new ShooterConfiguration(13.0, 0.72, -0.72, true),
        new ShooterConfiguration(14.0, 0.74, -0.74, true),
        new ShooterConfiguration(15.0, 0.76, -0.76, true),
        new ShooterConfiguration(16.0, 0.77, -0.77, true),
        new ShooterConfiguration(17.0, 0.78, -0.78, true),
        new ShooterConfiguration(18.0, 0.80, -0.80, true),
        new ShooterConfiguration(19.0, 0.81, -0.81, true),
        new ShooterConfiguration(20.0, 0.83, -0.83, true)
    };


    public static VisionShooterConversion vsConversion = new VisionShooterConversion(visionTable, shootingTable, 5);

    public static double closeShootDistance = 7.06;
    public static double farShootDistance = 10.59;
}

