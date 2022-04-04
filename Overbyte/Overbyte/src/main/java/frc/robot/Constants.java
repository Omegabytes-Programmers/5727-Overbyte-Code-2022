// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import frc.omegabytes.ShooterConfiguration;
import frc.omegabytes.VisionConfiguration;
import frc.omegabytes.VisionShooterConversion;
import frc.omegabytes.VisionWeightedAverage;

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
    
    public static int storageWheelMotorPort = 9;
    public static int storageBeltMotorPort = 14; // Talon.

    public static int topShooterMotorPort = 10;
    public static int bottomShooterMotorPort = 11;

    public static int leftClimberMotorPort = 12;
    public static int rightClimberMotorPort = 13;

    public static int flePort = 0;
    public static int frePort = 1;
    public static int rrePort = 2;
    public static int rlePort = 3;

    public static double fleo = Math.toRadians(-42.5390625);
    public static double freo = Math.toRadians(-38.232421875);
    public static double rreo = Math.toRadians(-251.71875);
    public static double rleo = Math.toRadians(-197.666015625);

    public static int talonCount = 14;

    public static XboxController driveController = new XboxController(0);
    public static XboxController manipController = new XboxController(1);
    public static XboxController autoController = new XboxController(2);
   
    public static XboxController topController = new XboxController(3);
    public static XboxController bottomController = new XboxController(4);
   

    // Driver controller buttons/axis
    public static int translateYAxis = 0;
    public static int translateXAxis = 1;
    public static int extendLiftAxis = 2;
    public static int retractLiftAxis = 3;
    public static int rotationAxis = 4;
    public static int halfSpeedButton = 1;
    public static int robotOrientedButton = 2;
    public static int readyToShootButton = 5;
    public static int intakeButton = 6;

    // Manipulator controll buttons
    public static int resetGyroButton = 1;
    public static int expelBallButton = 3;
    public static int overwriteShootCloseButton = 5;
    public static int overwriteShootFarButton = 6;
    


    public static int resetLeftButton = 3;
    public static int resetRightButton = 2;
    
    public static int ball1Toggle = 1;
    public static int ball2Toggle = 2;
    public static int ball3Toggle = 3;
    public static int ball4Toggle = 4;
    public static int ball5Toggle = 5;

    public static int intakeExtendPort = 13;
    public static int intakeRetractPort = 15;
    public static int shooterExtendPort = 0;
    public static int shooterRetractPort = 1;

    public static int storageTopProxSensorPort = 0;
    public static int storageBottomProxSensorPort = 1;
    public static int intakeLeftProxSensorPort = 2;
    public static int intakeRightProxSensorPort = 3;

    public static double deadzone = 0.1;

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
    
    public static int visionPersistTicks = 100;

    public static VisionWeightedAverage visionTarget = new VisionWeightedAverage();

    public static double closeShootDistance = 7.06;
    public static double farShootDistance = 10.59;

    public static Pose2d autoPoseBall2Left = new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(-20));
    public static Pose2d autoPoseBall2Right = new Pose2d(2.099, 0.386, Rotation2d.fromDegrees(11));
    
    public static Pose2d autoPoseShootLeft = new Pose2d(3.5, 1.5, Rotation2d.fromDegrees(-33));
    public static Pose2d autoPoseShootRight = new Pose2d(2.099, 0.386, Rotation2d.fromDegrees(38));

    public static Pose2d autoPoseBall2 = new Pose2d(0.69, 2.25, Rotation2d.fromDegrees(90));
    public static Pose2d autoPoseBall3 = new Pose2d(3.5, 0.0, Rotation2d.fromDegrees(20));
    public static Pose2d autoPoseBall45 = new Pose2d(9.0, 0.9, Rotation2d.fromDegrees(45));
    
    public static Pose2d autoPoseShoot1 = new Pose2d(3.6, 0.1, Rotation2d.fromDegrees(70.7));
    public static Pose2d autoPoseShoot2 = new Pose2d(5.5, -0.577, Rotation2d.fromDegrees(20.1));
    
    public static double translationRateLimit = 2.5;
    public static double rotationRateLimit = 2.5;

    public static double translationFeedForward = 0.1;
    public static double rotationFeedForward = 0.1;

    public static double maxVoltage = 12.0; //For "home" testing change this value to the range of 4.0 to 8.0.
    public static double wheelBase = .572;

    public static double maxVelocity = (6380.0 / 60.0 * 
        SdsModuleConfigurations.MK4_L2.getDriveReduction() * 
        SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI);
    
    public static double maxAngularVelocity = maxVelocity /
        Math.hypot(wheelBase / 2.0, wheelBase / 2.0);


    public static final TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularVelocity);

    public static PIDController translationXController = new PIDController(0.1, 0, 0); //10
    public static PIDController translationYController = new PIDController(0.1, 0, 0);
    public static ProfiledPIDController rotationController = new ProfiledPIDController(5.0, 0, 0, Constants.rotationConstraints); // 1.2
    
}
