// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
   
    public static XboxController calibrateController = new XboxController(5);
   

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

    // Manipulator controller buttons
    public static int resetGyroButton = 1;
    public static int expelBallButton = 3;
    public static int overwriteShootCloseButton = 5;
    public static int overwriteShootFarButton = 6;

    // Calibration controller buttons
    public static int topRoughAxis = 0;
    public static int bottomRoughAxis = 1;
    public static int topFineAxis = 2;
    public static int bottomFineAxis = 3;
    public static int runShooterButton = 1;
    public static int hoodUpButton = 2;
    public static int topTuneSet = 3;
    public static int bottomTuneSet = 4;
    public static int topTuneReset = 5;
    public static int bottomTuneReset = 6;
    public static int bottomTuneRange1 = 7;
    public static int topTuneRange1 = 8;
    public static int bottomTuneRange2 = 9;
    public static int topTuneRange2 = 10;
    public static int bottomTuneRange3 = 11;
    public static int topTuneRange3 = 12;

    public static int resetLeftButton = 3;
    public static int resetRightButton = 2;

    public static int intakeExtendPort = 13;
    public static int intakeRetractPort = 15;
    public static int shooterExtendPort = 0;
    public static int shooterRetractPort = 1;

    public static int storageTopProxSensorPort = 0;
    public static int storageBottomProxSensorPort = 1;
    public static int intakeLeftProxSensorPort = 2;
    public static int intakeRightProxSensorPort = 3;

    public static double deadzone = 0.1;

    //#region UNCA configuration
    public static VisionConfiguration[] visionTable = {
        new VisionConfiguration(43.25, 6.0),
        new VisionConfiguration(37.64, 7.0),
        new VisionConfiguration(32.37, 8.0),
        new VisionConfiguration(28.30, 9.0),
        new VisionConfiguration(24.28, 10.0),
        new VisionConfiguration(21.19, 11.0),
        new VisionConfiguration(18.10, 12.0),
        new VisionConfiguration(15.91, 13.0),
        new VisionConfiguration(13.53, 14.0),
        new VisionConfiguration(11.60, 15.0),
        new VisionConfiguration(9.90, 16.0),
        new VisionConfiguration(8.23, 17.0),
        new VisionConfiguration(6.59, 18.0),
        new VisionConfiguration(5.17, 19.0),
        new VisionConfiguration(3.97, 20.0),
        new VisionConfiguration(2.59, 21.0),
        new VisionConfiguration(1.81, 22.0),
        new VisionConfiguration(1.04, 23.0),
        new VisionConfiguration(0.09, 24.0)
    };
 
    public static ShooterConfiguration[] shootingTable = {
        new ShooterConfiguration(5.0, 2731.74, -3715.11, false),
        new ShooterConfiguration(6.0, 2951.96, -3851.38, false),
        new ShooterConfiguration(7.0, 3968.66, -4169.60, false),
        new ShooterConfiguration(8.0, 4037.34, -4585.62, false),
        new ShooterConfiguration(9.0, 4535.78, -4785, false),
        new ShooterConfiguration(10.0, 5682.1, -4286.56, false),
        new ShooterConfiguration(9.0, 3473.07, -3825.62, true),
        new ShooterConfiguration(10.0, 3476.22, -3778.12, true),
        new ShooterConfiguration(11.0, 3538.90, -4037.34, true),
        new ShooterConfiguration(12.0, 4386.25, -4485.9375, true),
        new ShooterConfiguration(13.0, 4585.62, -4589.37, true),
        new ShooterConfiguration(14.0, 4685.31, -4689.06, true),
        new ShooterConfiguration(15.0, 4834.84, -4702.18, true),
        new ShooterConfiguration(16.0, 4834.84, -4820.46, true),
        new ShooterConfiguration(17.0, 5084.06, -5019.84, true),
        new ShooterConfiguration(18.0, 5084.06, -5084.06, true),
        new ShooterConfiguration(20.0, 5233.59, -5219.21, true),
        new ShooterConfiguration(22.0, 5582.5, -5518.28, true),
        new ShooterConfiguration(24.0, 5682.18, -5990.07, true)
    };
    //#endregion

    public static VisionShooterConversion vsConversion = new VisionShooterConversion(visionTable, shootingTable, 5);
    
    public static int visionPersistTicks = 100;

    public static VisionWeightedAverage visionTarget = new VisionWeightedAverage();

    public static double closeShootDistance = 7.06;
    public static double farShootDistance = 10.59;
    
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
    
    public static double shooterkF = 1023.0/20660.0;
    public static double shooterkP = 0.1;
    public static double shooterkI = 0.001;
    public static double shooterkD = 5.0;
}
