// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

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

    public static double fleo = -0.3;
    public static double freo = -0.45;
    public static double rreo = 0.4;
    public static double rleo = 0.28;

    public static int talonCount = 12;

    public static double wheelBase = 14.0;


    public static XboxController driveController = new XboxController(0);

    public static int IntakeButton = 1;

    public static int intakeExtendPort = 6;
    public static int intakeRetractPort = 7;
    public static int shooterExtendPort = 1;
    public static int shooterRetractPort = 0;
    public static int motorCoolerPort = 4;

}
