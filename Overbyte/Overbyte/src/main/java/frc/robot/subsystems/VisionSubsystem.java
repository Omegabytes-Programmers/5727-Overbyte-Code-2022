// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.invoke.ConstantCallSite;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionSubsystem extends SubsystemBase {
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry stream;
  NetworkTableEntry snapshot;
  NetworkTableEntry pipeline;

  double lastGoodAngle = -1;
  int lastAngleAge = 0;

  double lastGoodPos = -1;
  int lastPosAge = 0;

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    stream = table.getEntry("stream");
    snapshot = table.getEntry("snapshot");
    pipeline = table.getEntry("pipeline");

    stream.setNumber(0);
    snapshot.setNumber(0);
    pipeline.setNumber(8);

  }
 
  public double getAngle() {
    double currentVal = ty.getDouble(-1);

    if (currentVal < 0) {
      if (lastAngleAge < Constants.visionPersistTicks) {
        currentVal = lastGoodAngle;
        lastAngleAge++;
      }
    } else {
      lastGoodAngle = currentVal;
      lastAngleAge = 0;
    }
    if (currentVal < 0) {
      currentVal = 0;
    }
    return currentVal;
  }

  public double getPosition() {
    double currentVal = tx.getDouble(-1);

    if (currentVal < 0) {
      if (lastPosAge < Constants.visionPersistTicks) {
        currentVal = lastGoodPos;
        lastPosAge++;
      }
    } else {
      lastGoodPos = currentVal;
      lastPosAge = 0;
    }
    if (currentVal < 0) {
      currentVal = 0;
    }
    return currentVal;
  }

  public void takeSnapshot(){
    table.getEntry("snapshot").setNumber(1);
  }

  public void resetSnapshot(){
    table.getEntry("snapshot").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //read values periodically

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    ////System.out.println(x);
    ////System.out.println(y);
    ////System.out.println(area);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
  }
}
