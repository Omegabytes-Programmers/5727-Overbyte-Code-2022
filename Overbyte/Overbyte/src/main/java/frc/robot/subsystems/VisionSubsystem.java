// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    stream = table.getEntry("stream");
    snapshot = table.getEntry("snapshot");

    stream.setNumber(0);
    snapshot.setNumber(0);
  }
 
  public double getAngle(){
    return ty.getDouble(0.0);
  }

  public double getPosition(){
    return tx.getDouble(0.0);
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
