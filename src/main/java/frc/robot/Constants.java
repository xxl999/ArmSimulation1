// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final int kMotorPort = 0;
  public static final int kEncoderAChannel = 0;
  public static final int kEncoderBChannel = 1;
  public static final int kJoystickPort = 0;

  public static final String kArmPositionKey = "ArmPosition";
  public static final String kArmPKey = "ArmP";

  public static final double kDefaultArmKp = 50.0;
  public static final double kDefaultArmSetpointDegrees = 75.0;

  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096

  public static final double kArmReduction = 10.0;
  public static final double kArmMass = 8.0; 
  public static final double kArmLength = Units.inchesToMeters(30);
  public static final double kMinAngleRads = Units.degreesToRadians(-50);
  public static final double kMaxAngleRads = Units.degreesToRadians(255);

}
