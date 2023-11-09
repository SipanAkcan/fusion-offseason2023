// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public final class Constants {
  public final int OMER_PIN = 0;
  public final int SEVVAL_PIN = 1;
  public static class DriveConstants {
    public final int FRONT_LEFT_SPARK_ID = 2;
    public final int REAR_LEFT_SPARK_ID = 3;
    public final int FRONT_RIGHT_SPARK_ID = 7;
    public final int REAR_RIGHT_SPARK_ID = 5;
    public final Port NAVX_PORT = SPI.Port.kMXP;
  }
  public static class ArmConstants {
    public final int ARM_SPARK_ID = 4;
  }
  public static class GripperConstants {
    public final int GRIPPER_SPARK_ID = 6;
  }

  public final double DRIVE_SPEED = 0.6;
  public final double INTAKE_SPEED = 0.4;
  public final double SHOOT_SPEED = 0.28;
  public final double ARM_SPEED = 0.3;

  public final double K_DRIVE_TICK_2_FEET = 1.0 / 128 * 6 * Math.PI / 12;
  
}
