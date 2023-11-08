// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public final class Constants {

  public static final int JOYSTICK_PIN = 0;

  public static class DriveConstants {
    public static final int FRONT_LEFT_SPARK_ID = 2;
    public static final int REAR_LEFT_SPARK_ID = 3;
    public static final int FRONT_RIGHT_SPARK_ID = 7;
    public static final int REAR_RIGHT_SPARK_ID = 5;
    public static final Port NAVX_PORT = SPI.Port.kMXP; 
    public static final double DRIVE_SPEED = 0.8;
    public static final double K_DRIVE_TICK_2_METER = 1.0 / 10.72 *(2 * Math.PI * 0.07619999);
  }

  public static final class ArmConstants {
    public static final int ARM_SPARK_ID = 4;
    public static final double UP_DOWN_TIME = 1.5;
    public static final double ARM_SPEED = 0.3;
  }

  public static class GripperConstants {
    public static final int GRIPPER_SPARK_ID = 6;
    public static final double INTAKE_SPEED = 0.4;
    public static final double SHOOT_SPEED = 0.4;
  }

}
