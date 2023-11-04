// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public final class Constants {
  public final int JOYSTICK_PIN = 0;
  public static class DriveConstants {
    public final int FRONT_LEFT_SPARK_ID = 11;
    public static final int MID_LEFT_SPARK_ID = 12;
    public final int REAR_LEFT_SPARK_ID = 13;
    public final int FRONT_RIGHT_SPARK_ID = 14;
    public final int MID_RIGHT_SPARK_ID = 15;
    public static final int REAR_RIGHT_SPARK_ID = 16;
    public final Port NAVX_PORT = SPI.Port.kMXP;
  }
  public static class ArmConstants {
    public final int WRIST_SPARK_ID = 17;
  }
  public static class GripperConstants {
    public final int GRIPPER_SPARK_ID = 3;
  }
}
