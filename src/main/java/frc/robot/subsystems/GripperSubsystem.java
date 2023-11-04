// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  Constants constants = new Constants();
  GripperConstants gripperConstants = new GripperConstants();
  CANSparkMax gripperSpark = new CANSparkMax(gripperConstants.GRIPPER_SPARK_ID, MotorType.kBrushless);

  public GripperSubsystem() {}

  public void givePower2Gripper(double speed) {
    gripperSpark.set(-speed);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
