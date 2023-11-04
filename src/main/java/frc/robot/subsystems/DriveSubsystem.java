// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.auto.AutoConfigurer;
import frc.robot.auto.RotationalDrive;
import frc.robot.auto.StraightDrive;

public class DriveSubsystem extends SubsystemBase {
  Constants constants = new Constants();
  DriveConstants driveConstants = new DriveConstants();

  CANSparkMax frontLeftSpark = new CANSparkMax(driveConstants.FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
  CANSparkMax rearLeftSpark = new CANSparkMax(driveConstants.REAR_LEFT_SPARK_ID, MotorType.kBrushless);
  CANSparkMax frontRightSpark = new CANSparkMax(driveConstants.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
  CANSparkMax rearRightSpark = new CANSparkMax(driveConstants.MID_RIGHT_SPARK_ID, MotorType.kBrushless);

  RelativeEncoder rightEncoder = frontRightSpark.getEncoder();
  RelativeEncoder leftEncoder = frontLeftSpark.getEncoder();

  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(frontRightSpark,rearRightSpark);
  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(frontLeftSpark,rearLeftSpark);

  DifferentialDrive differentialDrive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);

  AHRS navx = new AHRS(driveConstants.NAVX_PORT);


  Joystick driveJoystick = new Joystick(constants.JOYSTICK_PIN);

  AutoConfigurer autoConfigurer = new AutoConfigurer();

  RotationalDrive rotationalDrive = new RotationalDrive(autoConfigurer, navx);
  StraightDrive straightDrive = new StraightDrive(autoConfigurer, leftEncoder, rightEncoder);
  
  public DriveSubsystem() {
    rightMotorControllerGroup.setInverted(true);
  }

  public void arcadeDrive(double maxSpeed) {
    differentialDrive.arcadeDrive(driveJoystick.getRawAxis(1) * maxSpeed, driveJoystick.getRawAxis(4) * maxSpeed);
  }

  public void goXSecond(double speed) {
    differentialDrive.arcadeDrive(speed, 0);
  }

  public void goXMeter(double setpoint) {
    double output = straightDrive.goXmeter(setpoint);
    differentialDrive.arcadeDrive(output, 0);
  }

  public void turnXSecond(double speed) {
    differentialDrive.arcadeDrive(0, speed);
  }

  public void turnXDegrees(double setpoint, double speed) {
    double output = rotationalDrive.turnXDegrees(setpoint);
    differentialDrive.arcadeDrive(0, output);
  }

  public void stopDriveMotors() {
    leftMotorControllerGroup.stopMotor();
    rightMotorControllerGroup.stopMotor();
  }

  @Override
  public void periodic() {}
}