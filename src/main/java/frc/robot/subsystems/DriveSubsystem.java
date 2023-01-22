// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.auto.AutoConfigurer;
import frc.robot.auto.RotationalDrive;

public class DriveSubsystem extends SubsystemBase {
  WPI_VictorSPX frontLeftVictor;
  WPI_VictorSPX midLeftVictor;
  WPI_VictorSPX rearLeftVictor;
  WPI_VictorSPX frontRightVictor;
  WPI_VictorSPX midRightVictor;
  WPI_VictorSPX rearRightVictor;
  MotorControllerGroup leftMotorControllerGroup;
  MotorControllerGroup rightMotorControllerGroup;
  DifferentialDrive differentialDrive;
  Joystick driveJoystick;
  AHRS navx;
  AutoConfigurer autoConfigurer;
  RotationalDrive rotationalDrive;
  public DriveSubsystem() {
    frontLeftVictor = new WPI_VictorSPX(Constants.DriveConstants.FRONT_LEFT_VICTOR_ID);
    midLeftVictor = new WPI_VictorSPX(Constants.DriveConstants.MID_LEFT_VICTOR_ID);
    rearLeftVictor = new WPI_VictorSPX(Constants.DriveConstants.REAR_LEFT_VICTOR_ID);
    frontRightVictor = new WPI_VictorSPX(Constants.DriveConstants.FRONT_RIGHT_VICTOR_ID);
    midRightVictor = new WPI_VictorSPX(Constants.DriveConstants.MID_RIGHT_VICTOR_ID);
    rearRightVictor = new WPI_VictorSPX(Constants.DriveConstants.REAR_RIGHT_VICTOR_ID);
    leftMotorControllerGroup = new MotorControllerGroup(frontLeftVictor,midLeftVictor,rearLeftVictor);
    rightMotorControllerGroup = new MotorControllerGroup(frontRightVictor,midRightVictor,rearRightVictor);
    driveJoystick = new Joystick(Constants.JOYSTICK_PIN);
    navx = new AHRS(Constants.DriveConstants.NAVX_PORT);
    autoConfigurer = new AutoConfigurer();
    rotationalDrive = new RotationalDrive(autoConfigurer, navx);
  }

  public void arcadeDrive(double maxSpeed) {
    differentialDrive.arcadeDrive(driveJoystick.getRawAxis(2) * maxSpeed, driveJoystick.getRawAxis(1) * maxSpeed);
  }

  public void goXSecond(double speed) {
    differentialDrive.arcadeDrive(speed, speed);
  }

  public void turnXSecond(double speed) {
    differentialDrive.arcadeDrive(0, speed);
  }

  public void turnXDegrees(double setpoint, double speed) {
    double output = rotationalDrive.turnXDegrees(setpoint);
    differentialDrive.arcadeDrive(0, output * speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NAVX YAW: ", navx.getYaw());
  }
}