// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;


public class DriveSubsystem extends SubsystemBase {

  DriveConstants driveConstants = new DriveConstants();

  CANSparkMax frontLeftSpark = new CANSparkMax(Constants.DriveConstants.FRONT_LEFT_SPARK_ID, MotorType.kBrushless);
  CANSparkMax rearLeftSpark = new CANSparkMax(Constants.DriveConstants.REAR_LEFT_SPARK_ID, MotorType.kBrushless);
  CANSparkMax frontRightSpark = new CANSparkMax(Constants.DriveConstants.FRONT_RIGHT_SPARK_ID, MotorType.kBrushless);
  CANSparkMax rearRightSpark = new CANSparkMax(Constants.DriveConstants.REAR_RIGHT_SPARK_ID, MotorType.kBrushless);

  RelativeEncoder rightEncoder;
  RelativeEncoder leftEncoder;

  MotorControllerGroup rightMotorControllerGroup = new MotorControllerGroup(frontRightSpark,rearRightSpark);
  MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(frontLeftSpark,rearLeftSpark);

  DifferentialDrive differentialDrive = new DifferentialDrive(rightMotorControllerGroup, leftMotorControllerGroup);

  AHRS navx = new AHRS(Constants.DriveConstants.NAVX_PORT);

  Joystick driveJoystick = new Joystick(Constants.OMER_PIN);

  public ProfiledPIDController speedPIDController = new ProfiledPIDController(3, 0, 0, new Constraints(2, 1.5));
  public ProfiledPIDController rotationPIDController = new ProfiledPIDController(0.07, 0, 0, new Constraints(1.5, 1.5));
  public ProfiledPIDController balancePIDController = new ProfiledPIDController(0.022, 0.001, 0.0005, new Constraints(0.5, 0.5));
  public PIDController turnPidController = new PIDController(0.012, 0.00001, 0.00002);

  public DriveSubsystem() {
    rightEncoder = frontRightSpark.getEncoder();
    leftEncoder = frontLeftSpark.getEncoder();
    resetSensors();
  }

  public void arcadeDrive(double maxSpeed) {
    differentialDrive.arcadeDrive(driveJoystick.getRawAxis(1) * -maxSpeed, driveJoystick.getRawAxis(2) * maxSpeed);
  }

  public void balanceDrive(){
    differentialDrive.arcadeDrive(balancePIDController.calculate(-getPitch(), getAverageMeter()), getAverageMeter());
  }

  public void goXSecond(double speed) {
    differentialDrive.arcadeDrive(speed, 0);
  }

  public void goXMeter(double setpoint) {
    differentialDrive.arcadeDrive(speedPIDController.calculate(getAverageMeter() ,setpoint), rotationPIDController.calculate(navx.getYaw(), 0));
  }

  public void turnXSecond(double speed) {
    differentialDrive.arcadeDrive(0, speed);
  }

  public void turnXDegrees(double setpoint) {
    differentialDrive.arcadeDrive(0, turnPidController.calculate(getGyroAngle(), setpoint));
  }

  public void stopDriveMotors() {
    leftMotorControllerGroup.stopMotor();
    rightMotorControllerGroup.stopMotor();
  }

  public double getGyroAngle(){
    return navx.getYaw();
  }

  public double getPitch(){
    return navx.getPitch();
  }

  public double getAverageMeter(){
    return ((rightEncoder.getPosition()+leftEncoder.getPosition())/2) * Constants.DriveConstants.K_DRIVE_TICK_2_METER;
  }

  public void resetSensors(){
    resetGyro();
    resetEncoders();
  }

  public void resetGyro(){
    navx.reset();
  }

  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    rightMotorControllerGroup.setInverted(true);
    rightEncoder = frontRightSpark.getEncoder();
    leftEncoder = frontLeftSpark.getEncoder();
    SmartDashboard.putNumber("right encoder:", rightEncoder.getCountsPerRevolution());
    SmartDashboard.putNumber("left encoder:", leftEncoder.getCountsPerRevolution());
  }
}