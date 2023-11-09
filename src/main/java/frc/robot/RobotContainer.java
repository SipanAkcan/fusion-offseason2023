// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.arm_command.LookDown;
import frc.robot.commands.arm_command.LookUp;
import frc.robot.commands.auto_command.GoXSecond;
import frc.robot.commands.gripper_command.InTake;
import frc.robot.commands.gripper_command.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {
  Constants constants = new Constants();
  ArmSubsystem armSubsystem = new ArmSubsystem();
  GripperSubsystem gripperubsystem = new GripperSubsystem();
  DriveSubsystem driveSubsystem;
  Joystick omer = new Joystick(constants.OMER_PIN);
  Joystick sevval = new Joystick(constants.SEVVAL_PIN);
  
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem, constants.DRIVE_SPEED));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(sevval, 3).whileTrue(new LookUp(armSubsystem, constants.ARM_SPEED));
    new JoystickButton(sevval, 5).whileTrue(new LookDown(armSubsystem, constants.ARM_SPEED));
    new JoystickButton(omer, 1).whileTrue(new Shoot(gripperubsystem, constants.SHOOT_SPEED));
    new JoystickButton(sevval, 1).whileTrue(new InTake(gripperubsystem, constants.INTAKE_SPEED));
  }

  public Command getAutonomousCommand() {
    return new LookUp(armSubsystem, constants.ARM_SPEED).withTimeout(1.5).andThen(
        new Shoot(gripperubsystem, constants.SHOOT_SPEED).withTimeout(1)).andThen(
          new LookDown(armSubsystem, constants.ARM_SPEED).withTimeout(1)).andThen(
            new GoXSecond(driveSubsystem, -constants.DRIVE_SPEED, 1.8)
          );
    //return new TurnXSecond(driveSubsystem, 0.5, 1);
    //return null;
  }
}
