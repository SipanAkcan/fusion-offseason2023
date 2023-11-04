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
  Joystick joystick = new Joystick(constants.JOYSTICK_PIN);
  
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem, 0.8));
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(joystick, 1).whileTrue(new LookUp(armSubsystem, 0.4));
    new JoystickButton(joystick, 1).whileFalse(new LookUp(armSubsystem, 0));
    new JoystickButton(joystick, 2).whileTrue(new LookDown(armSubsystem, 0.4));
    new JoystickButton(joystick, 2).whileFalse(new LookDown(armSubsystem, 0));
    new JoystickButton(joystick, 3).whileTrue(new Shoot(gripperubsystem, 0.4));
    new JoystickButton(joystick, 3).whileFalse(new Shoot(gripperubsystem, 0));
    new JoystickButton(joystick, 4).whileTrue(new InTake(gripperubsystem, 0.4));
    new JoystickButton(joystick, 4).whileFalse(new InTake(gripperubsystem, 0));
  }

  public Command getAutonomousCommand() {
    //return new GoXSecond(driveSubsystem, 0.7, 2);
    //return new TurnXSecond(driveSubsystem, 0.>'5, 1);
    //return new TurnXDegrees(driveSubsystem, 180, 0.5);
    //return new GoXMeter(driveSubsystem, 1);
    return null;
  }
}
