// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.MevlanaCommand;
import frc.robot.commands.arm_command.LookDown;
import frc.robot.commands.arm_command.LookUp;
import frc.robot.commands.auto_command.GoXMeter;
import frc.robot.commands.auto_command.TurnXDegrees;
import frc.robot.commands.gripper_command.InTake;
import frc.robot.commands.gripper_command.Shoot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RobotContainer {

  ArmSubsystem armSubsystem;
  GripperSubsystem gripperubsystem;
  DriveSubsystem driveSubsystem;

  SendableChooser<Command> autoChooser = new SendableChooser<>();

  Joystick omer = new Joystick(constants.OMER_PIN);
  Joystick sevval = new Joystick(constants.SEVVAL_PIN);
  
  public RobotContainer() {
    driveSubsystem = new DriveSubsystem();
    gripperubsystem = new GripperSubsystem();
    armSubsystem = new ArmSubsystem();

    driveSubsystem.setDefaultCommand(new ArcadeDrive(driveSubsystem, Constants.DriveConstants.DRIVE_SPEED));
    configureBindings();

    // Autonomous Variations
    autoChooser.setDefaultOption("be like mevlana", new MevlanaCommand(new DriveSubsystem()));

    autoChooser.addOption("Balance", new SequentialCommandGroup(new GoXMeter(driveSubsystem, -0.80), new BalanceCommand(driveSubsystem)));

    autoChooser.addOption("Ball and Balance", new SequentialCommandGroup(new LookUp(armSubsystem, Constants.ArmConstants.ARM_SPEED).withTimeout(Constants.ArmConstants.UP_DOWN_TIME), new Shoot(gripperubsystem, Constants.GripperConstants.SHOOT_SPEED)));

    autoChooser.addOption("Just Ball", new SequentialCommandGroup(new LookUp(armSubsystem, Constants.ArmConstants.ARM_SPEED).withTimeout(1), new Shoot(gripperubsystem, Constants.GripperConstants.SHOOT_SPEED).withTimeout(2)));

    autoChooser.addOption("Ball and taxi", new SequentialCommandGroup(new LookUp(armSubsystem, Constants.ArmConstants.ARM_SPEED).withTimeout(1), new Shoot(gripperubsystem, Constants.GripperConstants.SHOOT_SPEED).withTimeout(2),
                                                                           new GoXMeter(driveSubsystem, 4)));

    autoChooser.addOption("Koçbaşi", new SequentialCommandGroup(
    new LookUp(armSubsystem, Constants.ArmConstants.ARM_SPEED).withTimeout(1),
    new Shoot(gripperubsystem, Constants.GripperConstants.SHOOT_SPEED).withTimeout(1),
    new GoXMeter(driveSubsystem, -0.3), 
    new TurnXDegrees(driveSubsystem, 180).withTimeout(2), 
    new ParallelCommandGroup(
    new GoXMeter(driveSubsystem, 4.9),
    new LookDown(armSubsystem, -Constants.ArmConstants.ARM_SPEED).withTimeout(1.5)),
    new InTake(gripperubsystem, Constants.GripperConstants.INTAKE_SPEED).withTimeout(2),
    new TurnXDegrees(driveSubsystem, 0).withTimeout(2),
    new ParallelCommandGroup(new GoXMeter(driveSubsystem, 4.9), 
    new LookUp(armSubsystem, Constants.ArmConstants.ARM_SPEED).withTimeout(0.5)),
    new Shoot(gripperubsystem, Constants.GripperConstants.SHOOT_SPEED).withTimeout(1)));

  }

  private void configureBindings() {

    new JoystickButton(joystick, 1).whileTrue(new LookUp(armSubsystem, Constants.ArmConstants.ARM_SPEED).repeatedly());
    new JoystickButton(joystick, 2).whileTrue(new LookDown(armSubsystem, -Constants.ArmConstants.ARM_SPEED).repeatedly());
    new JoystickButton(joystick, 3).whileTrue(new Shoot(gripperubsystem, Constants.GripperConstants.SHOOT_SPEED).repeatedly());
    new JoystickButton(joystick, 4).whileTrue(new InTake(gripperubsystem, Constants.GripperConstants.INTAKE_SPEED).repeatedly());

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
