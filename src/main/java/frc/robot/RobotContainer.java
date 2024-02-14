// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.DynamicDriveCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  public RobotContainer() {
    configureBindings();
    configureCommands();
  }

  private void configureBindings() {
    joystick.trigger().whileTrue(new ShootCommand(shooterSubsystem, () -> joystick.getRawAxis(3)));

    joystick.button(6).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.5)); // TAKE IN
    joystick.button(4).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.5)); // PUSH OUT
    
    joystick.button(5).whileTrue(new MoveArmCommand(armSubsystem, 0.5)); // MOVE UP
    joystick.button(3).whileTrue(new MoveArmCommand(armSubsystem, -0.5)); // MOVE DOWN
  }

  private void configureCommands() {
    drivetrainSubsystem.setDefaultCommand(new DynamicDriveCommand(drivetrainSubsystem, joystick::getY, joystick::getZ));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return Autos.hayalGucunuKullan(drivetrainSubsystem);
  }
}
