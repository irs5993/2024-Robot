package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.KeepArmPositionCommand;
import frc.robot.commands.arm.MoveArmVisionCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.StepArmCommand;
import frc.robot.commands.drive.CenterTargetCommand;
import frc.robot.commands.drive.DriveCenterNoteCommand;
import frc.robot.commands.drive.DynamicDriveCommand;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.commands.shoot.ShootVelocityCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  public RobotContainer() {
    configureBindings();
    configureCommands();
    configureDashboard();
  }

  private void configureBindings() {
    joystick.trigger().whileTrue(
        new ShootVelocityCommand(shooterSubsystem, () -> 0.66, () -> 0.66)); // SHOOT

    joystick.button(2).whileTrue(
        new ShootCommand(shooterSubsystem, () -> 0.15, () -> 0.15)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5))); // SHOOT

    // OUT

    joystick.button(12).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.8)); // PUSH OUT
    joystick.button(13).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.8)); // TAKE IN

    joystick.povUp().whileTrue(new StepArmCommand(armSubsystem, 0.5))
        .whileFalse(new KeepArmPositionCommand(armSubsystem)); // ARM UP
    joystick.povDown().whileTrue(new StepArmCommand(armSubsystem, -0.4))
        .whileFalse(new KeepArmPositionCommand(armSubsystem)); // ARM DOWN

    joystick.button(4).whileTrue(new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
        .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.45)));

    joystick.button(3).whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem)
        .alongWith(new MoveArmVisionCommand(armSubsystem, visionSubsystem)))
        .whileFalse(new KeepArmPositionCommand(armSubsystem));

    // Arm Presets
    // ---------------------------------------------------------------------
    joystick.button(6)
        .whileTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.HUMAN_POSITION)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5)));
    joystick.button(7)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MAX_POSITION));
    joystick.button(14)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> 0.15));
    joystick.button(15)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MIN_POSITION));
    joystick.button(16)
        .onTrue(new SetArmPositionCommand(armSubsystem,
            () -> Constants.Arm.DEFAULT_SHOOT_POSITION));
    // ---------------------------------------------------------------------

    // joystick.button(13).whileTrue(new MoveArmCommand(armSubsystem, 0.2));
    // joystick.button(12).whileTrue(new MoveArmCommand(armSubsystem, -0.2));

    // joystick.button(16)
    // .whileTrue(new AdjustArmVisionCommand(armSubsystem, visionSubsystem));

    // joystick.button(4)
    // .whileTrue(new SetArmPositionCommand(armSubsystem, () ->
    // RMath.map(joystick.getRawAxis(3), 1, -1, 0.004, 0.2)));

  }

  private void configureCommands() {
    drivetrainSubsystem.setDefaultCommand(
        new DynamicDriveCommand(drivetrainSubsystem, joystick::getY, joystick::getZ,
            () -> 1));
  }

  private void configureDashboard() {
    // ... (dashboard configuration)

  }

  public Command getAutonomousCommand() {
    // ... (autonomous command)
    return new WaitCommand(2);
  }

}
