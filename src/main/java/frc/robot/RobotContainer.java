package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CenterTargetCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.arm.AdjustArmVisionCommand;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.StepArmCommand;
import frc.robot.helpers.RMath;
import frc.robot.commands.DynamicDriveCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
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
    joystick.trigger().whileTrue(new ShootCommand(shooterSubsystem, () -> 0.6, () -> 0.6)); // SHOOT OUT

    joystick.button(12).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.5)); // TAKE IN
    joystick.button(13).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.5)); // PUSH OUT

    joystick.povUp().whileTrue(new StepArmCommand(armSubsystem, 0.5)); // ARM UP
    joystick.povDown().whileTrue(new StepArmCommand(armSubsystem, -0.4)); // ARM DOWN

    joystick.button(3).whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem)
        .alongWith(new AdjustArmVisionCommand(armSubsystem, visionSubsystem)));

    // Arm Presets
    // ---------------------------------------------------------------------
    joystick.button(14)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> 0.165));
    joystick.button(15)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> 0.004));
    joystick.button(16)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.DEFAULT_SHOOT_POSITION));
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
        new DynamicDriveCommand(drivetrainSubsystem, joystick::getY, joystick::getZ, () -> joystick.getRawAxis(3)));
  }

  private void configureDashboard() {
    // ... (dashboard configuration)

  }

  public Command getAutonomousCommand() {
    // ... (autonomous command)
    return Autos.hayalGucunuKullan(drivetrainSubsystem);
  }

}
