package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.helpers.RMath;
import frc.robot.commands.DynamicDriveCommand;
import frc.robot.commands.MoveArmCommand;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
    joystick.trigger().whileTrue(new ShootCommand(shooterSubsystem, () -> joystick.getRawAxis(3)));

    joystick.button(6).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.5)); // TAKE IN
    joystick.button(4).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.5)); // PUSH OUT

    joystick.button(5).whileTrue(new MoveArmCommand(armSubsystem, 0.2)); // ARM UP
    joystick.button(3).whileTrue(new MoveArmCommand(armSubsystem, -0.2)); // ARM DOWN

    joystick.button(11).whileTrue(new SetArmPositionCommand(armSubsystem, () -> RMath.map(joystick.getRawAxis(3), 1, -1, 0.46, 0.62)));
    // joystick.button(7).whileTrue(new TargetAndShootCommand(visionSubsystem,
    // drivetrainSubsystem, armSubsystem));
  }

  private void configureCommands() {
    drivetrainSubsystem.setDefaultCommand(new DynamicDriveCommand(drivetrainSubsystem, joystick::getY, joystick::getZ));
  }

  private void configureDashboard() {
    // ... (dashboard configuration)
  }

  public Command getAutonomousCommand() {
    // ... (autonomous command)
    return Autos.hayalGucunuKullan(drivetrainSubsystem);
  }

  
}
