package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.KeepArmPositionCommand;
import frc.robot.commands.arm.MoveArmCommand;
import frc.robot.commands.arm.MoveArmVisionCommand;
import frc.robot.commands.arm.SetArmPositionCommand;
import frc.robot.commands.arm.StepArmCommand;
import frc.robot.commands.drive.CenterTargetCommand;
import frc.robot.commands.drive.DriveCenterNoteCommand;
import frc.robot.commands.drive.DynamicDriveCommand;
import frc.robot.commands.led.LEDFallingPixels;
import frc.robot.commands.led.LEDLockedCommand;
import frc.robot.commands.led.LEDRainbowCommand;
import frc.robot.commands.shoot.ShootCommand;
import frc.robot.commands.shoot.ShootDistanceCommand;
import frc.robot.commands.shoot.ShootVelocityCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.RunConveyorCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class RobotContainer {
  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);
  private final XboxController gamepad = new XboxController(OperatorConstants.GAMEPAD_PORT);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  private final EventLoop shootDistanceLoop = new EventLoop();
  private final EventLoop ampScoringLoop = new EventLoop();
  private final EventLoop intakeLoop = new EventLoop();
  private final EventLoop autoIntakeLoop = new EventLoop();
  private final EventLoop releaseLoop = new EventLoop();
  private final EventLoop aimLoop = new EventLoop();
  private final EventLoop ampAimLoop = new EventLoop();
  private final EventLoop intakeAngleLoop = new EventLoop();
  private final EventLoop moveUpLoop = new EventLoop();
  private final EventLoop moveDownLoop = new EventLoop();
  private final EventLoop movementPosLoop = new EventLoop();
  private final EventLoop defaultShootLoop = new EventLoop();

  public RobotContainer() {
    configureBindings();
    configureCommands();
    configureDashboard();
    configureGamepad();
  }

  // Joystick tuş atamaları
  private void configureBindings() {
    // DESC - Run the shooter motors with the given velocity values
    joystick.trigger().whileTrue(
        new ShootDistanceCommand(shooterSubsystem, visionSubsystem).alongWith(new LEDFallingPixels(ledSubsystem)));

    // FOR - Scoring on the amp
    // DESC - Run the shooter motors slowly, while moving the game piece out
    joystick.button(2).whileTrue(
        new ShootCommand(shooterSubsystem, () -> 0.15, () -> 0.15)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5)));

    // FOR - Taking the game piece in
    joystick.button(11).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.8));
    // FOR - Pushing the game piece out
    joystick.button(12).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.8));
    // FOR - Feeding the game piece to the shooter
    joystick.button(13).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.4));

    // Top sıkışınca geri al lütfen🆙
    joystick.button(9).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.6)
        .alongWith(new ShootCommand(shooterSubsystem, () -> -0.2, () -> -0.2)));

    // FOR - Moving the arm upwards
    // DESC - Increase the desired arm angle periodically, allowing for the PID->
    // controller to set the motor voltages automatically
    // WARNING - pozisyon kontrolü, geçirilen speed değeri voltaj değil
    joystick.povUp().whileTrue(new StepArmCommand(armSubsystem, 0.5));

    // FOR - Moving the arm downwards
    // DESC - Decrease the desired arm angle periodically, allowing for the PID
    // controller to set the motor voltages automatically
    joystick.povDown().whileTrue(new StepArmCommand(armSubsystem, -0.4));

    // FOR - Automatically taking the game piece in
    // DESC - Center the game piece horizontally on the camera while running the
    // conveyor motors
    joystick.button(4).whileTrue(new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
        .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.35)));

    // FOR - Aiming at the speaker
    // DESC - Center the target horizontally on the camera while also adjusting the
    // arm angle calculated by the target pitch
    joystick.button(3).whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem).repeatedly()
        .alongWith(new MoveArmVisionCommand(armSubsystem, visionSubsystem)));

    // Arm Presets
    // ---------------------------------------------------------------------
    // joystick.button(11)
    // .whileTrue(new SetArmPositionCommand(armSubsystem, () ->
    // Constants.Arm.HUMAN_POSITION)
    // .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5)));
    joystick.button(7)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MAX_POSITION));
    joystick.button(8)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.STAGE_SHOOT_POSITION));
    joystick.button(14)
        .onTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MOVEMENT_POSITION));
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

  // Gamepad Komutları - Arhan
  private void configureGamepad() {

    // Atış tekerleri çalıştırma fonksiyonu, RT'ye atanmış.
    BooleanEvent shootDistance = new BooleanEvent(shootDistanceLoop, gamepad.rightTrigger(shootDistanceLoop));
    shootDistance.ifHigh(() -> new ShootDistanceCommand(shooterSubsystem, visionSubsystem)
        .alongWith(new LEDFallingPixels(ledSubsystem)));

    // AMP'e atış yapmak için atış motorları ve conveyor fonksiyonu, LT'ye atanmış.
    BooleanEvent ampScoring = new BooleanEvent(ampScoringLoop, gamepad.leftTrigger(ampScoringLoop));
    ampScoring.ifHigh(() -> new ShootCommand(shooterSubsystem, () -> 0.15, () -> 0.15)
        .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5)));

    // Intake ve conveyoru çalıştırmak için fonksiyon, A'ya atanmış
    BooleanEvent intakeButton = new BooleanEvent(intakeLoop, gamepad.a(intakeLoop));
    intakeButton.ifHigh(() -> new RunConveyorCommand(conveyorSubsystem, -0.8));

    // Vision ile otomatik intake için fonksiyon, START tuşuna atanmış.
    BooleanEvent autoIntake = new BooleanEvent(autoIntakeLoop, gamepad.start(autoIntakeLoop));
    autoIntake.ifHigh(() -> new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
        .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.35)));

    // Halkayı geri bırakmak için fonksiyon, B'ye atanmış.
    BooleanEvent releaseButton = new BooleanEvent(releaseLoop, gamepad.b(releaseLoop));
    releaseButton.ifHigh(() -> new RunConveyorCommand(conveyorSubsystem, 0.6)
        .alongWith(new ShootCommand(shooterSubsystem, () -> -0.2, () -> -0.2)));

    // Vision ile atış için otomatik robot ve kol hizalama fonksiyonu, RB tuşuna
    // atanmış.
    BooleanEvent aimButton = new BooleanEvent(aimLoop, gamepad.rightBumper(aimLoop));
    aimButton.ifHigh(() -> new CenterTargetCommand(drivetrainSubsystem, visionSubsystem).repeatedly()
        .alongWith(new MoveArmVisionCommand(armSubsystem, visionSubsystem)));

    // AMP'e atış yapmak için kol hizalama tuşu, LB'ye atanmış.
    BooleanEvent ampAimButton = new BooleanEvent(ampAimLoop, gamepad.leftBumper(ampAimLoop));
    ampAimButton.ifHigh(() -> new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MAX_POSITION));

    // Kolu intake açısına hizalamak için fonksiyon, X'e atanmış.
    BooleanEvent intakeAngleButton = new BooleanEvent(intakeAngleLoop, gamepad.x(intakeAngleLoop));
    intakeAngleButton.ifHigh(() -> new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MIN_POSITION));

    // Kolu yukarı oynatmak için fonksiyon, Yukarı tuşuna atanmış.
    BooleanEvent moveUpButton = new BooleanEvent(moveUpLoop, gamepad.povUp(moveUpLoop));
    moveUpButton.ifHigh(() -> new StepArmCommand(armSubsystem, 0.5));

    // Kolu aşağı oynatmak için fonksiyon, Aşağı tuşuna atanmış.
    BooleanEvent moveDownButton = new BooleanEvent(moveDownLoop, gamepad.povDown(moveDownLoop));
    moveDownButton.ifHigh(() -> new StepArmCommand(armSubsystem, -0.4));

    // Kolu maç içi hareket pozisyonuna getirmek için fonksiyon, Sol tuşuna atanmış.
    BooleanEvent movementPosButton = new BooleanEvent(movementPosLoop, gamepad.povLeft(movementPosLoop));
    movementPosButton.ifHigh(() -> new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MOVEMENT_POSITION));

    // Kolu yaslanmış şekilde atış yapabilecek açıya getirmek için fonksiyon, Sağ
    // tuşuna atanmış.
    BooleanEvent defaultShootButton = new BooleanEvent(defaultShootLoop, gamepad.povRight(defaultShootLoop));
    defaultShootButton.ifHigh(() -> new SetArmPositionCommand(armSubsystem,
        () -> Constants.Arm.DEFAULT_SHOOT_POSITION));
  }

  // Sürekli çalışan komutlar burda çalışıyo😍🚑
  private void configureCommands() {
    // Son parametre robot hız çarpanı (0.5 verilirse robot hızı yarı yarıya düşer)
    drivetrainSubsystem.setDefaultCommand(
        new DynamicDriveCommand(drivetrainSubsystem, joystick::getY, joystick::getZ,
            () -> 1));

    armSubsystem.setDefaultCommand(new KeepArmPositionCommand(armSubsystem));

    ledSubsystem.setDefaultCommand(Commands.run(() -> {
      if (visionSubsystem.getSpeakerTarget() != null) {
        ledSubsystem.setRGB(0, 0, 255);
      } else if (visionSubsystem.getBestTargetIntake() != null) {
        ledSubsystem.setRGB(255, 20, 0);
      } else {
        ledSubsystem.rainbow();
      }
    }, ledSubsystem));

    autoChooser.setDefaultOption("Center",
        Autos.CenterAuto(drivetrainSubsystem, visionSubsystem, armSubsystem, shooterSubsystem,
            conveyorSubsystem));
    autoChooser.addOption("Right",
        Autos.RightAuto(drivetrainSubsystem, visionSubsystem, armSubsystem, shooterSubsystem,
            conveyorSubsystem));
    autoChooser.addOption("Left",
        Autos.LeftAuto(drivetrainSubsystem, visionSubsystem, armSubsystem, shooterSubsystem,
            conveyorSubsystem));

    SmartDashboard.putData(autoChooser);

  }

  private void configureDashboard() {
    // ... (dashboard configuration)

  }

  public Command getAutonomousCommand() {
    // ... (autonomous command)
    return autoChooser.getSelected();
  }

}
