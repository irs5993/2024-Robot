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
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class RobotContainer {
  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.JOYSTICK_PORT);
  private final CommandPS4Controller gamepad = new CommandPS4Controller(1);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ConveyorSubsystem conveyorSubsystem = new ConveyorSubsystem();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();

  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();
    configureCommands();
    configureDashboard();
    configureGamepad();
  }

  // Joystick tuş atamaları
  private void configureBindings() {
    // Fırlatıcı motorları verilen hız değerleri ile çalıştırın
    joystick.trigger().whileTrue(
        new ShootDistanceCommand(shooterSubsystem, visionSubsystem).alongWith(new LEDFallingPixels(ledSubsystem)));

    // Amp'a atış
    // Oyun parçasını dışarı çıkarırken atıcı motorlarını yavaşça çalıştırır
    joystick.button(2).whileTrue(
        new ShootCommand(shooterSubsystem, () -> 0.15, () -> 0.15)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5)));

    // Game Piece'i içeri almak
    joystick.button(11).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.8));
    // Game Piece'i dışarı itmek
    joystick.button(12).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.8));
    // Game Piece'i shootera beslemek
    joystick.button(13).whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.4));

    // Top sıkışınca geri al lütfen🆙
    joystick.button(9).whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.6)
        .alongWith(new ShootCommand(shooterSubsystem, () -> -0.2, () -> -0.2)));

    // Kolun yukarıya doğru hareket ettirilmesi
    // İstenilen kol açısını periyodik olarak artırarak ->
    // PID contollerın motor voltajlarını otomatik olarak ayarlamasına olanak tanır
    // pozisyon kontrolü, geçirilen speed değeri voltaj değil
    joystick.povUp().whileTrue(new StepArmCommand(armSubsystem, 0.5));

    // Kolun aşağı doğru hareket ettirilmesi
    // İstenilen kol açısını periyodik olarak azaltarak ->
    // PID kontrol cihazının motor voltajlarını otomatik olarak ayarlamasına olanak
    // tanır
    joystick.povDown().whileTrue(new StepArmCommand(armSubsystem, -0.4));

    // Oyun parçasını otomatik olarak alma
    // Conveyor motorlarını çalıştırırken oyun parçasını ->
    // kamera üzerinde yatay olarak ortalar
    joystick.button(4).whileTrue(new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
        .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.35)));

    // Speaker'a nişan almak
    // Hedef eğimi tarafından hesaplanan kol açısını ayarlarken ->
    // aynı zamanda hedefi yatay olarak kamera üzerinde ortalar
    joystick.button(3).whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem).repeatedly()
        .alongWith(new MoveArmVisionCommand(armSubsystem, visionSubsystem)));

    // Kol Ön Ayarları
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

  // Gamepad Komutları - Arhan
  private void configureGamepad() {

    // Atış tekerleri çalıştırma fonksiyonu, RT'ye atanmış.
    gamepad.R2().whileTrue(
        new ShootDistanceCommand(shooterSubsystem, visionSubsystem).alongWith(new LEDFallingPixels(ledSubsystem)));

    // AMP'e atış yapmak için atış motorları ve conveyor fonksiyonu, LT'ye atanmış.
    gamepad.L2().whileTrue(
        new ShootCommand(shooterSubsystem, () -> 0.15, () -> 0.15)
            .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.5)));

    // Intake ve conveyoru çalıştırmak için fonksiyon, A'ya atanmış
    gamepad.cross().whileTrue(new RunConveyorCommand(conveyorSubsystem, -0.8));

    // Vision ile otomatik intake için fonksiyon, START tuşuna atanmış.
    gamepad.options().whileTrue(new DriveCenterNoteCommand(drivetrainSubsystem, visionSubsystem, 0.6)
        .alongWith(new RunConveyorCommand(conveyorSubsystem, -0.35)));

    // Halkayı geri bırakmak için fonksiyon, B'ye atanmış.
    gamepad.circle().whileTrue(new RunConveyorCommand(conveyorSubsystem, 0.6)
        .alongWith(new ShootCommand(shooterSubsystem, () -> -0.2, () -> -0.2)));

    // Vision ile atış için otomatik robot ve kol hizalama fonksiyonu, RB tuşuna
    // atanmış.
    gamepad.R1().whileTrue(new CenterTargetCommand(drivetrainSubsystem, visionSubsystem).repeatedly()
        .alongWith(new MoveArmVisionCommand(armSubsystem, visionSubsystem)));

    // AMP'e atış yapmak için kol hizalama tuşu, LB'ye atanmış.
    gamepad.L1().whileTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MAX_POSITION));

    // Kolu intake açısına hizalamak için fonksiyon, X'e atanmış.
    gamepad.square().whileTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MIN_POSITION));

    // Kolu yukarı oynatmak için fonksiyon, Yukarı tuşuna atanmış.
    gamepad.povUp().whileTrue(new StepArmCommand(armSubsystem, 0.5));

    // Kolu aşağı oynatmak için fonksiyon, Aşağı tuşuna atanmış.
    gamepad.povDown().whileTrue(new StepArmCommand(armSubsystem, -0.4));

    // Kolu maç içi hareket pozisyonuna getirmek için fonksiyon, Sol tuşuna atanmış.
    gamepad.povLeft().whileTrue(new SetArmPositionCommand(armSubsystem, () -> Constants.Arm.MOVEMENT_POSITION));

    // Kolu yaslanmış şekilde atış yapabilecek açıya getirmek için fonksiyon, Sağ
    // tuşuna atanmış.
    gamepad.povRight().whileTrue(new SetArmPositionCommand(armSubsystem,
        () -> Constants.Arm.DEFAULT_SHOOT_POSITION));

    // // Robot önünü değiştir
    // gamepad.triangle().onTrue(
    // Commands.runOnce(() -> drivetrainSubsystem.reverseDirection =
    // !drivetrainSubsystem.reverseDirection,
    // drivetrainSubsystem));
  }

  private void configureDashboard() {
    // ... (dashboard configuration)

  }

  public Command getAutonomousCommand() {
    // ... (autonomous command)
    return autoChooser.getSelected();
  }

}
