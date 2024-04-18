// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Commands {

  }

  public static class Arm {
    // Kol hareket limitleri için konum değerleri
    public static final double MAX_POSITION = 0.2;
    public static final double MIN_POSITION = 0;

    // Robot hareketliyken kol pozisyonu
    public static final double MOVEMENT_POSITION = 0.15;

    // Oyun parçasını human playerdan almak için kolun konumu
    public static final double HUMAN_POSITION = 0.141;

    // Robot tamponlarını hoparlöre dokundurduğunda kolun varsayılan fırlatma konumu
    public static final double DEFAULT_SHOOT_POSITION = 0.028;

    public static final double STAGE_SHOOT_POSITION = 0.074;

    // PID döngüsü tarafından ayarlanan kol konumunun ->
    // ne zaman (veya olup olmadığını) istenen konumda olduğunu belirler
    // Bunu artırmak, kolun konum güncellemelerine ->
    // daha az doğru yanıt vermesine neden olur
    public static final double CONTROLLER_TOLERANCE = 0.001;

    // Kol motoru voltajlarında net sınır.
    // Tüm yazılım limitleri başarısız olursa bu, motor kontrolörlerinde izin
    // verilen maksimum voltajdır
    public static final double SAFETY_MAX_VOLTAGE = 0.75;

    // Kol konumu hesaplaması için ofset. Arhan Burak Tüzün®™ ->
    // olmadığınız sürece bunu DEĞİŞTİRMEYİN
    public static final double ANGLE_OFFSET = -15;

    public static final int SWITCH_PORT = 6;
  }

  public static class OperatorConstants {
    public static final int JOYSTICK_PORT = 0;
  }

  public static class Vision {
    public static final int PIPELINE_APRILTAG = 0;
  }

  public static class DriverPorts {
    public static final int CHASIS_LEAD_LEFT = 2;
    public static final int CHASIS_FOLLOWER_LEFT = 0;

    public static final int CHASIS_LEAD_RIGHT = 1;
    public static final int CHASIS_FOLLOWER_RIGHT = 3;

    public static final int CONVEYOR = 7;
  }

  public static class CANIDS {
    public static final int SHOOTER_TOP = 6;
    public static final int SHOOTER_BOTTOM = 5;

    public static final int ARM_RIGHT = 3;
    public static final int ARM_LEFT = 4;
    public static final int ARM_ENCODER = 10;
  }

}
