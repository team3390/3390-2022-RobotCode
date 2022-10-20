package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;

public final class Constants {

    /**
     * Şasinin mekanik ve ölçümsel değerleri
     */
    public static final double CHASIS_WHEEL_RADIUS_METERS = Units.inchesToMeters(6.0);
    public static final double CHASIS_METERS_BETWEEN_WHEELS = 0.295;

    /**
     * Otonom ve rota izleme değerleri
     */
    public static final double AUTO_MOTOR_FEED_KS = 0.15;
    public static final double AUTO_MOTOR_FEED_KV = 0.1;
    public static final double AUTO_PID_KP = 0.35;
    public static final double AUTO_PID_KI = 0.005;
    public static final double AUTO_PID_KD = 0;
    public static final double AUTO_RAMSETE_CONFIG_B    = 2;
    public static final double AUTO_RAMSETE_CONFIG_ZETA = 0.7;
    public static final double AUTO_TRAJECTORY_MAX_VELMET = 0.5;
    public static final double AUTO_TRAJECTORY_MAX_ACCMETSQ = 0.1;

    /**
     * Limelight değerleri
     */
    public static final double LIMELIGHT_PID_KP        = 0.4;
    public static final double LIMELIGHT_PID_KI        = 0.01;
    public static final double LIMELIGHT_PID_KD        = 0;
    public static final double LIMELIGHT_PID_TOLERANCE = 0;
    public static final double LIMELIGHT_PID_MAX_OUT   = 0.7;
    public static final double LIMELIGHT_PID_MIN_OUT   = -0.7;
    public static final double LIMELIGHT_SHOOTER_COEFFICIENT = 12.759;
    public static enum LIMELIGHT_LIGHT_MODE { PIPELINE_VALUE, OFF, BLINK, ON }
    public static enum LIMELIGHT_CAMERA_MODE { VISION, DRIVE }
    public static enum LIMELIGHT_LOCK_MODE { X, Y, ALL }

    /**
     * Sürüş motorları ID'lerinin tanımlanması
     */
    public static final int DRIVE_LEFT_LEADER_ID  = 15;
    public static final int DRIVE_LEFT_MOTOR2_ID  = 14;
    public static final int DRIVE_LEFT_MOTOR3_ID  = 13;
    public static final int DRIVE_RIGHT_LEADER_ID = 0;
    public static final int DRIVE_RIGHT_MOTOR2_ID = 1;
    public static final int DRIVE_RIGHT_MOTOR3_ID = 2;
    public static final boolean DRIVE_LEFT_INVERTED  = true;
    public static final boolean DRIVE_RIGHT_INVERTED = false;

    /**
     * Şasi dişli kutularına bağlı encoder portlarının tanımlanması
     * ve gerekli ayarlamalar
     */
    public static final double DRIVE_ENCODER_PULSE_COEFFICIENT = 4.8;
    public static final int[] DRIVE_LEFT_ENCODER_PORTS  = { 0, 1 };
    public static final int[] DRIVE_RIGHT_ENCODER_PORTS = { 2, 3 };
    public static final boolean DRIVE_LEFT_ENCODER_INVERTED = false;
    public static final boolean DRIVE_RIGHT_ENCODER_INVERTED = true;
    public static final EncodingType DRIVE_LEFT_ENCODER_ENCODING_TYPE  = EncodingType.k2X;
    public static final EncodingType DRIVE_RIGHT_ENCODER_ENCODING_TYPE = EncodingType.k2X;
    public static final double DRIVE_LEFT_ENCODER_DISTANCE_PER_PULSE  = 2 * Math.PI * CHASIS_WHEEL_RADIUS_METERS / 4096 * DRIVE_ENCODER_PULSE_COEFFICIENT;
    public static final double DRIVE_RIGHT_ENCODER_DISTANCE_PER_PULSE = 2 * Math.PI * CHASIS_WHEEL_RADIUS_METERS / 4096 * DRIVE_ENCODER_PULSE_COEFFICIENT;

    /**
     * NavX Sensor
     */
    public static final Port NAVX_SENSOR_PORT = Port.kMXP;

    /**
     * Top alış motoru ID'sinin tanımlanması
     */
    public static final int INTAKE_MOTOR_ID = 12;

    /**
     * Top besleme ve atma motorlarının ID'lerinin tanımlanması
     */
    public static final int SHOOTER_SHOOT_MOTOR1_ID     = 16;
    public static final int SHOOTER_SHOOT_MOTOR2_ID     = 18;
    public static final int SHOOTER_FEED_TOP_MOTOR1_ID  = 17;
    public static final int SHOOTER_FEED_DOWN_MOTOR1_ID = 19;
    public static final int SHOOTER_FEED_DOWN_MOTOR2_ID = 20;
    public static final int SHOOTER_FEED_DOWN_MOTOR3_ID = 3;
    public static final double SHOOTER_SHOOT_PID_KP = 0.4;
    public static final double SHOOTER_SHOOT_PID_KI = 0.01;
    public static final double SHOOTER_SHOOT_PID_KD = 0;
    public static final double SHOOTER_SHOOT_PID_TOLERANCE = 10;
    public static final double SHOOTER_SHOOT_PID_MAXOUT = 0.4;
    public static final double SHOOTER_SHOOT_PID_MINOUT = 0.05;
    public static final double SHOOTER_FEED_TOP_SPEED   = 0.5;
    public static final double SHOOTER_FEED_DOWN_SPEED  = 0.3;
    public static final double SHOOTER_FEED_DOWN2_SPEED = 1;

    /**
     * Pnömatik
     */
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.CTREPCM;
    public static final int PNEUMATICS_VITESSE_PORT = 0;
    public static final int PNEUMATICS_INTAKE_PORT  = 1;

}
