package frc.robot.subystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  public static Drivetrain INSTANCE = new Drivetrain();

  /**
   * Hızkontrol sürücülerinin tanımlanması
   */
  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(Constants.DRIVE_LEFT_LEADER_ID);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(Constants.DRIVE_LEFT_MOTOR2_ID);
  private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(Constants.DRIVE_LEFT_MOTOR3_ID);
  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(Constants.DRIVE_RIGHT_LEADER_ID);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(Constants.DRIVE_RIGHT_MOTOR2_ID);
  private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(Constants.DRIVE_RIGHT_MOTOR3_ID);

  /**
   * Kullanılan sensörlerin tanımlanması
   */
  private final int[] leftEncoderPorts = Constants.DRIVE_LEFT_ENCODER_PORTS;
  private final int[] rightEncoderProts = Constants.DRIVE_RIGHT_ENCODER_PORTS;
  private final Encoder leftEncoder = new Encoder(leftEncoderPorts[0], leftEncoderPorts[1], Constants.DRIVE_LEFT_ENCODER_INVERTED, Constants.DRIVE_LEFT_ENCODER_ENCODING_TYPE);
  private final Encoder rightEncoder = new Encoder(rightEncoderProts[0], rightEncoderProts[1], Constants.DRIVE_RIGHT_ENCODER_INVERTED, Constants.DRIVE_RIGHT_ENCODER_ENCODING_TYPE);

  private final AHRS navX = new AHRS(Constants.NAVX_SENSOR_PORT);

  /**
   * Robot objeleri oluşturuyoruz.
   */
  private final double tekerleklerArasiMesafe = 0.295; // Metre cinsinden
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(tekerleklerArasiMesafe);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  /**
   * Limelight
   */
  public final LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;

  /**
   * Consturcter
   * 
   * Initialize fonksiyonu olarak düşünebilirsiniz.
   * Bu sınıf çağırıldığında(robot objesi içerisinde 1 kere) 1 kere çalışır
   */
  public Drivetrain() {
    drive.setSafetyEnabled(false);

    leftLeader.setInverted(Constants.DRIVE_LEFT_INVERTED);
    leftMotor2.setInverted(Constants.DRIVE_LEFT_INVERTED);
    leftMotor3.setInverted(Constants.DRIVE_LEFT_INVERTED);
    rightLeader.setInverted(Constants.DRIVE_RIGHT_INVERTED);
    rightMotor2.setInverted(Constants.DRIVE_RIGHT_INVERTED);
    rightMotor3.setInverted(Constants.DRIVE_RIGHT_INVERTED);

    leftMotor2.follow(leftLeader);
    leftMotor3.follow(leftLeader);
    rightMotor2.follow(rightLeader);
    rightMotor3.follow(rightLeader);

    rightEncoder.setDistancePerPulse(Constants.DRIVE_LEFT_ENCODER_DISTANCE_PER_PULSE);
    leftEncoder.setDistancePerPulse(Constants.DRIVE_RIGHT_ENCODER_DISTANCE_PER_PULSE);

    resetSensors();
  }

  /**
   * periodic() fonksiyonu bu sınıf çalışırken sürekli çalışan method.
   */
  @Override
  public void periodic() {
    odometry.update(getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("LeftEncoder Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("RightEncoder Distance", rightEncoder.getDistance());
  }

  /**
   * Robotun kinematiklerini döndürür.
   * @return kinematikler
   */
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Robotun şu anki lokasyonunu belirtilen lokasyon ile değiştirir
   * @param pose şu anki robotun lokasyonu
   */
  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();

    odometry.resetPosition(pose, getHeading());
  }

  /**
   * Bütün sensörlerin değerlerini 0 yapar.
   */
  public void resetSensors() {
    rightEncoder.reset();
    leftEncoder.reset();
    navX.reset();
  }

  /**
   * NavX sensörüne göre robotun şu anki açısını döndürür.
   * @return robotun açısı
   */
  public double getAngle() {
    return navX.getAngle();
  }

  /**
   * Robotun şu anki açısına bakarak kordinat sistemindeki açısını hesaplar.
   * Robotun nereye doğru baktığını döndürür.
   * @return Robotun açısı
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(navX.getAngle());
  }

  /**
   * Robotun kordinat sistemindeki koordinatlarının ne olduğunu döndürür
   * @return Kordinatlar
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Sol encoder değerini döndürür
   * @return sol encoder değeri
   */
  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }

  /**
   * Sağ encoder değerini döndürür
   * @return sağ encoder değeri
   */
  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }

  /**
   * Tekerleklerin ne hızda döndüklerini döndürür
   * @return tekerlek hızları
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Bütün motorları durdurur.
   */
  public void stopMotors() {
    drive.stopMotor();
  }

  /**
   * Robotun arcade tarzda sürülmesini sağlar
   * @param fwd X ekseninde hız (%)
   * @param rot Y ekseninde hız (%)
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Robotun tank tarzında sürülmesini sağlar
   * @param leftPercent Sol tekereklere gidecek hız (%)
   * @param rightPercent Sağ tekereklere gidecek hız (%)
   */
  public void tankDrivePercent(double leftPercent, double rightPercent) {
    drive.tankDrive(leftPercent, rightPercent);
  }

  /**
   * Robotun tank tarzında sürülmesini sağlar
   * Burada yüzdelik bir şekilde güç vermek yerine voltaj değerlerine göre güç verir.
   * 
   * Dikkat!
   * Bir anda çok yüksek voltaj vermek robotun aküsünün 10V un altına düşmesine neden olabilir!
   * Bu yüzden sınırlandırma konulmalıdır.
   * 
   * @param leftVolts Sol motorlara verilecek voltaj değeri
   * @param rightVolts Sağ motorlara verilecek voltaj değeri
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    drive.setSafetyEnabled(false);
    leftLeader.setVoltage(-1 * leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed();
  }

  /**
   * Limelight değerlerine göre hedefe şasi ile kitlenme
   */
  public void lockTarget(Constants.LIMELIGHT_LOCK_MODE mode) {
    if (!limelight.atSetpoint()) {
      double xSpeed = limelight.getXOutput();
      double ySpeed = limelight.getYOutput();
      
      switch (mode.ordinal()) {
        case 0:
          this.arcadeDrive(0, xSpeed);
          break;
        
        case 1:
          this.arcadeDrive(ySpeed, 0);
          break;
      
        default:
          this.arcadeDrive(ySpeed, xSpeed);
          break;
      }
    }
  }

  /**
   * Limelight hedef nokta da mı değil mi
   * @return Hedef noktada olup oladığımız
   */
  public boolean isLimelightAtSetpoint() {
    return limelight.atSetpoint();
  }

}
