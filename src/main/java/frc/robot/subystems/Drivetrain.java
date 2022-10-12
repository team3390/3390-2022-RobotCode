package frc.robot.subystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Motors
  private final WPI_TalonSRX leftLeader = new WPI_TalonSRX(Constants.MotorConstants.kLeftLeader);
  private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(Constants.MotorConstants.kLeftMotor2);
  private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(Constants.MotorConstants.kLeftMotor3);

  private final WPI_TalonSRX rightLeader = new WPI_TalonSRX(Constants.MotorConstants.kRightLeader);
  private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(Constants.MotorConstants.kRightMotor2);
  private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(Constants.MotorConstants.kRightMotor3);

  // Sensors
  private final int[] leftEncoderPorts = Constants.DIO.kLeftEncoderPorts;
  private final int[] rightEncoderProts = Constants.DIO.kRightEncoderPorts;
  private final Encoder leftEncoder = new Encoder(leftEncoderPorts[0], leftEncoderPorts[1], false, EncodingType.k2X);
  private final Encoder rightEncoder = new Encoder(rightEncoderProts[0], rightEncoderProts[1], true, EncodingType.k2X);

  private final AHRS navX = new AHRS(Port.kMXP);

  // Drive - Autonomous
  private final double tekerleklerArasiMesafe = 0.295; // Metre cinsinden
  private final DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(tekerleklerArasiMesafe);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  public Drivetrain() {
    drive.setSafetyEnabled(false);

    leftLeader.setInverted(true);
    leftMotor2.setInverted(true);
    leftMotor3.setInverted(true);

    leftMotor2.follow(leftLeader);
    leftMotor3.follow(leftLeader);

    rightMotor2.follow(rightLeader);
    rightMotor3.follow(rightLeader);

    /**
     * We use 6 inches wheels
     * Circumference of a circle formula = 2πr
     * So: 2 * Math.PI * 6inc
     */
    rightEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6));
    leftEncoder.setDistancePerPulse(2 * Math.PI * Units.inchesToMeters(6));

    resetSensors();
  }

  @Override
  public void periodic() {
    odometry.update(getHeading(), leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("LeftEncoder Distance", leftEncoder.getDistance());
    SmartDashboard.putNumber("RightEncoder Distance", rightEncoder.getDistance());
  }
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  public void resetOdometry(Pose2d pose) {
    leftEncoder.reset();
    rightEncoder.reset();

    odometry.resetPosition(pose, getHeading());
  }

  /**
   * Sensors
   */
  public void resetSensors() {
    rightEncoder.reset();
    leftEncoder.reset();
    navX.reset();
  }
  public double getAngle() {
    return navX.getAngle();
  }
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(navX.getAngle());
  }
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
  public double getLeftEncoderDistance() {
    return leftEncoder.getDistance();
  }
  public double getRightEncoderDistance() {
    return rightEncoder.getDistance();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  /**
   * Drive
   */
  public void stopMotors() {
    drive.stopMotor();
  }
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }
  public void tankDrivePercent(double leftPercent, double rightPercent) {
    drive.tankDrive(leftPercent, rightPercent);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    drive.setSafetyEnabled(false);
    leftLeader.setVoltage(leftVolts);
    rightLeader.setVoltage(rightVolts);
    drive.feed();
  }

}
