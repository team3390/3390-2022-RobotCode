package frc.robot.subystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.PID;

public class BallSubsystem extends SubsystemBase {

  private final CANSparkMax shooter1 = new CANSparkMax(Constants.SHOOTER_SHOOT_MOTOR1_ID, MotorType.kBrushless);
  private final CANSparkMax shooter2 = new CANSparkMax(Constants.SHOOTER_SHOOT_MOTOR2_ID, MotorType.kBrushless);
  private final CANSparkMax feederTop1 = new CANSparkMax(Constants.SHOOTER_FEED_TOP_MOTOR1_ID, MotorType.kBrushless);
  private final CANSparkMax feederDown1 = new CANSparkMax(Constants.SHOOTER_FEED_DOWN_MOTOR1_ID, MotorType.kBrushless);
  private final CANSparkMax feederDown2 = new CANSparkMax(Constants.SHOOTER_FEED_DOWN_MOTOR2_ID, MotorType.kBrushless);
  private final WPI_TalonSRX feederDown3 = new WPI_TalonSRX(Constants.SHOOTER_FEED_DOWN_MOTOR3_ID);

  public final Solenoid intakeSolenoid = new Solenoid(Constants.PNEUMATICS_MODULE_TYPE, Constants.PNEUMATICS_INTAKE_PORT);
  public final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKE_MOTOR_ID);

  private final RelativeEncoder shooterEncoder = shooter1.getEncoder();

  private final PID speedPID;
  private final LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;

  public BallSubsystem() {
    feederTop1.setInverted(true);
    shooter2.setInverted(true);
    feederDown1.setInverted(true);

    double kp = Constants.SHOOTER_SHOOT_PID_KP;
    double ki = Constants.SHOOTER_SHOOT_PID_KI;
    double kd = Constants.SHOOTER_SHOOT_PID_KD;
    double max = Constants.SHOOTER_SHOOT_PID_MAXOUT;
    double min = Constants.SHOOTER_SHOOT_PID_MINOUT;
    speedPID = new PID(kp, ki, kd, Constants.SHOOTER_SHOOT_PID_TOLERANCE, max, min);
  }

  @Override
  public void periodic() {}

  public void shoot() {
    if (speedPID.getSetpoint() != null) {
      if (!speedPID.atSetpoint()) {
        double setpoint = limelight.calculateShooterRPM();
        double output = speedPID.output(speedPID.calculate(shooterEncoder.getVelocity(), setpoint));
  
        shooter1.set(output);
        shooter2.set(output);
      } else {
        this.startFeedingToTop();
      }
    } else {
      this.startShooter();
      this.startFeedingToTop();
    }
  }

  public void startShooter() {
    shooter1.set(0.3);
    shooter2.set(0.3);
  }

  public void startFeedingToTop() {
    feederTop1.set(Constants.SHOOTER_FEED_TOP_SPEED);
    feederDown1.set(Constants.SHOOTER_FEED_DOWN_SPEED);
    feederDown2.set(Constants.SHOOTER_FEED_DOWN_SPEED);
    feederDown3.set(Constants.SHOOTER_FEED_DOWN2_SPEED);
  }

  public void startFeedingToDown() {
    feederTop1.set(-1 * (Constants.SHOOTER_FEED_TOP_SPEED / 2));
    feederDown1.set(-1 * (Constants.SHOOTER_FEED_DOWN_SPEED / 2));
    feederDown2.set(-1 * (Constants.SHOOTER_FEED_DOWN_SPEED / 2));
  }

  public void stopAll() {
    stopShooter();
    stopFeeding();
  }

  public void stopShooter() {
    shooter1.stopMotor();
    shooter2.stopMotor();
  }

  public void stopFeeding() {
    feederTop1.stopMotor();
    feederDown1.stopMotor();
    feederDown2.stopMotor();
    feederDown3.stopMotor();
  }
}
