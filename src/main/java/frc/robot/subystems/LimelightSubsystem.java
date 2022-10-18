package frc.robot.subystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.PID;

public class LimelightSubsystem extends SubsystemBase {
  private final NetworkTable networkTable;

  private final NetworkTableEntry tX; // Derece cinsinden dikey
  private final NetworkTableEntry tY; // Derece cinsinden yatay
  private final NetworkTableEntry tV; // Herhangi bir hedef var mı? (0, 1)
  private final NetworkTableEntry tA; // Hedefin kamerada ne kadar alan kapladığı
  private final NetworkTableEntry tL; // Limelight pipeline ping

  private final PID xPID;
  private final PID yPID;

  private double lastTargetX;
  private double lastTargetY;

  public static LimelightSubsystem INSTANCE = new LimelightSubsystem();

  public LimelightSubsystem() {
    networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    tX = networkTable.getEntry("tx");
    tY = networkTable.getEntry("ty");
    tV = networkTable.getEntry("tv");
    tA = networkTable.getEntry("ta");
    tL = networkTable.getEntry("tl");

    xPID = new PID(Constants.LIMELIGHT_PID_KP, Constants.LIMELIGHT_PID_KI, Constants.LIMELIGHT_PID_KD, Constants.LIMELIGHT_PID_TOLERANCE, Constants.LIMELIGHT_PID_MAX_OUT, Constants.LIMELIGHT_PID_MIN_OUT);
    yPID = new PID(Constants.LIMELIGHT_PID_KP, Constants.LIMELIGHT_PID_KI, Constants.LIMELIGHT_PID_KD, Constants.LIMELIGHT_PID_TOLERANCE, Constants.LIMELIGHT_PID_MAX_OUT, Constants.LIMELIGHT_PID_MIN_OUT);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LM-X-Output", getXOutput());
    SmartDashboard.putBoolean("LM-X-AtSetpoint", xPID.atSetpoint());

    SmartDashboard.putNumber("LM-Y-Output", getYOutput());
    SmartDashboard.putBoolean("LM-Y-AtSetpoint", yPID.atSetpoint());

    SmartDashboard.putBoolean("LM-isTarget", isTarget());
    SmartDashboard.putNumber("LM-Target-Area", tA.getDouble(0));
    SmartDashboard.putNumber("LM-Pipeline-Ping", tL.getDouble(0));
  }

  /**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	public NetworkTableEntry getValue(String key) {
		return networkTable.getEntry(key);
	}

  /**
   * Herhangi bir hedef olup olmadığını döndürür.
   * @return erğer hedef var ise true yok ise false
   */
  public boolean isTarget() {
    return tV.getDouble(0) == 1;
  }

  /**
   * LED modunu ayarlar
   * @param mode LED modu
   */
  public void setLedMode(Constants.LIMELIGHT_LIGHT_MODE mode) {
    getValue("ledMode").setNumber(mode.ordinal());
  }

  /**
   * Kamera modunu ayarlar
   * @param mode Kamera modu
   */
  public void setCamMode(Constants.LIMELIGHT_CAMERA_MODE mode) {
    getValue("camMode").setNumber(mode.ordinal());
  }

  /**
   * Pipelineı değiştirir (0-9)
   * @param number Pipeline ID
   */
  public void setPipeline(int number) {
    getValue("pipeline").setNumber(number);
  }

  /**
   * Eğer crosshair hedefin ortasına geldiyse true olarak döndürüyor.
   * Tolerance değerine göre değişir bu kısım
   * @return hedef noktada ise true, değilse false
   */
  public boolean atSetpoint() {
    return xPID.atSetpoint() && yPID.atSetpoint();
  }

  /**
   * Hedef noktaya ulaşılması için X ekseninde motorların ne kadar dönmesi gerektiğini döndürür.
   * @return yüzdelik güç
   */
  public double getXOutput() {
    if (this.isTarget()) {
      if (!xPID.atSetpoint()) {
        double x = tX.getDouble(0);
        this.lastTargetX = x;
        return xPID.output(xPID.calculate(x, 0));
      }
      return 0;
    }
    return xPID.output(xPID.calculate(this.lastTargetX, 0));
  }

  /**
   * Hedef noktaya ulaşılması için Y ekseninde motorların ne kadar dönmesi gerektiğini döndürür.
   * @return yüzdelik güç
   */
  public double getYOutput() {
    if (this.isTarget()) {
      if (!yPID.atSetpoint()) {
        double y = tY.getDouble(0);
        this.lastTargetY = y;
        return yPID.output(yPID.calculate(y, 0));
      }
      return 0;
    }
    return yPID.output(yPID.calculate(this.lastTargetY, 0));
  }

  /**
   * Hedefin kamerada kapladığı alana göre atıcı motorunun ne kadar
   * dönmesi gerktiğini hesaplayan fonksiyon
   * @return RPM
   */
  public int calculateShooterRPM() {
    return (int) Math.round((100 - tA.getDouble(0)) * Constants.LIMELIGHT_SHOOTER_COEFFICIENT);
  }
}
