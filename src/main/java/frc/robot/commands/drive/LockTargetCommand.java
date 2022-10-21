package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subystems.Drivetrain;
import frc.robot.subystems.LimelightSubsystem;

public class LockTargetCommand extends CommandBase {

  private final Drivetrain drivetrain;
  private final LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;

  public LockTargetCommand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }
  
  @Override
  public void initialize() {
    limelight.setLedMode(Constants.LIMELIGHT_LIGHT_MODE.ON);
    // Pipeline ping nedeniyle geç gelebilir
    // Delay eklenebilir
  }

  @Override
  public void execute() {
    drivetrain.lockTarget(Constants.LIMELIGHT_LOCK_MODE.ALL);
  }

  @Override
  public void end(boolean interrupted) {
    limelight.setLedMode(Constants.LIMELIGHT_LIGHT_MODE.OFF);
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return drivetrain.isLimelightAtSetpoint();
  }
}
