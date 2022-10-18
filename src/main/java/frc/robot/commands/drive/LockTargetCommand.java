package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subystems.Drivetrain;

public class LockTargetCommand extends CommandBase {

  private final Drivetrain drivetrain;

  public LockTargetCommand(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.lockTarget(Constants.LIMELIGHT_LOCK_MODE.ALL);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  @Override
  public boolean isFinished() {
    return drivetrain.isLimelightAtSetpoint();
  }
}
