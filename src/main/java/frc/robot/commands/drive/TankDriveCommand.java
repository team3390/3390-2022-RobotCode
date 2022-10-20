package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.Drivetrain;

public class TankDriveCommand extends CommandBase {

  private final Drivetrain drivetrain;

  private final double left;
  private final double right;

  public TankDriveCommand(Drivetrain drivetrain, double left, double right) {
    this.drivetrain = drivetrain;
    this.left = left;
    this.right = right;
    addRequirements(this.drivetrain);
  }

  @Override
  public void execute() {
    drivetrain.tankDrivePercent((left / 3)* 2, (right / 3) * 2);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
