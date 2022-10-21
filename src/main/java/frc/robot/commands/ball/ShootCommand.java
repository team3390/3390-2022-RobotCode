package frc.robot.commands.ball;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.BallSubsystem;

public class ShootCommand extends CommandBase {

  private final BallSubsystem ballSubsystem;

  public ShootCommand(BallSubsystem ballSubsystem) {
    this.ballSubsystem = ballSubsystem;
    addRequirements(ballSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    ballSubsystem.stopAll();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
