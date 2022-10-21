package frc.robot.commands.ball;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.BallSubsystem;

public class FeedUpCommand extends CommandBase {

  private final BallSubsystem ballSubsystem;

  public FeedUpCommand(BallSubsystem ballSubsystem) {
    this.ballSubsystem = ballSubsystem;
    addRequirements(ballSubsystem);
  }

  @Override
  public void execute() {
    ballSubsystem.startFeedingToTop();
  }

  @Override
  public void end(boolean interrupted) {
    ballSubsystem.stopFeeding();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
