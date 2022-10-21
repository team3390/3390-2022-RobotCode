package frc.robot.commands.ball;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.BallSubsystem;

public class FeedDownCommand extends CommandBase {

  private final BallSubsystem ballSubsystem;

  public FeedDownCommand(BallSubsystem ballSubsystem) {
    this.ballSubsystem = ballSubsystem;
    addRequirements(ballSubsystem);
  }

  @Override
  public void execute() {
    ballSubsystem.startFeedingToDown();
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