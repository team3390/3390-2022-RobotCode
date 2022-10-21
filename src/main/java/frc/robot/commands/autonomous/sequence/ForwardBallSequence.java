package frc.robot.commands.autonomous.sequence;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.trajectory.ForwardTrajectory;
import frc.robot.commands.ball.IntakeCommand;
import frc.robot.subystems.BallSubsystem;
import frc.robot.subystems.Drivetrain;

public class ForwardBallSequence extends SequentialCommandGroup {

  public ForwardBallSequence(Drivetrain drivetrain, BallSubsystem ballSubsystem) {
    addCommands(
      new ParallelDeadlineGroup(
        new ForwardTrajectory(drivetrain),
        new IntakeCommand(ballSubsystem)
      )
    );
  }

}
