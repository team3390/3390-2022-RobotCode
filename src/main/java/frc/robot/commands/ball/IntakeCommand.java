package frc.robot.commands.ball;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.BallSubsystem;

public class IntakeCommand extends CommandBase {
  
  private BallSubsystem ballSubsystem;

  public IntakeCommand(BallSubsystem ballSubsystem) {
    this.ballSubsystem = ballSubsystem;

    addRequirements(ballSubsystem);
  }

  @Override
  public void initialize() {
    ballSubsystem.intakeSolenoid.set(true);
  }

  @Override
  public void execute() {
    ballSubsystem.intakeMotor.set(1);
  }

  @Override
  public void end(boolean interrupted) {
    ballSubsystem.intakeSolenoid.set(false);
    ballSubsystem.intakeMotor.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
