package frc.robot.commands.utility;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.Drivetrain;

public class ToggleVitesse extends CommandBase {

  private final Drivetrain drivetrain;
  boolean isActive;

  public ToggleVitesse(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    isActive = drivetrain.vitesse.get();
  }

  @Override
  public void execute() {
    drivetrain.vitesse.set(!isActive);
  }

  @Override
  public boolean isFinished() {
    return isActive != drivetrain.vitesse.get();
  }
}
