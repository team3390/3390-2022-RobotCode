package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subystems.Drivetrain;

public class TankDriveCommand extends CommandBase {

  private final Drivetrain drivetrain;

  private final Joystick left;
  private final Joystick right;

  public TankDriveCommand(Drivetrain drivetrain, Joystick left, Joystick right) {
    this.drivetrain = drivetrain;
    this.left = left;
    this.right = right;
    addRequirements(this.drivetrain);
  }

  @Override
  public void initialize() {
    System.out.println("Tink Driv");
  }

  @Override
  public void execute() {
    drivetrain.tankDrivePercent(left.getY(), right.getY());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
