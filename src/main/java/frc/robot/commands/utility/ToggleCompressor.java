package frc.robot.commands.utility;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleCompressor extends CommandBase {

  private final Compressor comp = new Compressor(PneumaticsModuleType.CTREPCM);
  boolean isActive;

  @Override
  public void initialize() {
    isActive = comp.enabled();
  }

  @Override
  public void execute() {
    if (isActive) {
      comp.disable();
    } else {
      comp.enableDigital();
    }
  }

  @Override
  public boolean isFinished() {
    return isActive != comp.enabled();
  }
}
