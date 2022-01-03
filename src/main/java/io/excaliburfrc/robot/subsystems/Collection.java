package io.excaliburfrc.robot.subsystems;

import static io.excaliburfrc.robot.Constants.CollectorConstants.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collection extends SubsystemBase {
  private final DoubleSolenoid piston;

  public Collection() {
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL, REVERSE_CHANNEL);
  }

  public void raise() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void lower() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public Command raiseCommand() {
    return new InstantCommand(this::raise, this);
  }

  public Command lowerCommand() {
    return new InstantCommand(this::lower, this);
  }

  public void activatePiston() {
    piston.toggle();
  }

  public DoubleSolenoid.Value getPiston() {
    return piston.get();
  }
}
