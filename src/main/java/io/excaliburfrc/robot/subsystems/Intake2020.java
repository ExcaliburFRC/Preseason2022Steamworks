package io.excaliburfrc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.excaliburfrc.robot.Constants.InTake2020Constants;

import static io.excaliburfrc.robot.Constants.CollectorConstants.FORWARD_CHANNEL;
import static io.excaliburfrc.robot.Constants.CollectorConstants.REVERSE_CHANNEL;

public class Intake2020 extends SubsystemBase {
  private final DoubleSolenoid piston;
  private final CANSparkMax motor;

  public Intake2020() {
    piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, FORWARD_CHANNEL, REVERSE_CHANNEL);
    motor = new CANSparkMax(InTake2020Constants.IntakeMotorID, MotorType.kBrushless);
  }

  public void raise() {
    piston.set(DoubleSolenoid.Value.kReverse);
  }

  public void lower() {
    piston.set(DoubleSolenoid.Value.kForward);
  }

  public DoubleSolenoid.Value getPiston() {
    return piston.get();
  }

  public void turnOnMotor() {
    motor.set(InTake2020Constants.MotorSpeed);
  }

  public void turnOffMotor() {
    motor.stopMotor();
  }

  public double getSpeed() {
    return motor.get();
  }

  public Command collectCommand() {
    return new InstantCommand(
        () -> {
          lower();
          turnOnMotor();
        }, this);
  }

  public Command closeCommand() {
    return new InstantCommand(
        () -> {
          turnOffMotor();
          raise();
        },
        this);
  }
}
