package io.excaliburfrc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems, as `public final`

  // auto selector
  private final SendableChooser<Command> chooser = new SendableChooser<>();

  // joysticks
  private final PS4Controller driveJoystick = new PS4Controller(0);
  private final Joystick armJoystick = new Joystick(1);
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chooser.setDefaultOption("Nothing", new PrintCommand("No Auto Command was selected"));
    // Configure the button bindings
    configureButtonBindings();
    initSubsystemStates();
  }

  private void configureButtonBindings() {
    // create `JoystickButton`s binding between the buttons and commands.
    // use the two joysticks that are already declared: `driveJoystick` and `armJoystick`3
    // DO NOT CREATE MORE JOYSTICKS! or rename them.
    // first declare button/axis indexes as `final int`s here.

    // driverJoystick

    // armJoystick

  }

  public void initSubsystemStates() {}

  public Command getAuto() {
    return chooser.getSelected();
  }
}
