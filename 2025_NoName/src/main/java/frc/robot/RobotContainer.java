package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.subsystems.endefector.rollers.Rollers;
import frc.robot.subsystems.endefector.rollers.RollersIOSim;
import frc.robot.subsystems.endefector.rollers.RollersIOTalonFX;

public class RobotContainer {

  // Endefector
  private Rollers rollers;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  /* Path follower */
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        rollers = new Rollers(new RollersIOTalonFX());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        rollers = new Rollers(new RollersIOSim());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.camConstants, drive::getPose));

        break;
      default:
        break;
    }

    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));

    // Configure the button bindings
    configureButtonBindings();
    // log all reef positions, useful for debugging.
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //
  private void configureButtonBindings() {
    controller.a().whileTrue(rollers.runVoltage(-1));
    controller.b().whileTrue(rollers.runVoltage(1));
    controller.x().whileTrue(rollers.runVoltage(5));
    controller.y().whileTrue(rollers.runVoltage(-5));

    controller.a().onFalse(rollers.runVoltage(0));
    controller.b().onFalse(rollers.runVoltage(0));

    controller.x().onFalse(rollers.runVoltage(0));

    controller.y().onFalse(rollers.runVoltage(0));

    // Default command, normal field-relative drive
    // // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(drive.stopWithXCmd());

    // // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
