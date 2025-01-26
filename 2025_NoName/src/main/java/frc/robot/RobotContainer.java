package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.commands.CharacterizationCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.constants.RealConstants;

// import frc.robot.subsystems.vision.Vision;
// import frc.robot.subsystems.vision.VisionIOReal;
// import frc.robot.subsystems.vision.VisionIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  // private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  /* Path follower */
  private final AutoRoutines autoRoutines;
  private final AutoFactory autoFactory;
  public final AutoChooser autoChooser = new AutoChooser();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        // vision =
        //     new Vision(drive::addVisionMeasurement, new
        // VisionIOReal(RealConstants.camConstants));
        autoFactory =
            new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::setPose, // The drive subsystem
                drive::followChoreoPath, // The controller for the drive subsystem
                true,
                drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new GyroIO() {}, Drive.createSimModules());
        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOSim(RealConstants.camConstants, drive::getPose));
        autoFactory =
            new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::setPose, // The drive subsystem
                drive::followChoreoPath, // The controller for the drive subsystem
                true,
                drive);
        autoRoutines = new AutoRoutines(autoFactory);

        break;
      default:
        drive = new Drive(new GyroIOPigeon2(), Drive.createTalonFXModules());
        // vision =
        //     new Vision(drive::addVisionMeasurement, new
        // VisionIOReal(RealConstants.camConstants));
        autoFactory =
            new AutoFactory(
                drive::getPose, // A function that returns the current robot pose
                drive::setPose, // The drive subsystem
                drive::followChoreoPath, // The controller for the drive subsystem
                true,
                drive);
        autoRoutines = new AutoRoutines(autoFactory);
        break;
    }

    // Add options to the chooser
    autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAutoRoutine);

    // Put the auto chooser on the dashboard
    SmartDashboard.putData(autoChooser);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    DogLog.setOptions(
        new DogLogOptions().withCaptureDs(true).withCaptureNt(true).withNtPublish(true));
    // Example: To get the Pose3d for branch #5 at height L2
    // Schedule the selected auto during the autonomous period
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        drive.runVoltageTeleopFieldRelative(
            () ->
                new ChassisSpeeds(
                    -joystickDeadbandApply(controller.getLeftY())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    -joystickDeadbandApply(controller.getLeftX())
                        * RealConstants.MAX_LINEAR_SPEED
                        * 0.85,
                    joystickDeadbandApply(controller.getRightX())
                        * RealConstants.MAX_ANGULAR_SPEED)));
    controller.y().onTrue(drive.zeroGyrCommand());
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return CharacterizationCommands.feedforwardCharacterization(drive).withTimeout(7.5);
    // return autoChooser.selectedCommandScheduler();
  }

  private static double joystickDeadbandApply(double x) {
    // return MathUtil.applyDeadband(Math.abs(Math.pow(x, 3) * Math.signum(x)), 0.02);
    return MathUtil.applyDeadband(
        (Math.signum(x) * (1.01 * Math.pow(x, 2) - 0.0202 * x + 0.0101)), 0.02);
  }
}
