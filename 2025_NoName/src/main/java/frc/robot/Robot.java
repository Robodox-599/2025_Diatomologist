// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.autos.AutoRoutines;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain.WantedState;
import frc.robot.subsystems.drive.constants.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOSim;
import frc.robot.subsystems.endefector.endefectorrollers.RollersIOTalonFX;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOSim;
import frc.robot.subsystems.endefector.endefectorwrist.WristIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDsIOReal;
import frc.robot.subsystems.leds.LEDsIOSim;
import frc.robot.subsystems.subsystemVisualizer.SubsystemVisualizer;
import frc.robot.subsystems.vision4.Vision4;
import frc.robot.subsystems.vision4.camera.Camera;
import frc.robot.subsystems.vision4.camera.CameraIOReal;
import frc.robot.subsystems.vision4.camera.CameraTransforms;
import frc.robot.util.SubsystemChecker;
import frc.robot.util.Tracer;

public class Robot extends TimedRobot {
  private final CommandScheduler scheduler = CommandScheduler.getInstance();

  final CommandXboxController driver =
      new CommandXboxController(Constants.ControllerConstants.kDriverControllerPort);
  final CommandXboxController operator =
      new CommandXboxController(Constants.ControllerConstants.kOperatorControllerPort);
  final SubsystemChecker subsystemChecker = new SubsystemChecker();
  final AutoChooser autoChooser = new AutoChooser();
  final Superstructure superstructure;
  final CommandSwerveDrivetrain drivetrain;
  final Elevator elevator;
  final Wrist wrist;
  final Rollers rollers;
  final Climb climb;
  final LEDs leds;
  final Vision4 vision;
  final AutoFactory autoFactory;
  final AutoRoutines autoRoutines;
  final SubsystemVisualizer subsystemVisualizer;

  @Override
  protected void loopFunc() {
    Tracer.startTrace("RobotLoop");
    super.loopFunc();
    Tracer.endTrace();
  }

  public Robot() {
    Tracer.enableSingleThreadedMode();
    Tracer.enableTracingForCurrentThread();

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withCaptureNt(true)
            .withNtPublish(true)
            .withCaptureConsole(true));

    switch (Constants.currentMode) {
      case REAL:
        elevator = new Elevator(new ElevatorIOTalonFX(), subsystemChecker);
        rollers = new Rollers(new RollersIOTalonFX(), subsystemChecker);
        wrist = new Wrist(new WristIOTalonFX(), subsystemChecker);
        drivetrain = TunerConstants.createDrivetrain(driver, subsystemChecker);
        leds = new LEDs(new LEDsIOReal());
        climb = new Climb(new ClimbIOTalonFX());
        vision =
            new Vision4(
                new Camera(
                    new CameraIOReal(CameraTransforms.frontLeftCameraConstants),
                    drivetrain::addVisionMeasurement,
                    subsystemChecker),
                new Camera(
                    new CameraIOReal(CameraTransforms.frontRightCameraConstants),
                    drivetrain::addVisionMeasurement,
                    subsystemChecker));
        break;
      default: // SIMULATION
        DriverStation.silenceJoystickConnectionWarning(true);
        elevator = new Elevator(new ElevatorIOSim(), subsystemChecker);
        rollers = new Rollers(new RollersIOSim(), subsystemChecker);
        wrist = new Wrist(new WristIOSim(), subsystemChecker);
        drivetrain = TunerConstants.createDrivetrain(driver, subsystemChecker);
        leds = new LEDs(new LEDsIOSim());
        climb = new Climb(new ClimbIOSim());
        vision =
            new Vision4(
                new Camera(
                    new CameraIOReal(CameraTransforms.frontLeftCameraConstants),
                    drivetrain::addVisionMeasurement,
                    subsystemChecker),
                new Camera(
                    new CameraIOReal(CameraTransforms.frontRightCameraConstants),
                    drivetrain::addVisionMeasurement,
                    subsystemChecker));
        break;
    }
    autoFactory =
        new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::setDesiredChoreoTrajectory,
            true,
            drivetrain);

    superstructure =
        new Superstructure(
            drivetrain,
            elevator,
            wrist,
            rollers,
            climb,
            leds,
            vision,
            subsystemChecker,
            driver,
            operator);

    subsystemChecker.addDrivetrain(drivetrain);
    subsystemChecker.addElevator(elevator);
    subsystemChecker.addWrist(wrist);
    subsystemChecker.addRollers(rollers);

    new Bindings(driver, operator, superstructure);

    subsystemVisualizer = new SubsystemVisualizer(elevator, wrist, rollers);

    autoRoutines = new AutoRoutines(autoFactory, superstructure, drivetrain);

    // Auto chooser setup
    RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

    /** AUTO ROUTINES */
    // COMPETITION
    autoChooser.addRoutine("Left Auto - 4 Coral", autoRoutines::leftAutoRoutine);
    autoChooser.addRoutine("Right Auto - 4 Coral", autoRoutines::rightAutoRoutine);
    // autoChooser.addRoutine(
    //     "Middle Auto & Algae - 1 Coral + Grab Algae",
    // autoRoutines::middleAutoAndGrabAlgaeRoutine);
    autoChooser.addRoutine("Middle Auto - 1 Coral", autoRoutines::middleAutoRoutine);
    autoChooser.addRoutine("Taxi Auto - Taxi", autoRoutines::taxiAutoRoutine);

    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    Tracer.traceFunc("CommandScheduler", scheduler::run);
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    scheduler.cancelAll();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    scheduler.cancelAll();
    drivetrain.setWantedState(WantedState.TELEOP_DRIVE);
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    scheduler.cancelAll();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {
    scheduler.cancelAll();
  }
}
