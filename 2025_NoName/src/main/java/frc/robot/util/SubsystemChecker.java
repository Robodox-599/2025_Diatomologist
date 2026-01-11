package frc.robot.util;

import static frc.robot.FieldConstants.REEF_BLUE_MIDDLE;
import static frc.robot.FieldConstants.REEF_RED_MIDDLE;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.subsystems.endefector.endefectorwrist.Wrist;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;

public class SubsystemChecker {
  private CommandSwerveDrivetrain drivetrain;
  private Elevator elevator;
  private Wrist wrist;
  private Rollers rollers;

  private final double maximumElevatorSwingThroughHeight =
      SubsystemUtil.elevatorStateToHeightInches(
          ElevatorConstants.ElevatorStates
              .POSITION_CORAL_STATION); // max height of elevator where endefector can safely swing
  // through (8.2)
  private final double minimumElevatorSwingAboveHeight =
      28.0; // min height of elevator where endefector can safely swing behind
  private final double minimumElevatorSwingBelowHeight =
      7.0; // min height of elevator where endefector can safely swing below
  private final double minimumElevatorSwingInfrontHeight =
      SubsystemUtil.elevatorStateToHeightInches(ElevatorStates.POSITION_CORAL_L1);
  private final double endefectorBehindElevatorPosition =
      SubsystemUtil.wristStateToSetpoint(
          WristConstants.WristStates
              .POSITION_PREPARED); // any wrist position less than this is behind the elevator
  private final double endefectorBeyondVerticalPosition =
      SubsystemUtil.wristStateToSetpoint(WristConstants.WristStates.POSITION_PREPARED);
  private final double endefectorOutsideBumpersPosition =
      -0.10 - WristConstants.wristPositionTolerance;

  public void addDrivetrain(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void addElevator(Elevator elevator) {
    this.elevator = elevator;
  }

  public void addWrist(Wrist wrist) {
    this.wrist = wrist;
  }

  public void addRollers(Rollers rollers) {
    this.rollers = rollers;
  }

  public double calculateElevatorSoftLowerLimit() {
    // if (wrist.getPosition() > endefectorBeyondHorizontalPosition
    //     && elevator.getHeightInches() > minimumElevatorSwingBelowHeight) {
    //   return minimumElevatorSwingBelowHeight;
    // return minimumElevatorSwingBelowHeight * (Math.sin(-2 * wrist.getPosition())); <- this
    // would make the limit dynamic based on wrist position
    // } else
    double elevatorSoftLowerLimit;
    // if (wrist.getPosition()
    //         < endefectorBehindElevatorPosition - WristConstants.wristPositionTolerance
    //     && elevator.getHeightInches() > minimumElevatorSwingAboveHeight) {
    //   elevatorSoftLowerLimit = minimumElevatorSwingAboveHeight;
    // } else
    if (wrist.getPosition()
        > endefectorBeyondVerticalPosition + WristConstants.wristPositionTolerance) {
      elevatorSoftLowerLimit = minimumElevatorSwingInfrontHeight;
    } else {
      elevatorSoftLowerLimit = ElevatorConstants.elevatorHardLowerLimit;
    }
    DogLog.log("SubsystemChecker/ElevatorSoftLowerLimit", elevatorSoftLowerLimit);
    return elevatorSoftLowerLimit;
  }

  public double calculateElevatorSoftUpperLimit() {
    double elevatorSoftUpperLimit;
    if (wrist.getPosition()
            < endefectorBehindElevatorPosition - WristConstants.wristPositionTolerance
        && elevator.getHeightInches()
            < maximumElevatorSwingThroughHeight + ElevatorConstants.positionToleranceInches) {
      elevatorSoftUpperLimit = maximumElevatorSwingThroughHeight;
    } else {
      elevatorSoftUpperLimit = ElevatorConstants.elevatorHardUpperLimit;
    }
    DogLog.log("SubsystemChecker/ElevatorSoftUpperLimit", elevatorSoftUpperLimit);
    return elevatorSoftUpperLimit;
  }

  public boolean isEndefectorUnderElevator() {
    if ((elevator.getHeightInches()
        < maximumElevatorSwingThroughHeight + ElevatorConstants.positionToleranceInches)) {
      DogLog.log("SubsystemChecker/isEndefectorUnderElevator", true);
      return true;
    }
    DogLog.log("SubsystemChecker/isEndefectorUnderElevator", false);
    return false;
  }

  public boolean isEndefectorBeyondBumpers() {
    if (wrist.getPosition() > endefectorOutsideBumpersPosition) {
      DogLog.log("SubsystemChecker/isEndefectorBeyondBumpers", true);
      return true;
    }
    DogLog.log("SubsystemChecker/isEndefectorBeyondBumpers", false);
    return false;
  }

  public boolean isSafeDistanceFromReef(boolean isTrough) {
    int nearestFaceIndex = AutoAlignPoseGenerator.getNearestReefFaceIndex();
    Pose2d transformToReef;
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      transformToReef = REEF_BLUE_MIDDLE[nearestFaceIndex].relativeTo(drivetrain.getPose());
    } else {
      transformToReef = REEF_RED_MIDDLE[nearestFaceIndex].relativeTo(drivetrain.getPose());
    }
    double xDistance =
        Math.abs(transformToReef.getX())
            + CommandSwerveDrivetrain.DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE
            + 0.02; // +2 cm for extra tolerance

    DogLog.log("SubsystemChecker/DistanceFromReefForTrough", xDistance);
    if (isTrough) {
      DogLog.log(
          "SubsystemChecker/isSafeDistanceFromReef",
          xDistance >= Math.abs(AutoAlignPoseGenerator.L1_REEF_FACE_OFFSET));
      return xDistance >= Math.abs(AutoAlignPoseGenerator.L1_REEF_FACE_OFFSET);
    } else {
      DogLog.log(
          "SubsystemChecker/isSafeDistanceFromReef",
          xDistance >= Math.abs(AutoAlignPoseGenerator.L2_L3_REEF_FACE_OFFSET));
      return xDistance >= Math.abs(AutoAlignPoseGenerator.L2_L3_REEF_FACE_OFFSET);
    }
  }

  public boolean isAtElevatorHeight(ElevatorStates state) {
    boolean isAtElevatorHeight = elevator.isAtSetpoint(state);
    DogLog.log("SubsystemChecker/isAtElevatorHeight", isAtElevatorHeight);
    return isAtElevatorHeight;
  }

  // public boolean isAtElevatorSetpoint(ElevatorStates state) {
  //   return isAtElevatorHeight(SubsystemUtil.elevatorStateToHeightInches(state));
  // }

  // public boolean isAtElevatorHeight(double height) {
  //   return Math.abs(elevatorHeight - height) < ElevatorConstants.positionToleranceInches;
  // }

  // public boolean isAtPositionWrist(WristConstants.WristStates state) {
  //   boolean isAtPositionWrist = wrist.isAtSetpoint(state);
  //   DogLog.log("SubsystemChecker/isAtPositionWrist", isAtPositionWrist);
  //   return isAtPositionWrist;
  // }

  public boolean isAtWristPosition(WristStates state) {
    boolean isAtWristPosition = wrist.isAtSetpoint(state);
    DogLog.log("SubsystemChecker/isAtWristSetpoint", isAtWristPosition);
    return isAtWristPosition;
  }

  // public boolean isAtWristAngle(double angle) {
  //   return Math.abs(wristPosition - angle) < WristConstants.wristPositionTolerance;
  // }

  public boolean isCoralInEndefector() {
    boolean isCoralIntakedInEndefector = rollers.isCoralIntakedInEndefector();
    DogLog.log("SubsystemChecker/isCoralInEndefector", isCoralIntakedInEndefector);
    return isCoralIntakedInEndefector;
  }

  // public void setWristPosition(double position) {
  //   wristPosition = position;
  // }

  // public void setElevatorHeight(double height) {
  //   elevatorHeight = height;
  // }

  // public void setCoralInEndefector(boolean hasCoral) {
  //   isCoralIntakedInEndefector = hasCoral;
  // }

  // public void setRobotPose(Pose2d pose) {
  //   robotPose = pose;
  // }

  // public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
  //   this.speeds = chassisSpeeds;
  // }

  public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getChassisSpeeds();
  }

  public boolean isSpeedsSettled() {
    ChassisSpeeds speeds = getChassisSpeeds();
    boolean isSpeedsSettled =
        Math.abs(speeds.vxMetersPerSecond) < 0.01
            && Math.abs(speeds.vyMetersPerSecond) < 0.01
            && Math.abs(speeds.omegaRadiansPerSecond) < 0.01;
    DogLog.log("isSpeedsSettled", isSpeedsSettled);
    return isSpeedsSettled;
  }

  // public boolean isSafeElevator() {
  //   if (!isBehindElevator(wrist.getPosition())
  //       || isUnderElevator(
  //           elevator.getHeightInches())) { // if the wrist is NOT behind the elevator or if the
  //     // endefector is
  //     // BELOW
  //     // the elevator swing height, elevator is safe
  //     DogLog.log("SafetyChecker/isSafeElevator", true);
  //     return true;
  //   }
  //   DogLog.log("SafetyChecker/isSafeElevator", false);
  //   return false;
  // }

  // public boolean isEndefectorBehindElevator() {
  //   DogLog.log("SafetyChecker/isEndefectorBehindElevator", isBehindElevator(wristDegrees));
  //   return isBehindElevator(wristDegrees);
  // }

  // public boolean isBehindElevator(double wristSupplyDegrees) {
  //   return (endefectorBehindElevatorDegrees > wristSupplyDegrees);
  // }

  // public boolean isAboveElevator(double elevatorSupplyInches) {
  //   return (elevatorSupplyInches > minimumElevatorSwingAboveHeight);
  // }

  // public boolean isReadyToScore() {
  //   DogLog.log("SafetyChecker/readyToScore", isAtSetpointElevator && isAtSetpointWrist);
  //   return isAtSetpointElevator && isAtSetpointWrist;
  // }

  // public boolean isAtSetpointElevator() {
  //   return isAtSetpointElevator;
  // }

  // public boolean isWristAtPrepared() {
  //   return Math.abs(wristDegrees - 0.79) < WristConstants.wristPositionTolerance;
  // }
}
