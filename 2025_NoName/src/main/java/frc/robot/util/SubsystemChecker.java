package frc.robot.util;

import static frc.robot.FieldConstants.REEF_BLUE_MIDDLE;
import static frc.robot.FieldConstants.REEF_RED_MIDDLE;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants;
import frc.robot.subsystems.endefector.endefectorwrist.WristConstants.WristStates;

public class SubsystemChecker {
  private double wristPosition;
  private double elevatorHeight;
  private boolean isCoralIntakedInEndefector;
  private Pose2d robotPose;
  private ChassisSpeeds speeds = new ChassisSpeeds();

  private final double maximumElevatorSwingThroughHeight =
      SubsystemUtil.elevatorStateToHeightInches(
              ElevatorConstants.ElevatorStates.INTAKING_CORAL_STATION)
          + ElevatorConstants
              .positionToleranceInches; // max height of elevator where endefector can safely swing
  // through (8.2)
  private final double minimumElevatorSwingAboveHeight =
      28.0; // min height of elevator where endefector can safely swing behind
  private final double minimumElevatorSwingBelowHeight =
      7.0; // min height of elevator where endefector can safely swing below
  private final double endefectorBehindElevatorPosition =
      SubsystemUtil.wristStateToSetpoint(WristConstants.WristStates.POSITION_PREPARED)
          - WristConstants
              .wristPositionTolerance; // any wrist position less than this is behind the elevator
  private final double endefectorBeyondHorizontalPosition = 0.05;
  private final double endefectorOutsideBumpersPosition =
      -0.10 - WristConstants.wristPositionTolerance;

  public double calculateElevatorSoftLowerLimit() {
    // if (wrist.getPosition() > endefectorBeyondHorizontalPosition
    //     && elevator.getHeightInches() > minimumElevatorSwingBelowHeight) {
    //   return minimumElevatorSwingBelowHeight;
    // return minimumElevatorSwingBelowHeight * (Math.sin(-2 * wrist.getPosition())); <- this
    // would make the limit dynamic based on wrist position
    // } else
    if (wristPosition < endefectorBehindElevatorPosition
        && elevatorHeight > minimumElevatorSwingAboveHeight) {
      return minimumElevatorSwingAboveHeight;
    } else {
      return ElevatorConstants.elevatorHardLowerLimit;
    }
  }

  public double calculateElevatorSoftUpperLimit() {
    if (wristPosition < endefectorBehindElevatorPosition
        && elevatorHeight < maximumElevatorSwingThroughHeight) {
      return maximumElevatorSwingThroughHeight;
    } else {
      return ElevatorConstants.elevatorHardUpperLimit;
    }
  }

  public boolean isEndefectorUnderElevator() {
    if ((elevatorHeight < maximumElevatorSwingThroughHeight)) {
      DogLog.log("SubsystemChecker/isEndefectorUnderElevator", true);
      return true;
    }
    DogLog.log("SubsystemChecker/isEndefectorUnderElevator", false);
    return false;
  }

  public boolean isEndefectorBeyondBumpers() {
    if (wristPosition > endefectorOutsideBumpersPosition) {
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
      transformToReef = REEF_BLUE_MIDDLE[nearestFaceIndex].relativeTo(robotPose);
    } else {
      transformToReef = REEF_RED_MIDDLE[nearestFaceIndex].relativeTo(robotPose);
    }
    double xDistance =
        Math.abs(transformToReef.getX())
            + CommandSwerveDrivetrain
                .DRIVE_TO_POINT_TRANSLATION_ERROR_TOLERANCE; // +2 cm for extra tolerance

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

  public boolean isAtHeightElevator(ElevatorConstants.ElevatorStates state) {
    boolean isAtHeightElevator = isAtElevatorSetpoint(state);
    DogLog.log("SubsystemChecker/isAtHeightElevator", isAtHeightElevator);
    return isAtHeightElevator;
  }

  public boolean isAtElevatorSetpoint(ElevatorStates state) {
    return isAtElevatorHeight(SubsystemUtil.elevatorStateToHeightInches(state));
  }

  public boolean isAtElevatorHeight(double height) {
    return Math.abs(elevatorHeight - height) < ElevatorConstants.positionToleranceInches;
  }

  public boolean isAtPositionWrist(WristConstants.WristStates state) {
    boolean isAtPositionWrist = isAtWristSetpoint(state);
    DogLog.log("SubsystemChecker/isAtPositionWrist", isAtPositionWrist);
    return isAtPositionWrist;
  }

  public boolean isAtWristSetpoint(WristStates state) {
    return isAtWristAngle(WristConstants.setpoints[state.getIndex()]);
  }

  public boolean isAtWristAngle(double angle) {
    return Math.abs(wristPosition - angle) < WristConstants.wristPositionTolerance;
  }

  public boolean isCoralInEndefector() {
    DogLog.log("SubsystemChecker/isCoralInEndefector", isCoralIntakedInEndefector);
    return isCoralIntakedInEndefector;
  }

  public void setWristPosition(double position) {
    wristPosition = position;
  }

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }

  public void setCoralInEndefector(boolean hasCoral) {
    isCoralIntakedInEndefector = hasCoral;
  }

  public void setRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    this.speeds = chassisSpeeds;
  }

  public ChassisSpeeds getChassisSpeeds() {
    return this.speeds;
  }

  public boolean isSpeedsSettled() {
    return Math.abs(this.speeds.vyMetersPerSecond) < 0.1
        && Math.abs(this.speeds.omegaRadiansPerSecond) < 0.1;
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
