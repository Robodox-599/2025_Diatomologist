// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.util.GeomUtil.averagePose;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.constants.TunerConstants;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldConstants {
  public static final double fieldLength = Units.inchesToMeters(690.875);
  public static final double fieldWidth = Units.inchesToMeters(317);
  public static final double startingLineX =
      Units.inchesToMeters(299.438); // Measured from the inside of starting line

  private static final Transform2d BRANCH_TRANSFORM =
      new Transform2d(0.33 + (TunerConstants.BUMPER_TO_BUMPER_LENGTH / 2), 0.01, new Rotation2d(0));

  public static Pose2d REEF_BLUE_A =
      new Pose2d(3.95, 4.20, Rotation2d.fromDegrees(180))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_B =
      new Pose2d(3.95, 3.87, Rotation2d.fromDegrees(180))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_C =
      new Pose2d(4.07, 3.66, Rotation2d.fromDegrees(240))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_D =
      new Pose2d(4.35, 3.49, Rotation2d.fromDegrees(240))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_E =
      new Pose2d(4.60 + 0.01, 3.50 - 0.02, Rotation2d.fromDegrees(300))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_F =
      new Pose2d(4.88 + 0.01, 3.66 - 0.02, Rotation2d.fromDegrees(300))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_G =
      new Pose2d(5.00 + 0.02, 3.86, Rotation2d.fromDegrees(0))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_H =
      new Pose2d(5.00 + 0.02, 4.18, Rotation2d.fromDegrees(0))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_I =
      new Pose2d(4.88 + 0.025, 4.41 - 0.01, Rotation2d.fromDegrees(60))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_J =
      new Pose2d(4.60 + 0.025, 4.57 - 0.01, Rotation2d.fromDegrees(60))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_K =
      new Pose2d(4.36 + 0.015, 4.57 + 0.005, Rotation2d.fromDegrees(120))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));
  public static Pose2d REEF_BLUE_L =
      new Pose2d(4.06 + 0.03, 4.41, Rotation2d.fromDegrees(120))
          .transformBy(BRANCH_TRANSFORM)
          .plus(new Transform2d(0, 0, new Rotation2d(Math.PI)));

  public static final Pose2d REEF_BLUE_MIDDLE_AB = averagePose(REEF_BLUE_A, REEF_BLUE_B);
  public static final Pose2d REEF_BLUE_MIDDLE_CD = averagePose(REEF_BLUE_C, REEF_BLUE_D);
  public static final Pose2d REEF_BLUE_MIDDLE_EF = averagePose(REEF_BLUE_E, REEF_BLUE_F);
  public static final Pose2d REEF_BLUE_MIDDLE_GH = averagePose(REEF_BLUE_G, REEF_BLUE_H);
  public static final Pose2d REEF_BLUE_MIDDLE_IJ = averagePose(REEF_BLUE_I, REEF_BLUE_J);
  public static final Pose2d REEF_BLUE_MIDDLE_KL = averagePose(REEF_BLUE_K, REEF_BLUE_L);

  public static final Pose2d REEF_RED_A = ChoreoAllianceFlipUtil.flip(REEF_BLUE_A);
  public static final Pose2d REEF_RED_B = ChoreoAllianceFlipUtil.flip(REEF_BLUE_B);
  public static final Pose2d REEF_RED_C = ChoreoAllianceFlipUtil.flip(REEF_BLUE_C);
  public static final Pose2d REEF_RED_D = ChoreoAllianceFlipUtil.flip(REEF_BLUE_D);
  public static final Pose2d REEF_RED_E = ChoreoAllianceFlipUtil.flip(REEF_BLUE_E);
  public static final Pose2d REEF_RED_F = ChoreoAllianceFlipUtil.flip(REEF_BLUE_F);
  public static final Pose2d REEF_RED_G = ChoreoAllianceFlipUtil.flip(REEF_BLUE_G);
  public static final Pose2d REEF_RED_H = ChoreoAllianceFlipUtil.flip(REEF_BLUE_H);
  public static final Pose2d REEF_RED_I = ChoreoAllianceFlipUtil.flip(REEF_BLUE_I);
  public static final Pose2d REEF_RED_J = ChoreoAllianceFlipUtil.flip(REEF_BLUE_J);
  public static final Pose2d REEF_RED_K = ChoreoAllianceFlipUtil.flip(REEF_BLUE_K);
  public static final Pose2d REEF_RED_L = ChoreoAllianceFlipUtil.flip(REEF_BLUE_L);

  public static final Pose2d REEF_RED_MIDDLE_AB = averagePose(REEF_RED_A, REEF_RED_B);
  public static final Pose2d REEF_RED_MIDDLE_CD = averagePose(REEF_RED_C, REEF_RED_D);
  public static final Pose2d REEF_RED_MIDDLE_EF = averagePose(REEF_RED_E, REEF_RED_F);
  public static final Pose2d REEF_RED_MIDDLE_GH = averagePose(REEF_RED_G, REEF_RED_H);
  public static final Pose2d REEF_RED_MIDDLE_IJ = averagePose(REEF_RED_I, REEF_RED_J);
  public static final Pose2d REEF_RED_MIDDLE_KL = averagePose(REEF_RED_K, REEF_RED_L);

  public static final Pose2d[] REEF_BLUE_LEFT = {
    FieldConstants.REEF_BLUE_A,
    FieldConstants.REEF_BLUE_C,
    FieldConstants.REEF_BLUE_E,
    FieldConstants.REEF_BLUE_G,
    FieldConstants.REEF_BLUE_I,
    FieldConstants.REEF_BLUE_K
  };

  public static final Pose2d[] REEF_BLUE_RIGHT = {
    FieldConstants.REEF_BLUE_B,
    FieldConstants.REEF_BLUE_D,
    FieldConstants.REEF_BLUE_F,
    FieldConstants.REEF_BLUE_H,
    FieldConstants.REEF_BLUE_J,
    FieldConstants.REEF_BLUE_L
  };

  public static final Pose2d[] REEF_BLUE_MIDDLE = {
    FieldConstants.REEF_BLUE_MIDDLE_AB,
    FieldConstants.REEF_BLUE_MIDDLE_CD,
    FieldConstants.REEF_BLUE_MIDDLE_EF,
    FieldConstants.REEF_BLUE_MIDDLE_GH,
    FieldConstants.REEF_BLUE_MIDDLE_IJ,
    FieldConstants.REEF_BLUE_MIDDLE_KL
  };

  public static final Pose2d[] REEF_RED_LEFT = {
    FieldConstants.REEF_RED_A,
    FieldConstants.REEF_RED_C,
    FieldConstants.REEF_RED_E,
    FieldConstants.REEF_RED_G,
    FieldConstants.REEF_RED_I,
    FieldConstants.REEF_RED_K
  };

  public static final Pose2d[] REEF_RED_RIGHT = {
    FieldConstants.REEF_RED_B,
    FieldConstants.REEF_RED_D,
    FieldConstants.REEF_RED_F,
    FieldConstants.REEF_RED_H,
    FieldConstants.REEF_RED_J,
    FieldConstants.REEF_RED_L
  };

  public static final Pose2d[] REEF_RED_MIDDLE = {
    FieldConstants.REEF_RED_MIDDLE_AB,
    FieldConstants.REEF_RED_MIDDLE_CD,
    FieldConstants.REEF_RED_MIDDLE_EF,
    FieldConstants.REEF_RED_MIDDLE_GH,
    FieldConstants.REEF_RED_MIDDLE_IJ,
    FieldConstants.REEF_RED_MIDDLE_KL
  };

  public static class Processor {
    public static final Pose2d centerFace =
        new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
  }

  public static class Barge {
    public static final Translation2d farCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(286.779));
    public static final Translation2d middleCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(242.855));
    public static final Translation2d closeCage =
        new Translation2d(Units.inchesToMeters(345.428), Units.inchesToMeters(199.947));

    // Measured from floor to bottom of cage
    public static final double deepHeight = Units.inchesToMeters(3.125);
    public static final double shallowHeight = Units.inchesToMeters(30.125);
  }

  public static class CoralStation {
    public static final Pose2d leftCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(291.176),
            Rotation2d.fromDegrees(90 - 144.011));
    public static final Pose2d rightCenterFace =
        new Pose2d(
            Units.inchesToMeters(33.526),
            Units.inchesToMeters(25.824),
            Rotation2d.fromDegrees(144.011 - 90));
  }

  public static class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
    public static final double faceToZoneLine =
        Units.inchesToMeters(12); // Side of the reef to the inside of the reef zone line

    public static final Pose2d[] centerFaces =
        new Pose2d[6]; // Starting facing the driver station in clockwise order
    public static final List<Map<ReefHeight, Pose3d>> branchPositions;

    static {
      // Initialize faces
      centerFaces[0] =
          new Pose2d(
              Units.inchesToMeters(144.003),
              Units.inchesToMeters(158.500),
              Rotation2d.fromDegrees(180));
      centerFaces[1] =
          new Pose2d(
              Units.inchesToMeters(160.373),
              Units.inchesToMeters(186.857),
              Rotation2d.fromDegrees(120));
      centerFaces[2] =
          new Pose2d(
              Units.inchesToMeters(193.116),
              Units.inchesToMeters(186.858),
              Rotation2d.fromDegrees(60));
      centerFaces[3] =
          new Pose2d(
              Units.inchesToMeters(209.489),
              Units.inchesToMeters(158.502),
              Rotation2d.fromDegrees(0));
      centerFaces[4] =
          new Pose2d(
              Units.inchesToMeters(193.118),
              Units.inchesToMeters(130.145),
              Rotation2d.fromDegrees(-60));
      centerFaces[5] =
          new Pose2d(
              Units.inchesToMeters(160.375),
              Units.inchesToMeters(130.144),
              Rotation2d.fromDegrees(-120));

      // Initialize branch positions
      branchPositions = new ArrayList<>(Collections.nCopies(12, null));

      for (int face = 0; face < 6; face++) {
        Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
        Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();

        for (var level : ReefHeight.values()) {
          Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
          double adjustX = Units.inchesToMeters(30.738);
          double adjustY = Units.inchesToMeters(6.469);

          fillRight.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));

          fillLeft.put(
              level,
              new Pose3d(
                  new Translation3d(
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getX(),
                      poseDirection
                          .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                          .getY(),
                      level.height),
                  new Rotation3d(
                      0,
                      Units.degreesToRadians(level.pitch),
                      poseDirection.getRotation().getRadians())));
        }

        // Now we use `set()` instead of `add()` since indices exist
        branchPositions.set(face * 2, fillRight);
        branchPositions.set((face * 2) + 1, fillLeft);
      }
    }
  }

  public static class StagingPositions {
    // Measured from the center of the ice cream
    public static final Pose2d leftIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(230.5), new Rotation2d());
    public static final Pose2d middleIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(158.5), new Rotation2d());
    public static final Pose2d rightIceCream =
        new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(86.5), new Rotation2d());
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // in degrees
    }

    public final double height;
    public final double pitch;
  }

  public class AprilTags {
    /**
     * {@link
     * https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json}
     */
    public static final AprilTag[] TAGS =
        new AprilTag[] {
          new AprilTag(
              1,
              new Pose3d(
                  new Translation3d(16.697198, 0.65532, 1.4859),
                  new Rotation3d(
                      new Quaternion(0.4539904997395468, 0.0, 0.0, 0.8910065241883678)))),
          new AprilTag(
              2,
              new Pose3d(
                  new Translation3d(16.697198, 7.3964799999999995, 1.4859),
                  new Rotation3d(
                      new Quaternion(-0.45399049973954675, -0.0, 0.0, 0.8910065241883679)))),
          new AprilTag(
              3,
              new Pose3d(
                  new Translation3d(11.560809999999998, 8.05561, 1.30175),
                  new Rotation3d(
                      new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
          new AprilTag(
              4,
              new Pose3d(
                  new Translation3d(9.276079999999999, 6.137656, 1.8679160000000001),
                  new Rotation3d(
                      new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0)))),
          new AprilTag(
              5,
              new Pose3d(
                  new Translation3d(9.276079999999999, 1.914906, 1.8679160000000001),
                  new Rotation3d(
                      new Quaternion(0.9659258262890683, 0.0, 0.25881904510252074, 0.0)))),
          new AprilTag(
              6,
              new Pose3d(
                  new Translation3d(13.474446, 3.3063179999999996, 0.308102),
                  new Rotation3d(
                      new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994)))),
          new AprilTag(
              7,
              new Pose3d(
                  new Translation3d(13.890498, 4.0259, 0.308102),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          new AprilTag(
              8,
              new Pose3d(
                  new Translation3d(13.474446, 4.745482, 0.308102),
                  new Rotation3d(
                      new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
          new AprilTag(
              9,
              new Pose3d(
                  new Translation3d(12.643358, 4.745482, 0.308102),
                  new Rotation3d(
                      new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
          new AprilTag(
              10,
              new Pose3d(
                  new Translation3d(12.227305999999999, 4.0259, 0.308102),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          new AprilTag(
              11,
              new Pose3d(
                  new Translation3d(12.643358, 3.3063179999999996, 0.308102),
                  new Rotation3d(
                      new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
          new AprilTag(
              12,
              new Pose3d(
                  new Translation3d(0.851154, 0.65532, 1.4859),
                  new Rotation3d(
                      new Quaternion(0.8910065241883679, 0.0, 0.0, 0.45399049973954675)))),
          new AprilTag(
              13,
              new Pose3d(
                  new Translation3d(0.851154, 7.3964799999999995, 1.4859),
                  new Rotation3d(
                      new Quaternion(-0.8910065241883678, -0.0, 0.0, 0.45399049973954686)))),
          new AprilTag(
              14,
              new Pose3d(
                  new Translation3d(8.272272, 6.137656, 1.8679160000000001),
                  new Rotation3d(
                      new Quaternion(
                          5.914589856893349e-17,
                          -0.25881904510252074,
                          1.5848095757158825e-17,
                          0.9659258262890683)))),
          new AprilTag(
              15,
              new Pose3d(
                  new Translation3d(8.272272, 1.914906, 1.8679160000000001),
                  new Rotation3d(
                      new Quaternion(
                          5.914589856893349e-17,
                          -0.25881904510252074,
                          1.5848095757158825e-17,
                          0.9659258262890683)))),
          new AprilTag(
              16,
              new Pose3d(
                  new Translation3d(5.9875419999999995, -0.0038099999999999996, 1.30175),
                  new Rotation3d(
                      new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
          new AprilTag(
              17,
              new Pose3d(
                  new Translation3d(4.073905999999999, 3.3063179999999996, 0.308102),
                  new Rotation3d(
                      new Quaternion(-0.4999999999999998, -0.0, 0.0, 0.8660254037844387)))),
          new AprilTag(
              18,
              new Pose3d(
                  new Translation3d(3.6576, 4.0259, 0.308102),
                  new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
          new AprilTag(
              19,
              new Pose3d(
                  new Translation3d(4.073905999999999, 4.745482, 0.308102),
                  new Rotation3d(
                      new Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))),
          new AprilTag(
              20,
              new Pose3d(
                  new Translation3d(4.904739999999999, 4.745482, 0.308102),
                  new Rotation3d(
                      new Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))),
          new AprilTag(
              21,
              new Pose3d(
                  new Translation3d(5.321046, 4.0259, 0.308102),
                  new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
          new AprilTag(
              22,
              new Pose3d(
                  new Translation3d(4.904739999999999, 3.3063179999999996, 0.308102),
                  new Rotation3d(
                      new Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994))))
        };

    public static final Pose2d[] TAGS_POSE2D;

    static {
      TAGS_POSE2D = new Pose2d[TAGS.length];
      for (int i = 0; i < TAGS.length; i++) {
        TAGS_POSE2D[i] =
            new Pose2d(
                TAGS[i].pose.getTranslation().toTranslation2d(),
                TAGS[i].pose.getRotation().toRotation2d());
      }
    }

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        new AprilTagFieldLayout(
            List.of(AprilTags.TAGS), FieldConstants.fieldLength, FieldConstants.fieldWidth);
  }
}
