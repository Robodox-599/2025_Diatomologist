package frc.robot.subsystems.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.endefector.endefectorrollers.Rollers;
import frc.robot.util.ScoringUtil;

public class AlignToReef {
  private Rollers rollers;
  private Drive drive;

  public AlignToReef(Rollers rollers, Drive drive) {
    this.rollers = rollers;
    this.drive = drive;
  }

  public Command alignToLeft() {
    return Commands.run(
        () -> {
          MoveToPointCommand.moveToPointFast(
              drive,
              ScoringUtil.getNearestBranchPosition(
                  () -> drive.getPose(),
                  true,
                  new Translation2d(rollers.getCoralDistance(), new Rotation2d())));
        });
  }

  public Command alignToRight() {
    return Commands.run(
        () -> {
          MoveToPointCommand.moveToPointFast(
              drive,
              ScoringUtil.getNearestBranchPosition(
                  () -> drive.getPose(),
                  true,
                  new Translation2d(rollers.getCoralDistance(), new Rotation2d())));
        });
  }
}
