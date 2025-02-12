package frc.robot.commands;

import java.util.Optional;

import frc.robot.subsystems.SwerveDriveWPI;

public class AutoAlignBest extends AutoAlignBase {
  public AutoAlignBest(SwerveDriveWPI swerve, Alignment align) {
    super(swerve, align);
  }
  @Override
  protected Optional<TagPosition> getTagPosition() {
    return Optional.empty();
  }
}
