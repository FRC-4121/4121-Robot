package frc.robot.commands;

import java.util.Optional;

import frc.robot.subsystems.SwerveDriveWPI;
import frc.robot.ExtraClasses.NetworkTableQuerier;

public class AutoAlignBest extends AutoAlignBase {
  private NetworkTableQuerier.BestTag best;

  public AutoAlignBest(SwerveDriveWPI swerve, Alignment align, NetworkTableQuerier.BestTag best) {
    super(swerve, align);
    this.best = best;
  }

  @Override
  protected Optional<TagPosition> getTagPosition() {
    return best.best.map(tag -> new TagPosition(tag.distance(), tag.offset(), tag.rotation()));
  }
}
