package frc.robot.commands;

import java.util.Optional;

import frc.robot.subsystems.SwerveDriveWPI;
import frc.robot.ExtraClasses.NetworkTableQuerier;

public class AutoAlignBest extends AutoAlignBase {
  private NetworkTableQuerier.BestTag best;
  private long[] filter;

  public AutoAlignBest(SwerveDriveWPI swerve, Alignment align, NetworkTableQuerier.BestTag best, long[] filter) {
    super(swerve, align);
    this.best = best;
    this.filter = filter;
  }

  @Override
  public void initialize() {
    best.filter = filter;
    best.refresh();
    super.initialize();
  }

  @Override
  protected Optional<TagPosition> getTagPosition() {
    return best.best.map(tag -> new TagPosition(tag.distance(), tag.offset(), tag.rotation()));
  }
}
