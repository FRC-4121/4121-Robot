package frc.robot.commands;

import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.subsystems.SwerveDriveWPI;

import java.util.Optional;

public class AutoAlignSingle extends AutoAlignBase {
  
  private NetworkTableQuerier.TagCollection tags;
  private int tagId;

  public AutoAlignSingle(SwerveDriveWPI swerve, Alignment align, ToMove toMove, NetworkTableQuerier.TagCollection tags, int tagId) {
    super(swerve, align, toMove);
    this.tags = tags;
    this.tagId = tagId;
  }

  @Override
  protected Optional<TagPosition> getTagPosition() {
    return tags.tagIndex(tagId).map(idx -> new TagPosition(tags.distances[idx], tags.distances[idx], tags.offsets[idx]));
  }

  @Override
  public void initialize() {
    tags.lock.readLock().lock();
    super.initialize();
    tags.lock.readLock().unlock();
  }
}
