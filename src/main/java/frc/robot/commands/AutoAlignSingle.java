package frc.robot.commands;

import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.subsystems.SwerveDriveWPI;

import java.util.Optional;

public class AutoAlignSingle extends AutoAlignBase {
  
  private NetworkTableQuerier.TagCollection tags;
  private int tagId;

  public AutoAlignSingle(SwerveDriveWPI swerve, Alignment align, NetworkTableQuerier.TagCollection tags, int tagId) {
    super(swerve, align);
    this.tags = tags;
    this.tagId = tagId;
  }

  @Override
  protected Optional<TagPosition> getTagPosition() {
    return tags.tagIndex(tagId).map(idx -> new TagPosition(tags.distances[idx], tags.distances[idx], tags.offsets[idx]));
  }
  @Override
  protected void preExectute() {
    tags.lock.readLock().lock();
  }
  @Override
  protected void postExecute() {
    tags.lock.readLock().unlock();
  }
}
