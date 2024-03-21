package org.frc5687.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Optional;

import javax.swing.text.html.HTML.Tag;

import org.frc5687.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonProcessor {

    private static final int NUM_CAMERAS = 4;

    private static final int NE_IDX = 0;
    private static final int NW_IDX = 1;
    private static final int SE_IDX = 2;
    private static final int SW_IDX = 3;

    private final PhotonCamera[] _cameras = new PhotonCamera[NUM_CAMERAS];
    private final PhotonPoseEstimator[] _estimators = new PhotonPoseEstimator[NUM_CAMERAS];

    private final Transform3d[] _robotToCamera = new Transform3d[NUM_CAMERAS];

    private ArrayList<TagTransform> _tags = new ArrayList<>();

    public PhotonProcessor(AprilTagFieldLayout layout) {
        _cameras[NE_IDX] = new PhotonCamera("North_East_Camera");
        _cameras[NW_IDX] = new PhotonCamera("North_West_Camera");
        _cameras[SE_IDX] = new PhotonCamera("South_East_Camera");
        _cameras[SW_IDX] = new PhotonCamera("South_West_Camera");

        // TODO these should be in config and also they should be inverted

        _robotToCamera[NE_IDX] = new Transform3d(
            new Pose3d(new Translation3d(Units.inchesToMeters(3.90), Units.inchesToMeters(-7.05), Units.inchesToMeters(11.00)), new Rotation3d(0.0, Units.degreesToRadians(16.5), Units.degreesToRadians(-25.5))),
            new Pose3d()
        );
        _robotToCamera[NW_IDX] = new Transform3d(
            new Pose3d(new Translation3d(Units.inchesToMeters(3.90), Units.inchesToMeters(7.05), Units.inchesToMeters(11.00)), new Rotation3d(0.0, Units.degreesToRadians(16.5), Units.degreesToRadians(25.5))),
            new Pose3d()
        );
        _robotToCamera[SE_IDX] = new Transform3d(
            new Pose3d(new Translation3d(-0.107009, -0.104835, 0.57991), new Rotation3d(0.0, 0.0, Units.degreesToRadians(162.5))),
            new Pose3d()
        );
        _robotToCamera[SW_IDX] = new Transform3d(
            new Pose3d(new Translation3d(-0.107009, 0.104835, 0.57991), new Rotation3d(0.0, 0.0, Units.degreesToRadians(-162.5))),
            new Pose3d()
        );

        for (int i = 0; i < NUM_CAMERAS; i++) {
            _estimators[i] = new PhotonPoseEstimator(
                layout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                _cameras[i],
                _robotToCamera[i]
            );
            // use the tag with a lower pose ambiguity if multiple tags are visible (this may not be used with xavier's vision system)
            _estimators[i].setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        }
    }

    /**
     * Updates the locally cached <code>_tags</code> to those given by the cameras.
     * @param robotHeading the heading in the field frame (_heading on blue side and _heading + Math.PI on red side)
     */
    public void updateFieldOrientedRobotFrameTagPositions(Rotation2d robotHeading) {
        ArrayList<TagTransform> tags = new ArrayList<TagTransform>();

        for (int i = 0; i < NUM_CAMERAS; i++) {
            PhotonPipelineResult results = _cameras[i].getLatestResult();

            if (results.hasTargets()) {
                for (PhotonTrackedTarget target : results.getTargets()) {
                    if (target.getPoseAmbiguity() <= 0.2) {
                        Transform3d cameraToTarget = target.getBestCameraToTarget();
                        // System.out.println("Southwest cam sees tag "+target.getFiducialId()+" with transform " + cameraToTarget);
                        Transform3d robotToCamera = _robotToCamera[i].inverse();
                        Transform3d robotToTarget = robotToCamera.plus(cameraToTarget);
                        // rotate around the origin of the robot frame by the robot's heading, aligning its axes with the field frame
                        Transform3d fieldOrientedRobotRelativeTagPosition = robotToTarget.plus(new Transform3d(new Translation3d(), new Rotation3d(0.0, 0.0, -robotHeading.getRadians())));
                        
                        tags.add(new TagTransform(fieldOrientedRobotRelativeTagPosition.getTranslation(), target.getFiducialId(), results.getTimestampSeconds()));
                    }
                }
            }
        }

        _tags = tags;
    }

    /**
     * Get the locally cached list of tag poses in a reference frame which has the origin at the center of the robot and the axes aligned with those of the field.
     * @return a list of all tag poses
     */
    public ArrayList<TagTransform> getTagPoses() {
        return _tags;
    }

    /**
     * Gets a given locally cached tag pose in a reference frame which has the origin at the center of the robot and the axes aligned with those of the field. The rotation of the pose is empty.
     * @param tagId The ID of the tag to look up in the local <code>_tags</code> variable
     * @return Optional.of(Pose2d) if a camera can see the tag, Optional.empty if no camera can see the tag.
     */
    public Optional<Pose2d> getPoseToTag(int tagId) {
        double xToTagAccumulator = 0.0;
        double yToTagAccumulator = 0.0;
        int tagCount = 0;

        for (TagTransform tag : _tags) {
            if (tag.tagId == tagId) {
                xToTagAccumulator += tag.translation.getX();
                yToTagAccumulator += tag.translation.getY();
                tagCount ++;
            }
        }

        if (tagCount == 0) {
            return Optional.empty();
        } else {
            return Optional.of(new Pose2d(xToTagAccumulator / tagCount, yToTagAccumulator / tagCount, new Rotation2d()));
        }

    }

    /**
     * a dumb class (there is probably a better thing to use for this)
     * - xavier bradford 03/21/24
     */
    public class TagTransform {
        public final Translation3d translation;
        public final int tagId;
        public final double timestamp;

        public TagTransform(Translation3d translation, int tagId, double timestamp) {
            this.translation = translation;
            this.tagId = tagId;
            this.timestamp = timestamp;
        }
    }
}
