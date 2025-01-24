package frc.robot.subsystems.Limelight;

import java.lang.annotation.Target;
import java.util.List;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AprilTagStatsLimelight extends SubsystemBase {

    private final Drivetrain drivetrain;
    private final NetworkTable table;
    private AprilTagFieldLayout m_layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private int targetTagID;

    public AprilTagStatsLimelight() {
        this.drivetrain = Drivetrain.getInstance();
        this.table = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.limelightNetworkTableKey.LIMELIGHT_NETWORKTABLE_KEY);
        configureAliance();
    }


    public void updateStats() {
        double x = getTX();
        double y = getTY();
        double area = getArea();
        double id = getID();

        if (hasValidTargets()) {
            SmartDashboard.putBoolean("Has Targets", true);
            updateRobotPoseInSmartDashboard();
        } else {
            SmartDashboard.putBoolean("Has Targets", false);
        }

        updateValues(x, y, area, id);
    }

    public double getTX() {
        return getEntryValue("tx");
    }

    public double getTY() {
        return getEntryValue("ty");
    }

    public boolean hasValidTargets() {
        return getEntryValue("tv") == 1;
    }

    public double getArea() {
        return getEntryValue("ta");
    }

    public int getID() {
        if (hasValidTargets()) return (int) getEntryValue("tid");
        return -1;
    }

    public double getYaw() {
        double yaw = 0;
        Pose3d robot = getBotPose();
        if (robot == null) {
            return yaw;
        }
        return robot.getRotation().getZ();
    }

    public Pose3d getBotPose() {
        double[] botpose = table.getEntry("botpose").getDoubleArray(new double[6]);
        if (botpose.length < 6) {
            return null;
        }
        return new Pose3d(
            botpose[0],
            botpose[1],
            botpose[2],
            new Rotation3d(
                Math.toRadians(botpose[3]),
                Math.toRadians(botpose[4]),
                Math.toRadians(botpose[5])
            )
        );
    }

    private double getEntryValue(String entryName) {
        return table.getEntry(entryName).getDouble(0.0);
    }

    private void updateValues(double x, double y, double area, double id) {
        SmartDashboard.putNumber("Limelight X", x);
        SmartDashboard.putNumber("Limelight Y", y);
        SmartDashboard.putNumber("Limelight Area", area);
        SmartDashboard.putNumber("Limelight ID", id);
    }

    public double getPitch() {
        Pose3d pose = getBotPose();
        return pose != null ? pose.getRotation().getY() : 0.0;
    }

    private void updateRobotPoseInSmartDashboard() {
        boolean hasTarget = hasValidTargets();
        SmartDashboard.putBoolean("Limelight/Has Target", hasTarget);

        if (hasTarget) {
            Pose3d pose = getBotPose();
            if (pose != null) {
                updatePoseDashboard(pose);
            }
        } else {
            clearPoseDashboard();
        }
    }

    private void updatePoseDashboard(Pose3d pose) {
        SmartDashboard.putNumber("Limelight/Position/X", pose.getX());
        SmartDashboard.putNumber("Limelight/Position/Y", pose.getY());
        SmartDashboard.putNumber("Limelight/Position/Z", pose.getZ());
        SmartDashboard.putNumber("Limelight/Rotation/Roll", Math.toDegrees(pose.getRotation().getX()));
        SmartDashboard.putNumber("Limelight/Rotation/Pitch", Math.toDegrees(pose.getRotation().getY()));
        SmartDashboard.putNumber("Limelight/Rotation/Yaw", Math.toDegrees(pose.getRotation().getZ()));
        SmartDashboard.putNumber("Limelight/Distance", calculateDistance(getID()));
    }

    private void clearPoseDashboard() {
        SmartDashboard.putNumber("Limelight/Position/X", 0);
        SmartDashboard.putNumber("Limelight/Position/Y", 0);
        SmartDashboard.putNumber("Limelight/Position/Z", 0);
        SmartDashboard.putNumber("Limelight/Rotation/Roll", 0);
        SmartDashboard.putNumber("Limelight/Rotation/Pitch", 0);
        SmartDashboard.putNumber("Limelight/Rotation/Yaw", 0);
        SmartDashboard.putNumber("Limelight/Distance", 0);
    }

    public double calculateDistance(int apriltagID){
        if (apriltagID == -1) return 0;

        final double TARGET_HEIGHT = m_layout.getTagPose(apriltagID).get().getY();
        final double CAMERA_HEIGHT = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_HEIGHT;
        final double CAMERA_PITCH = Constants.VisionConstants.limeLightDimensionConstants.CAMERA_PITCH; //check for accuracy

        double angleToSpeakerEntranceRadians = Math.toRadians( CAMERA_PITCH + getTY());
        return (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToSpeakerEntranceRadians);
    }

    // public boolean isReefTag() {
    //     return targetTagID.contains(Integer.valueOf(getID()));
    // }

    public void configureAliance(){
        var allianceColor = DriverStation.getAlliance();
        targetTagID = (allianceColor.get() == Alliance.Blue) ? Constants.VisionConstants.aprilTagIDConstants.BLUE_REEF_TAG_IDS : Constants.VisionConstants.aprilTagIDConstants.RED_REEF_TAG_IDS;
        table.getEntry("pipeline").setNumber(targetTagID);
    }

    public void updateLimelightTracking() {
        table.getEntry("camMode").setNumber(0); // Sets the vision processing mode 
        table.getEntry("ledMode").setNumber(3); // Forces the LED to stay on always
    
    }
}