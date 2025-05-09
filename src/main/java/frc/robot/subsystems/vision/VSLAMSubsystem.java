package frc.robot.subsystems.vision;

import java.util.EnumSet;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DrivetrainVisionCallback;

public class VSLAMSubsystem {

    private float yaw_offset = 0.0f;

    private NetworkTableInstance networkTableInstance;
    private NetworkTable ntDatatable;
    private Pose2d resetPoseOculus = new Pose2d();
    private Pose2d resetPoseRobot = new Pose2d();

    private IntegerSubscriber questMiso;
    private IntegerPublisher questMosi;
    private DoubleArrayPublisher resetPosePub;
    // private IntegerSubscriber questFrameCount;
    private DoubleSubscriber questTimestamp;
    private FloatArraySubscriber questPosition;
    // private FloatArraySubscriber questQuaternion;
    private FloatArraySubscriber questEulerAngles;
    private DoubleSubscriber questBatteryPercent;
    private DoubleSubscriber heartbeatRequestSub;
    private DoublePublisher heartbeatResponsePub;
    private double lastProcessedHeartbeatId = 0;
    private BooleanSubscriber questIsTracking;

    private DrivetrainVisionCallback visionMeasurementCallback;

    private final Field2d oculusPoseField = new Field2d();
    private final Field2d oculusRawPoseField = new Field2d();

    /**
     * Transform from the robot center to the headset. Coordinate system: - X:
     * Positive is forwards -
     * Y: Positive is left - Rotation: Positive is counter-clockwise
     */
    public static final Transform2d ROBOT_TO_OCULUS = new Transform2d(Units.inchesToMeters(6.0),
            Units.inchesToMeters(-10), new Rotation2d());

    public VSLAMSubsystem(DrivetrainVisionCallback visionCallback) {
        visionMeasurementCallback = visionCallback;
        networkTableInstance = NetworkTableInstance.getDefault();
    }

    public void configure() {
        // add a connection listener; the first parameter will cause the
        // callback to be called immediately for any current connections
        addNTConnectionListener();

        // get the subtable called "questnav"
        ntDatatable = networkTableInstance.getTable("questnav");
        populateFromDatatable();

        System.out.println("add positionlistener");
        // add a listener to only value changes on the Y subscriber
        addPositionListener();

        // add a listener to see when new topics are published within datatable
        // the string array is an array of topic name prefixes.
        addDatatableListener();

        SmartDashboard.putData("oculus final answer pose", oculusPoseField);
        SmartDashboard.putData("oculus Raw pose", oculusRawPoseField);
    }

    // Get the yaw Euler angle of the headset
    public float getYaw() {
        float[] eulerAngles = questEulerAngles.get();
        var ret = eulerAngles[1] - yaw_offset;
        ret %= 360;
        if (ret < 0) {
            ret += 360;
        }
        return ret * -1;
    }

    public Translation2d getPosition() {
        float[] oculusPosition = questPosition.get();
        return new Translation2d(oculusPosition[2], -oculusPosition[0]);
    }

    public void calculateNewOffset(Pose2d newPose) {
        System.out.println("VSLAMSubsystem: calculating a new offset");
        resetPoseOculus = new Pose2d().transformBy(ROBOT_TO_OCULUS.inverse());
        resetPoseRobot = newPose;
        resetQuestHeading();
    }

    public void resetQuestHeading() {
        System.out.println("VSLAMSubsystem: resetting QuestNav heading");
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
    }

    public void zeroHeading() {
        float[] eulerAngles = questEulerAngles.get();
        yaw_offset = eulerAngles[1];
    }

    // Return the robot heading in degrees, between -180 and 180 degrees
    public double getHeading() {
        return Rotation2d.fromDegrees(getYaw()).getDegrees();
    }

    // Get the rotation rate of the robot
    public double getTurnRate() {
        return getYaw();
    }

    public void cleanUpSubroutineMessages() {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    /** Process heartbeat requests from Quest and respond with the same ID */
    public void processHeartbeat() {
        double requestId = heartbeatRequestSub.get();
        // Only respond to new requests to avoid flooding
        if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
            heartbeatResponsePub.set(requestId);
            lastProcessedHeartbeatId = requestId;
        }
    }

    private int addNTConnectionListener() {
        return networkTableInstance.addConnectionListener(true, event -> {
            if (event.is(NetworkTableEvent.Kind.kConnected)) {
                System.out.println("Connected to " + event.connInfo.remote_id);


            } else if (event.is(NetworkTableEvent.Kind.kDisconnected)) {
                System.out.println("Disconnected from " + event.connInfo.remote_id);
            }
        });
    }

    private int addPositionListener() {
        return networkTableInstance.addListener(
                questPosition,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    var timestampedPosition = questPosition.getAtomic();
                    float[] oculusPosition = timestampedPosition.value;
                    double timestamp = timestampedPosition.timestamp;
                    timestamp = timestamp / 1000000;
                    Rotation2d oculousRawRotation = Rotation2d.fromDegrees(getYaw());
                    Translation2d oculousRawPosition = new Translation2d(oculusPosition[2], -oculusPosition[0]);
                    Pose2d oculousRawPose = new Pose2d(oculousRawPosition, oculousRawRotation);

                    var poseRelativeToReset = oculousRawPose.minus(resetPoseOculus);
                    var estPose = resetPoseRobot.transformBy(poseRelativeToReset);

                    estPose = estPose.transformBy(ROBOT_TO_OCULUS.inverse());

                    SmartDashboard.putNumber("timestamp from nt", timestamp);
                    SmartDashboard.putNumber("timestamp current from FPGA)", Timer.getFPGATimestamp());
                    timestamp = Utils.fpgaToCurrentTime(timestamp);
                    SmartDashboard.putNumber("converted timestamp from FPGA)", timestamp);
                    SmartDashboard.putNumber("timestamp from oculus", questTimestamp.getAsDouble());

                    SmartDashboard.putString("VSLAM pose", String.format("(%.2f, %.2f) %.2f %.2f %.2f",
                            estPose.getTranslation().getX(),
                            estPose.getTranslation().getY(),
                            estPose.getRotation().getDegrees(),
                            timestamp,
                            Timer.getFPGATimestamp()));
                    oculusPoseField.setRobotPose(estPose);
                    if (isConnected()) {
                    visionMeasurementCallback.addVisionMeasurement(estPose, timestamp, VisionConstants.kVSLAMStdDevs);
                    }
                });
    }

    private int addDatatableListener() {
        return networkTableInstance.addListener(
                new String[] { ntDatatable.getPath() + "/" },
                EnumSet.of(NetworkTableEvent.Kind.kTopic),
                event -> {
                    if (event.is(NetworkTableEvent.Kind.kPublish)) {
                        // topicInfo.name is the full topic name, e.g. "/datatable/X"
                        System.out.println("newly published " + event.topicInfo.name);
                    }
                });
    }

    private void populateFromDatatable() {
        questMiso = ntDatatable.getIntegerTopic("miso").subscribe(0);
        questMosi = ntDatatable.getIntegerTopic("mosi").publish();
        // questFrameCount = ntDatatable.getIntegerTopic("frameCount").subscribe(0);
        questTimestamp = ntDatatable.getDoubleTopic("timestamp").subscribe(0.0f);
        questPosition = ntDatatable.getFloatArrayTopic("position").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        // questQuaternion = ntDatatable.getFloatArrayTopic("quaternion").subscribe(new
        // float[] { 0.0f, 0.0f, 0.0f, 0.0f });
        questEulerAngles = ntDatatable.getFloatArrayTopic("eulerAngles").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        questBatteryPercent = ntDatatable.getDoubleTopic("device/batteryPercent").subscribe(0.0f);
        resetPosePub = ntDatatable.getDoubleArrayTopic("resetpose").publish();
        heartbeatRequestSub = ntDatatable.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
        heartbeatResponsePub = ntDatatable.getDoubleTopic("heartbeat/robot_to_quest").publish();
        questIsTracking = ntDatatable.getBooleanTopic("device/isTracking").subscribe(false);

    }



    /**
     * Returns if the Quest is connected
     * 
     * @return true if the Quest is connected
     */
    public boolean isConnected() {
        return areGettingUpdates() && isTracking();
    }
    public boolean areGettingUpdates() {
        return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
    }
    public boolean isTracking() {
        return questIsTracking.getAsBoolean();
    }

}