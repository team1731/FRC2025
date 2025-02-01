package frc.robot.subsystems.vision;

import java.util.EnumSet;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DrivetrainVisionCallback;

import static frc.robot.Constants.Vision.*;


public class VSLAMSubsystem {
    private final Field2d vslamField = new Field2d();
    private float yaw_offset = 0.0f;

    private NetworkTableInstance networkTableInstance;
    private NetworkTable ntDatatable;
    private Pose2d startingOffset;

    private IntegerSubscriber questMiso;
    private IntegerPublisher questMosi;
    //private IntegerSubscriber questFrameCount;
    //private DoubleSubscriber questTimestamp;
    private FloatArraySubscriber questPosition;
    //private FloatArraySubscriber questQuaternion;
    private FloatArraySubscriber questEulerAngles;
    //private DoubleSubscriber questBattery;

    private DrivetrainVisionCallback drivetrainCallback;

    
    public VSLAMSubsystem(DrivetrainVisionCallback callback) {
        drivetrainCallback = callback;
        networkTableInstance = NetworkTableInstance.getDefault();
        startingOffset = new Pose2d();
        configure();
    }

    public void configure() {
        // add a connection listener; the first parameter will cause the
        // callback to be called immediately for any current connections
        addNTConnectionListener();
        
        // get the subtable called "questnav"
        ntDatatable = networkTableInstance.getTable("questnav");
        populateFromDatatable();

        ShuffleboardTab tab = Shuffleboard.getTab("test");
        tab.add(vslamField);

        System.out.println("add ind listener******************************************");
        // add a listener to only value changes on the Y subscriber
        addPositionListener();

        // add a listener to see when new topics are published within datatable
        // the string array is an array of topic name prefixes.
        addDatatableListener();
    }

    // Get the yaw Euler angle of the headset
    public float getYaw() {
        float[] eulerAngles = questEulerAngles.get();
        var ret = eulerAngles[1] - yaw_offset;
        ret %= 360;
        if (ret < 0) {
            ret += 360;
        }
        return ret*-1;
    }

    public Translation2d getPosition() {
        float[] oculusPosition = questPosition.get();
        return new Translation2d(oculusPosition[2], -oculusPosition[0]);
    }
    
    public Pose2d getPose() {
        var oculousPositionCompensated = getPosition().minus(new Translation2d(0, 0.1651)); // 6.5
        return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getYaw()));
    }  

    public void setPose(Pose2d pose) {
        vslamField.setRobotPose(pose);
    }

    public void calculateNewOffset(Pose2d position) {
        Translation2d cameraoffset = position.getTranslation().minus(new Translation2d(.33333,0));
        startingOffset = new Pose2d(cameraoffset,position.getRotation());
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
        if(questMiso.get() == 99) {
            questMosi.set(0);
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
                timestamp = timestamp/1000000;
                Translation2d oculousRawPosition = new Translation2d(-oculusPosition[2], oculusPosition[0]);
                Translation2d  oculousPositionCompensated = oculousRawPosition.plus(new Translation2d(0.3333* Math.cos(Math.toRadians(getYaw())), 0.333*Math.sin(Math.toRadians(getYaw())))); // TODO GET Numbers since robot is not in the center of the robot
                oculousPositionCompensated = oculousPositionCompensated.plus(startingOffset.getTranslation());  // translate by the starting position

                Rotation2d oculousRawRotation = Rotation2d.fromDegrees(getYaw()).plus(Rotation2d.fromDegrees(0));  // since camera is on back of robot
                Rotation2d  oculousCompensatedRotation = oculousRawRotation.plus(startingOffset.getRotation());
                
                Pose2d estPose = new Pose2d(oculousPositionCompensated, oculousCompensatedRotation);
                
                //System.out.println("addind a vslam");
                vslamField.getObject("MyRobotVSLAM").setPose(estPose);
                SmartDashboard.putString("VSLAM pose", String.format("(%.2f, %.2f) %.2f %.2f %.2f",
                        estPose.getTranslation().getX(),
                        estPose.getTranslation().getY(),
                        estPose.getRotation().getDegrees(),
                        timestamp,
                        Timer.getFPGATimestamp()));
                
                drivetrainCallback.addVisionMeasurement(estPose, timestamp, kVSLAMStdDevs);
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
        //questFrameCount = ntDatatable.getIntegerTopic("frameCount").subscribe(0);
        //questTimestamp = ntDatatable.getDoubleTopic("timestamp").subscribe(0.0f);
        questPosition = ntDatatable.getFloatArrayTopic("position").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        //questQuaternion = ntDatatable.getFloatArrayTopic("quaternion").subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
        questEulerAngles = ntDatatable.getFloatArrayTopic("eulerAngles").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        //questBattery = ntDatatable.getDoubleTopic("batteryLevel").subscribe(0.0f);
    }
}