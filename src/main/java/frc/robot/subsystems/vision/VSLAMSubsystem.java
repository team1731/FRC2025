package frc.robot.subsystems.vision;

import java.util.EnumSet;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
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
   // private final Field2d vslamField = new Field2d();
    private float yaw_offset = 0.0f;

    private NetworkTableInstance networkTableInstance;
    private NetworkTable ntDatatable;
    private Pose2d startingOffset;
    private Pose2d resetPoseOculus = new Pose2d();
    private Pose2d resetPoseRobot = new Pose2d();
    

    private IntegerSubscriber questMiso;
    private IntegerPublisher questMosi;
    private DoubleArrayPublisher   resetPosePub;
    //private IntegerSubscriber questFrameCount;
    private DoubleSubscriber questTimestamp;
    private FloatArraySubscriber questPosition;
    //private FloatArraySubscriber questQuaternion;
    private FloatArraySubscriber questEulerAngles;
    private DoubleSubscriber questBattery;

    private DrivetrainVisionCallback drivetrainCallback;

    private final Field2d oculusPoseField = new Field2d();
    private final Field2d oculusRawPoseField = new Field2d();
// Do this in either robot or subsystem init

// Do this in either robot periodic or subsystem periodic

      /**
   * Transform from the robot center to the headset. Coordinate system: - X: Positive is forwards -
   * Y: Positive is left - Rotation: Positive is counter-clockwise
   */
     public static final Transform2d ROBOT_TO_OCULUS = new Transform2d(Units.inchesToMeters(6.0), Units.inchesToMeters(-10), new Rotation2d());

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

       // ShuffleboardTab tab = Shuffleboard.getTab("test");
        //tab.add(vslamField);

        System.out.println("add ind listener******************************************");
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
        return ret*-1;
    }

    public Translation2d getPosition() {
        float[] oculusPosition = questPosition.get();
        return new Translation2d(oculusPosition[2], -oculusPosition[0]);
    }
    
   // public Pose2d getPose() {
   //     var oculousPositionCompensated = getPosition().minus(new Translation2d(0, 0.1651)); // 6.5
    //    return new Pose2d(oculousPositionCompensated, Rotation2d.fromDegrees(getYaw()));
   // }  
/* 
    public void calculateNewOffset(Pose2d position) {
        System.out.println("in calculateNewOffset");
       // startingOffset = position.transformBy(ROBOT_TO_OCULUS);
      //  Translation2d cameraoffset = position.getTranslation().minus(new Translation2d(.33333,0));
       // startingOffset = new Pose2d(cameraoffset,position.getRotation());
       startingOffset = position;
        System.out.println(
            "Calculate New Offset" +
            String.format(
                "Initiating pose reset to X:%.2f Y:%.2f Rot:%.2f°",
                startingOffset.getX(), startingOffset.getY(), startingOffset.getRotation().getDegrees()));
        if (questMiso.get() != 99) {
          questMosi.set(1);
        }
    }
        */

    public void calculateNewOffset(Pose2d newPose) {
        resetPoseOculus = new Pose2d().transformBy(ROBOT_TO_OCULUS.inverse());
        resetPoseRobot = newPose;
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
                Rotation2d oculousRawRotation = Rotation2d.fromDegrees(getYaw());
                Translation2d oculousRawPosition = new Translation2d(oculusPosition[2], -oculusPosition[0]);
                Pose2d oculousRawPose = new Pose2d(oculousRawPosition,oculousRawRotation);

              //  Pose2d oculusrobotmove = oculousRawPose.transformBy(ROBOT_TO_OCULUS.inverse());
              //  oculusRawPoseField.setRobotPose(oculusrobotmove);

              //  var positionrelativetoreset = oculousRawPose.minus(resetPoseOculus);
               // var estPose = resetPoseRobot.plus(positionrelativetoreset);
                //var estPose = resetPoseRobot.transformBy(oculousRawPose);
                

              //  Pose2d robotRawPose = oculousRawPose.plus(ROBOT_TO_OCULUS.getTranslation()).plus(ROBOT_TO_OCULUS.getTranslation().times(-1).rotateBy(oculousRawRotation ));



                var poseRelativeToReset = oculousRawPose.minus(resetPoseOculus);
                var estPose = resetPoseRobot.transformBy(poseRelativeToReset);

                estPose = estPose.transformBy(ROBOT_TO_OCULUS.inverse());



                SmartDashboard.putNumber("timestamp from nt",timestamp);
                SmartDashboard.putNumber("timestamp current from FPGA)",Timer.getFPGATimestamp());
                timestamp  = Utils.fpgaToCurrentTime(timestamp);
                SmartDashboard.putNumber("converted timestamp from FPGA)",timestamp);
                SmartDashboard.putNumber("timestamp from oculus",questTimestamp.getAsDouble());

                
              //  Rotation2d  oculousCompensatedRotation = oculousRawRotation.plus(startingOffset.getRotation());
                
              //  Pose2d oculusPose = new Pose2d(oculousPositionCompensated, oculousCompensatedRotation);
              //  oculusRawPoseField.setRobotPose(startingOffset);
               // System.out.println(
                //    "using Offset" +
                //    String.format(
                //        "Initiating pose reset to X:%.2f Y:%.2f Rot:%.2f°",
                //        startingOffset.getX(), startingOffset.getY(), startingOffset.getRotation().getDegrees()));
               


               // System.out.println("addind a vslam");
               // vslamField.getObject("MyRobotVSLAM").setPose(estPose);
                SmartDashboard.putString("VSLAM pose", String.format("(%.2f, %.2f) %.2f %.2f %.2f",
                        estPose.getTranslation().getX(),
                        estPose.getTranslation().getY(),
                        estPose.getRotation().getDegrees(),
                        timestamp,
                        Timer.getFPGATimestamp()));
                 oculusPoseField.setRobotPose(estPose);
                //drivetrainCallback.addVisionMeasurement(estPose, timestamp, kVSLAMStdDevs);
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
        questTimestamp = ntDatatable.getDoubleTopic("timestamp").subscribe(0.0f);
        questPosition = ntDatatable.getFloatArrayTopic("position").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        //questQuaternion = ntDatatable.getFloatArrayTopic("quaternion").subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
        questEulerAngles = ntDatatable.getFloatArrayTopic("eulerAngles").subscribe(new float[] { 0.0f, 0.0f, 0.0f });
        questBattery = ntDatatable.getDoubleTopic("batteryLevel").subscribe(0.0f);
        resetPosePub = ntDatatable.getDoubleArrayTopic("resetpose").publish();
    }

// This method didn't really work - not sure why so we are storing the offset in the roborio for now
    public boolean resetToPoseOnQuest(Pose2d targetPose) {
       /*  if (poseResetInProgress) {
          Logger.recordOutput("Oculus/status", "Cannot reset pose - reset already in progress");
          return false;
        }
    
        if (inputs.misoValue != STATUS_READY) {
          Logger.recordOutput(
              "Oculus/status", "Cannot reset pose - Quest busy (MISO=" + inputs.misoValue + ")");
          return false;
        }
    */
        targetPose = targetPose.plus(ROBOT_TO_OCULUS);
      //  pendingResetPose = targetPose;
        System.out.println(
            "Oculus/status" +
            String.format(
                "Initiating pose reset to X:%.2f Y:%.2f Rot:%.2f°",
                targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()));
    
        resetPosePub.set(new double[] {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});


      //  poseResetInProgress = true;
      //  resetStartTime = Timer.getTimestamp();
       // currentResetAttempt = 0;
        questMosi.set(2); // Request pose reset
    
        return true;
      }
        /**
   * Returns if the Quest is connected
   * 
   * @return true if the Quest is connected
   */
  public boolean isConnected() {
    return ((RobotController.getFPGATime() - questBattery.getLastChange()) / 1000) < 250;
  }
}