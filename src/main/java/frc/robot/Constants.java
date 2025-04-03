package frc.robot;

import java.util.Map;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.log.LogWriter.Log;
import frc.robot.util.log.LogWriter.LogMode;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final int kTICKS = 33024; // 16.125 * 2048;
    public static final String CANBUS_NAME = "canivore1";

    public static final class JoystickConstants {
        public static final int opA = 1;
        public static final int opB = 2;
        public static final int opC = 3;
        public static final int opD = 4;
        public static final int opE = 5;
        public static final int opF = 6;
        public static final int opG = 7;
        public static final int opH = 8;
        public static final int opI = 9;
        public static final int opJ = 10;
        public static final int opK = 11;
        public static final int opL = 12;

        public static final int op1 = 1;
        public static final int op2 = 2;
        public static final int op3 = 3;
        public static final int op4 = 4;
        public static final int op5 = 5;
        public static final int op6 = 6;
        public static final int op7 = 7;
        public static final int op8 = 8;
    }

    public static final class LogConstants {
        /*
         * To write to a log you must:
         * 1. Set loggingEnabled = true
         * 2. Set the desired logMode (CSV, DATA_LOG)
         * 2. Set the desired loggers below = true
         */
        public static final boolean loggingEnabled = false; // note: must also turn on applicable loggers below
        public static final boolean logNetworkTables = false; // only applicable when logMode = DATA_LOG
        public static final LogMode logMode = LogMode.CSV;

        // list of loggers and enabled status, note: must also enable logging above
        public static final Map<Log, Boolean> loggers = Map.of(
                Log.MESSAGE, false,
                Log.ARM_PATH_RECORDING, false,
                Log.POSE_ESTIMATIONS, false);
    }

    public static final class AutoConstants {
        public static final String kNoVSLAMPostfix = "_NoVSLAM";
        public static final String kAutoDefault = "1_Barge"; // Note: when setting the default auto, do not include Blu_/Red_ prefixes
        public static final String kAutoCodeKey = "Auto Selector";

        public static final double kMaxSpeedMetersPerSecond = 0.5; // disabled for testing = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.5; // disabled for testing = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 3.0;
        public static final double kPYController = 3.0;
        public static final double kPThetaController = 2;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class OpConstants {
        // KEYBOARD CONSTANTS
        public static final int kPWM_LedString = 1; // PWM # for Addressable Led String
        public static final int kLedStringLength = 11; // Length of Addressable Led String; 33 individual / sets of 3
        public static final double kLedStringBlinkDelay = 0.1; // Delay in Seconds of Addressable Led String

        public enum LedOption {
            INIT, YELLOW, PURPLE, BLACK, WHITE, BLUE, RED, GREEN
        }
    }
}
