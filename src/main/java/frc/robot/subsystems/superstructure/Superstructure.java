package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.*;

public class Superstructure {
    private MotionPlanner planner;
    private ClimbSubsystem climb;

    private Level targetLevel = Level.L4;

    private static boolean shouldPluckAlgae = false;
    private static boolean shouldPreventL4 = false;
    private static boolean algaeMode = false;

    public enum Level {
        L1(PositionsFactory.getCoralScoreL1Positions(), PositionsFactory.getAlgaeFloorPickupPositions(), PositionsFactory.getAlgaeHandoffPositions()),
        L2(PositionsFactory.getCoralScoreL2Positions(), PositionsFactory.getAlgaeReefL2PickupPositions(), PositionsFactory.getAlgaeScoreBargePositions()),
        L3(PositionsFactory.getCoralScoreL3Positions(), PositionsFactory.getAlgaeReefL3PickupPositions(), PositionsFactory.getAlgaeScoreBargePositions()),
        L4(PositionsFactory.getCoralScoreL4Positions(), PositionsFactory.getAlgaeReefL3PickupPositions(), PositionsFactory.getAlgaeScoreBargePositions());

        public final Positions coralPositions, algaeIntakePositions, algaeScorePositions;
        private Level(Positions coral, Positions algaeIntake, Positions algaeScore) {
            this.coralPositions = coral;
            this.algaeIntakePositions = algaeIntake;
            this.algaeScorePositions = algaeScore;
        }
    }

    public Superstructure(ArmSubsystem arm, ElevatorSubsystem elevator, HandClamperSubsystem hand, HandIntakeSubsystem intake, ClimbSubsystem climb) {
        this.climb = climb;
        this.planner = new MotionPlanner(arm, elevator, hand, intake);
    }

    // ===========================
    //        MAIN COMMANDS
    // =========================== 

    public Command intakeCommand() {
        return Commands.either(
            switch (targetLevel) {
                case L1 -> this.planner.intakeFloorAlgae();
                default -> this.planner.intakeReefAlgae(getTargetLevel().algaeIntakePositions);
            },
            this.planner.intakeCoralFeeder(), 
            () -> algaeMode
        ).unless(() -> this.planner.hasPiece());
    }

    public Command finishIntakeCommand() {
        return Commands.either(
            switch (targetLevel) {
                case L1 -> this.planner.finishGroundAlgaeIntake();
                default -> this.planner.finishReefAlgaeIntake(getTargetLevel().algaeIntakePositions);
            }, 
            planner.homeAll(),
            () -> algaeMode
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command scoreCommand() {
        return Commands.either(
            switch (targetLevel) {
                case L1 -> this.planner.initializeProcessorScore();
                default -> this.planner.scoreAlgaeNet();
            },
            planner.initializeCoralScore(getTargetLevel().coralPositions, getTargetLevel() == Level.L1, shouldPreventL4), 
            () -> algaeMode
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command finishScoreCommand() {
        return Commands.either(
            switch (targetLevel) {
                case L1 -> this.planner.finishProcessorScore();
                default -> this.planner.homeAll()
                                .andThen(
                                    Commands.either(
                                        this.planner.moveArm(ArmConstants.minArmPosition), 
                                        Commands.none(),
                                        () -> planner.hasPiece()
                                    )
                                );
            }, 
            switch (targetLevel) {
                case L1 -> this.planner.finishCoralScoreL1();
                case L2 -> this.planner.finishCoralScoreL2();
                default -> this.planner.finishCoralScoreHigh(getTargetLevel().coralPositions, shouldPluckAlgae);
            }, 
            () -> algaeMode
        ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command setClimbingCommand() {
        return climb.setIsClimbingCommand(true)
        .andThen(Commands.parallel(
            climb.readyCommand(),
            planner.clearArmForClimb()
        ));
    }

    // ===========================
    //       Helper Commands
    // ===========================

    public Level getTargetLevel() {
        return (targetLevel == Level.L4 && shouldPreventL4) ? Level.L2 : targetLevel;
    }

    public Command setShouldPreventL4Command(boolean preventL4) {
        return new InstantCommand(() -> shouldPreventL4 = preventL4);
    }

    public Command setShouldPluckAlgaeCommand(boolean pluckAlgae) {
        return new InstantCommand(() -> shouldPluckAlgae = pluckAlgae);
    }

    public Command setLevelCommand(Level level) {
        return new InstantCommand(() -> this.targetLevel = level);
    }

    public Command setAlgaeModeCommand(boolean mode) {
        return new InstantCommand(() -> algaeMode = mode);
    }
}