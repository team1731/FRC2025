package frc.robot.state.sequencer;

import javax.security.auth.callback.Callback;

import frc.robot.state.Input;
import frc.robot.state.StateMachine;
import frc.robot.state.StateMachineCallback;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandIntakeSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;
import frc.robot.subsystems.hand.HandConstants;

public class SequenceStateMachine extends StateMachine {
    // subsystems
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;
    private HandIntakeSubsystem handIntakeSubsystem;
    private HandClamperSubsystem handClamperSubsystem;

    // sequence tracking
    private Sequence currentSequence;
    private Action currentAction;
    private GamePiece currentGamePiece;
    private Positions positions;

    // reset/abort tracking
    private boolean isResetting = false;
    private boolean elevatorResetDone = false;
    private boolean armResetDone = false;


    public SequenceStateMachine(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem, HandClamperSubsystem handClamperSubsystem, HandIntakeSubsystem handIntakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armSubsystem = armSubsystem;
        this.handClamperSubsystem = handClamperSubsystem;
        this.handIntakeSubsystem = handIntakeSubsystem;
        setCurrentState(SequenceState.HOME);
    }

    public void hardReset() {
        resetState();
        setCurrentState(SequenceState.HOME);
        System.out.println("SequenceStateMachine: hard reset complete - current state: " + currentState);
    }

    /*
     * COMMAND INTERFACE
     */

    public boolean isReady() {
        return currentState == SequenceState.HOME;
    }

    public Sequence getCurrentSequence() {
        return currentSequence;
    }

    public void setSequence(Sequence sequence) {
        currentSequence = sequence;
        currentAction = SequenceManager.getActionSelection();
        currentGamePiece = SequenceManager.getGamePieceSelection();
        // the sequence determines the choreographed movement of elevator/arm/hand
        setStateTransitionTable(SequenceFactory.getTransitionTable(sequence)); // state machine transitions
        positions = SequenceFactory.getPositions(sequence); // position constants for subsystems
        
        // on reset, put it into special state to kick off reset
        // may be in an incomplete state from a previous sequence
        if(sequence == Sequence.RESET) setCurrentState(SequenceState.INIT_RESET);
    }

    // Called by the SequenceManager if the operator changes the level mid-stream
    // Note: Can only be used when the new sequence utilizes the same transition table
    public void overwriteSequenceForLevelChange(Sequence newSequence) {
        currentSequence = newSequence;
        positions = SequenceFactory.getPositions(newSequence); // overwrite w/positions for new level
    }


    /*
     * SUBSYSTEM INTERFACE
     */

    protected void handleSubsystemCallback(Input input) {
        if(isResetting) {
            System.out.println("SequenceStateMachine: reset input " + input);
            SequenceInput sequenceInput = (SequenceInput)input;
            if(sequenceInput == SequenceInput.ELEVATOR_THRESHOLD_MET) setInput(input); // not home yet
            if(sequenceInput == SequenceInput.ELEVATOR_DONE) elevatorResetDone = true;
            if(sequenceInput == SequenceInput.ARM_DONE) armResetDone = true;
            if(elevatorResetDone && armResetDone) {
                isResetting = false;
                if(SequenceManager.isCoralScoreSequence(currentSequence)) resetHandIfCoralNotDetected();
                setInput(SequenceInput.RESET_DONE);
            }
        } else {
            if(input == SequenceInput.RELEASED_PIECE) {
                closeHandWithoutCallback();
                System.out.println("closing hand without callback"); 
            }
            if(input == SequenceInput.DETECTED_PIECE && currentGamePiece == GamePiece.CORAL) holdCoralPiece();
            if(currentSequence == Sequence.SCORE_CORAL_L2 && input == SequenceInput.SENSOR_SCORE) return;
            if(currentSequence == Sequence.SCORE_CORAL_L2 && input == SequenceInput.ARM_DONE && currentState == SequenceState.SCORING) {
                releaseCoralPiece();
                return;
            }
            setInput(input);
        }
    }

    public StateMachineCallback getSubsystemCallback() {
        return subsystemCallback;
    }


    /*
     * ELEVATOR OPERATIONAL METHODS
     * Note: these are general methods shared by multiple sequences, use care when updating and understand what the impact
     * will be in other sequences. If you need something custom for a specific sequence, spin off a separate method.
     */


     public boolean raiseElevatorAndArmForBarge() {
        elevatorSubsystem.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback, positions.raiseElevatorThreshold);
        armSubsystem.moveArmSlowSpeed(-8.0);
        return true;
    }

    public boolean raiseElevatorAndArmForL1() {
        elevatorSubsystem.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback, positions.raiseElevatorThreshold);
        armSubsystem.moveArmSlowSpeed(positions.firstStageArmPosition, subsystemCallback);
        return true;
    }

    public boolean raiseElevator() {
        elevatorSubsystem.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback, positions.raiseElevatorThreshold);
        return true;
    }

    public boolean moveElevatorHome() {
        isResetting = true;
        elevatorSubsystem.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback, positions.lowerElevatorThreshold);
        if(SequenceManager.shouldPluckAlgae()) {
            handIntakeSubsystem.intakeWithCurrent();
        }
        return true;
    }

    // Note: first and second stage elevator raises are used in movements (like reef pickup, which require multi-stage elevator raises)
    public boolean elevatorFirstStage() {
        elevatorSubsystem.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback);
        return true;
    }

    public boolean elevatorSecondStage() {
        elevatorSubsystem.moveElevatorNormalSpeed(positions.secondStageElevatorPosition, subsystemCallback);
        return true;
    }

    /*
     * ARM OPERATIONAL METHODS
     * Note: these are general methods shared by multiple sequences, use care when updating and understand what the impact
     * will be in other sequences. If you need something custom for a specific sequence, spin off a separate method.
     */

    public boolean moveArm() {
        armSubsystem.moveArmNormalSpeed(positions.firstStageArmPosition, subsystemCallback);
        return true;
    }

    public boolean moveArmWithThreshold() {
        armSubsystem.moveArmNormalSpeed(positions.firstStageArmPosition, subsystemCallback, positions.firstStageArmThreshold);
        return true;
    }

    public boolean moveArmSlowly() {
        armSubsystem.moveArmSlowSpeed(positions.firstStageArmPosition, subsystemCallback);
        return true;
    }

    public boolean armSecondStage() {
        armSubsystem.moveArmNormalSpeed(positions.secondStageArmPosition, subsystemCallback);
        return true;
    }

    public boolean moveArmHome() {
        armSubsystem.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
        return true;
    }

    public boolean moveArmHomeCoral() {
        if (SequenceManager.shouldPluckAlgae()) {
            armSubsystem.moveArmSlowAlgae(0.6, 5.0, subsystemCallback);
        } else
            armSubsystem.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
        return true;
    }

    public boolean moveArmHomeSlowly() {
        armSubsystem.moveArmSlowSpeed(ArmConstants.armHomePosition, subsystemCallback);
        return true;
    }

    /*
     * HAND/INTAKE OPERATIONAL METHODS
     * Note: these are general methods shared by multiple sequences, use care when updating and understand what the impact
     * will be in other sequences. If you need something custom for a specific sequence, spin off a separate method.
     */

    public boolean closeHandWithoutCallback() {
        handClamperSubsystem.close();
        return true;
    }

    public boolean openHand() {
        handClamperSubsystem.open(positions.clamperOpenPosition, subsystemCallback);
        return true;
    }

    public boolean closeHand() {
        handClamperSubsystem.close(subsystemCallback);
        return true;
    }
    
    public boolean prepareToIntake() {
        handClamperSubsystem.open(positions.clamperIntakePosition);
        if(currentSequence == Sequence.INTAKE_ALGAE_FLOOR) {
            handIntakeSubsystem.intakeWithCurrent();
        } else {
            handIntakeSubsystem.intake(
                currentGamePiece == GamePiece.CORAL? HandConstants.intakeCoralVelocity : HandConstants.intakeAlgaeVelocity, 
                subsystemCallback
            );
        }
        return true;
    }

    public boolean stopIntaking() {
        handClamperSubsystem.close();
        handIntakeSubsystem.stop(subsystemCallback);
        return true;
    }

    /*
     * CORAL-SPECIFIC OPERATIONAL METHODS
     * Note: these methods are specific to certain parts of sequences and should only be updated when updating 
     * those specific sequences.
     */

    public boolean coralTimedIntake() {
        armSecondStage();
        handIntakeSubsystem.timedPieceDetection(2, subsystemCallback);
        return true;
    }

    public boolean holdCoralPiece() {
        System.out.println("SequenceStateMachine: holding coral piece...");
        handIntakeSubsystem.stop();
        handClamperSubsystem.holdCoral();
        return true;
    }

    public boolean releaseCoralPiece() {
        System.out.println("SequenceStateMachine: releasing coral piece...");
        handClamperSubsystem.open(0.013);  
        handIntakeSubsystem.release(HandConstants.releaseVelocity, 1.0, subsystemCallback);
        return true;
    }

    public boolean checkIfShouldScoreCoral() {
        // watch for the reef detection sensor to flip
        handIntakeSubsystem.watchForScoreDetection(subsystemCallback);
        return true;
    }

    public boolean moveArmToScoreCoral() {
        if(SequenceManager.shouldPluckAlgae()) {
            armSubsystem.moveArmNormalSpeed(positions.secondStageArmPosition, subsystemCallback);
            handClamperSubsystem.open(positions.clamperOpenPosition);
        } else {
            armSubsystem.moveArmNormalSpeed(positions.secondStageArmPosition, subsystemCallback);
        }
        return true;
    }

    public boolean resetHandIfCoralNotDetected() {
        if (SequenceManager.shouldPluckAlgae() == true) {
            handClamperSubsystem.open(0.05);
        } else if (!handIntakeSubsystem.pieceDetectionSwitchFlipped()) {
            handClamperSubsystem.close();
        }
        return true;
    }

    /*
     * ALGAE-SPECIFIC OPERATIONAL METHODS
     * Note: these methods are specific to certain parts of sequences and should only be updated when updating 
     * those specific sequences.
     */


     public boolean moveArmForBarge() {
        armSubsystem.moveArmNormalSpeed(positions.firstStageArmPosition, subsystemCallback);
        handIntakeSubsystem.release(HandConstants.releaseVelocity, 2.0, subsystemCallback);
        return true;
    }

    public boolean shootAlgaeInBarge() {
        handClamperSubsystem.close();
        handIntakeSubsystem.release(HandConstants.releaseVelocity, 0.1, subsystemCallback);
        return true;
    }

    public boolean pickupReefAlgae() {
        elevatorSubsystem.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback);
        return true;
    }

    public boolean grabAlgaeAndLower() {
        handClamperSubsystem.moveHand(positions.clamperHoldPosition);
        elevatorSubsystem.moveElevatorSlowSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback);
        return true;
    }

    public boolean handoffAlgae() {
        handClamperSubsystem.close();
        handIntakeSubsystem.release(HandConstants.releaseVelocity, HandConstants.defaultReleaseRuntime, subsystemCallback);
        return true;
    }

    public boolean algaeJiggle(){
        System.out.println("Jiggling algae");
        handClamperSubsystem.moveHand(positions.clamperJigglePosition);
        handIntakeSubsystem.releaseWithVelocity(5, 0.5, subsystemCallback);
        return true;
    }

    public boolean algaeIntake(){
        System.out.println("Intaking algae");
       // handIntakeSubsystem.stop();
       handIntakeSubsystem.intakeWithCurrent(); 
       resetState();
        return true;
    }

    public boolean algaeStopIntake(){
        handIntakeSubsystem.stop();
        return true;
    }

    /*
     * UPDATE LEVEL OPERATIONAL METHODS
     */

    // Drive the elevator to a new position when the operator overrides it midstream
    public boolean updateElevator() {
        // raise with no threshold b/c may have to move up or down, threshold potentially not valid
        elevatorSubsystem.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback);
        return true;
    }

    // Used to return the arm home (and stop intake) before driving the elevator to a new position
    public boolean returnArmForUpdate() {
        armSubsystem.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
        return true;
    }


    /*
     * RESET OPERATIONAL METHODS
     */

    public boolean startReset() {
        isResetting = true;
        // stop current movements
        armSubsystem.stopArm();
        elevatorSubsystem.stopElevator();
        // move back home
        armSubsystem.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
        elevatorSubsystem.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback);
        return true;
    }

    public boolean startIntakeReset() {
        if(!handIntakeSubsystem.pieceDetectionSwitchFlipped()) {
            handClamperSubsystem.close();
        }
        handIntakeSubsystem.stop();
        startReset();
        return true;
    }

    public boolean resetState() {
        if(SequenceManager.shouldPluckAlgae()){
            armSubsystem.moveArmSlowAlgae(0.1, -2.0, subsystemCallback);
        }
        currentSequence = null;
        currentAction = null;
        currentGamePiece = null;
        positions = null;
        isResetting = false;
        elevatorResetDone = false;
        armResetDone = false;
        processComplete();
        return true;
    }
}