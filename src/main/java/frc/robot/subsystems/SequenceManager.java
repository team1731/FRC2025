package frc.robot.subsystems;

import frc.robot.subsystems.arm.NEW_ArmSubsystem;
import frc.robot.subsystems.elevator.NEW_ElevatorSubsystem;
import frc.robot.subsystems.hand.NEW_HandClamperSubsystem;
import frc.robot.subsystems.hand.NEW_HandIntakeSubsystem;

public class SequenceManager {
    private NEW_ArmSubsystem arm;
    private NEW_ElevatorSubsystem elevator;
    private NEW_HandClamperSubsystem hand;
    private NEW_HandIntakeSubsystem intake;

    public SequenceManager(NEW_ArmSubsystem arm, NEW_ElevatorSubsystem elevator, NEW_HandClamperSubsystem hand, NEW_HandIntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.intake = intake;
    }

    // public boolean raiseElevatorAndArmForBarge(Positions positions) {
    //     elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition, positions.raiseElevatorThreshold);
    //     arm.moveArmSlowSpeed(-8.0);
    //     return true;
    // }

    // public boolean raiseElevatorAndArmForL1(Positions positions) {
    //     elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback, positions.raiseElevatorThreshold);
    //     arm.moveArmSlowSpeed(positions.firstStageArmPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean raiseElevator(Positions positions) {
    //     elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback, positions.raiseElevatorThreshold);
    //     return true;
    // }

    // public boolean moveElevatorHome(Positions positions) {
    //     isResetting = true;
    //     elevator.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback, positions.lowerElevatorThreshold);
    //     return true;
    // }

    // // Note: first and second stage elevator raises are used in movements (like reef pickup, which require multi-stage elevator raises)
    // public boolean elevatorFirstStage(Positions positions) {
    //     elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean elevatorSecondStage() {
    //     elevator.moveElevatorNormalSpeed(positions.secondStageElevatorPosition, subsystemCallback);
    //     return true;
    // }

    // /*
    //  * ARM OPERATIONAL METHODS
    //  * Note: these are general methods shared by multiple sequences, use care when updating and understand what the impact
    //  * will be in other sequences. If you need something custom for a specific sequence, spin off a separate method.
    //  */

    // public boolean moveArm() {
    //     arm.moveArmNormalSpeed(positions.firstStageArmPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmWithThreshold() {
    //     arm.moveArmNormalSpeed(positions.firstStageArmPosition, subsystemCallback, positions.firstStageArmThreshold);
    //     return true;
    // }

    // public boolean moveArmSlowly() {
    //     arm.moveArmSlowSpeed(positions.firstStageArmPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean armSecondStage() {
    //     arm.moveArmNormalSpeed(positions.secondStageArmPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmHome() {
    //     arm.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmHomeCoral() {
    //     arm.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmHomeSlowly() {
    //     arm.moveArmSlowSpeed(ArmConstants.armHomePosition, subsystemCallback);
    //     return true;
    // }

    // /*
    //  * HAND/INTAKE OPERATIONAL METHODS
    //  * Note: these are general methods shared by multiple sequences, use care when updating and understand what the impact
    //  * will be in other sequences. If you need something custom for a specific sequence, spin off a separate method.
    //  */

    // public boolean closeHandWithoutCallback() {
    //     handClamperSubsystem.close();
    //     return true;
    // }

    // public boolean openHand() {
    //     handClamperSubsystem.open(positions.clamperOpenPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean closeHand() {
    //     handClamperSubsystem.close(subsystemCallback);
    //     return true;
    // }
    
    // public boolean prepareToIntake() {
    //     handClamperSubsystem.open(positions.clamperIntakePosition);
    //     if(currentSequence == Sequence.INTAKE_ALGAE_FLOOR) {
    //         handIntakeSubsystem.intakeWithCurrent();
    //     } else {
    //         handIntakeSubsystem.intake(
    //             currentGamePiece == GamePiece.CORAL? HandConstants.intakeCoralVelocity : HandConstants.intakeAlgaeVelocity, 
    //             subsystemCallback
    //         );
    //     }
    //     return true;
    // }

    // public boolean stopIntaking() {
    //     handClamperSubsystem.close();
    //     handIntakeSubsystem.stop(subsystemCallback);
    //     return true;
    // }

    // /*
    //  * CORAL-SPECIFIC OPERATIONAL METHODS
    //  * Note: these methods are specific to certain parts of sequences and should only be updated when updating 
    //  * those specific sequences.
    //  */

    // public boolean coralTimedIntake() {
    //     armSecondStage();
    //     handIntakeSubsystem.timedPieceDetection(2, subsystemCallback);
    //     return true;
    // }

    // public boolean holdCoralPiece() {
    //     System.out.println("SequenceStateMachine: holding coral piece...");
    //     handIntakeSubsystem.stop();
    //     handClamperSubsystem.holdCoral();
    //     return true;
    // }

    // public boolean releaseCoralPiece() {
    //     System.out.println("SequenceStateMachine: releasing coral piece...");
    //     handClamperSubsystem.open(0.013);  
    //     handIntakeSubsystem.releaseWithVelocity(10, 1.0, subsystemCallback);
    //     elevator.moveElevatorNormalSpeed(15);
    //     return true;
    // }

    // public boolean checkIfShouldScoreCoral() {
    //     // watch for the reef detection sensor to flip
    //     handIntakeSubsystem.watchForScoreDetection(subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmToScoreCoral() {
    //     if(SequenceManager.shouldPluckAlgae()) {
    //         arm.moveArmSlowSpeed(positions.secondStageArmPosition, subsystemCallback);
    //         //handClamperSubsystem.open(positions.clamperOpenPosition);
    //     } else {
    //         arm.moveArmNormalSpeed(positions.secondStageArmPosition, subsystemCallback);
    //     }
    //     return true;
    // }

    // public boolean resetHandIfCoralNotDetected() {
    //     if (SequenceManager.shouldPluckAlgae() == true) {
    //         handClamperSubsystem.open(0.05);
    //     } else if (!handIntakeSubsystem.pieceDetectionSwitchFlipped()) {
    //         handClamperSubsystem.close();
    //     }
    //     return true;
    // }

    // /*
    //  * ALGAE-SPECIFIC OPERATIONAL METHODS
    //  * Note: these methods are specific to certain parts of sequences and should only be updated when updating 
    //  * those specific sequences.
    //  */


    //  public boolean moveArmForBarge() {
    //     arm.moveArmNormalSpeed(positions.firstStageArmPosition, subsystemCallback);
    //     handIntakeSubsystem.release(HandConstants.releaseVelocity, 2.0, subsystemCallback);
    //     return true;
    // }

    // public boolean shootAlgaeInBarge() {
    //   //  handClamperSubsystem.close();
    //     handClamperSubsystem.moveHand(positions.clamperJigglePosition);
    //     handIntakeSubsystem.release(HandConstants.releaseVelocity, 0.1, subsystemCallback);
    //     return true;
    // }

    // public boolean pickupReefAlgae() {
    //     elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean grabAlgaeAndLower() {
    //     handClamperSubsystem.moveHand(positions.clamperHoldPosition);
    //     elevator.moveElevatorSlowSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback);
    //     return true;
    // }

    // public boolean handoffAlgae() {
    //     handClamperSubsystem.close();
    //     handIntakeSubsystem.release(HandConstants.releaseVelocity, HandConstants.defaultReleaseRuntime, subsystemCallback);
    //     return true;
    // }

    // public boolean pluckAlgae() {
    //     handIntakeSubsystem.intakeWithCurrent();
    //     arm.moveArmSlowAlgae(0.3, 5.0, 1, subsystemCallback);
    //     return true;
    // }

    // public boolean moveArmForAlgaeJiggle() {
    //     arm.moveArmSlowSpeed(positions.thirdStageArmPosition, subsystemCallback);
    //     return true;
    // }

    // public boolean algaeJiggle(){
    //     System.out.println("Jiggling algae");
    //     handClamperSubsystem.moveHand(positions.clamperJigglePosition);
    //     handIntakeSubsystem.releaseWithVelocity(5, 0.2, subsystemCallback);
    //     return true;
    // }

    // public boolean algaeIntake(){
    //     System.out.println("Intaking algae");
    //     // handIntakeSubsystem.stop();
    //     handIntakeSubsystem.intakeWithCurrent(); 
    //     resetState();
    //     return true;
    // }

    // public boolean algaeStopIntake(){
    //     handIntakeSubsystem.stop();
    //     return true;
    // }

    // /*
    //  * UPDATE LEVEL OPERATIONAL METHODS
    //  */

    // // Drive the elevator to a new position when the operator overrides it midstream
    // public boolean updateElevator() {
    //     // raise with no threshold b/c may have to move up or down, threshold potentially not valid
    //     elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition, subsystemCallback);
    //     return true;
    // }

    // // Used to return the arm home (and stop intake) before driving the elevator to a new position
    // public boolean returnArmForUpdate() {
    //     arm.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
    //     return true;
    // }


    // /*
    //  * RESET OPERATIONAL METHODS
    //  */

    // public boolean startReset() {
    //     isResetting = true;
    //     // stop current movements
    //     arm.stopArm();
    //     elevator.stopElevator();
    //     // move back home
    //     arm.moveArmNormalSpeed(ArmConstants.armHomePosition, subsystemCallback);
    //     elevator.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback);
    //     return true;
    // }

    // public boolean startResetSlowly() {
    //     isResetting = true;
    //     // stop current movements
    //     arm.stopArm();
    //     elevator.stopElevator();
    //     // move back home slowly
    //     arm.moveArmSlowSpeed(ArmConstants.armHomePosition, subsystemCallback);
    //     elevator.moveElevatorSlowSpeed(ElevatorConstants.elevatorHomePosition, subsystemCallback);
    //     return true;
    // }

    // public boolean startIntakeResetSlow() {
    //     if(!handIntakeSubsystem.pieceDetectionSwitchFlipped()) {
    //         handClamperSubsystem.close();
    //     }
    //     handIntakeSubsystem.stop();
    //     startResetSlowly();
    //     return true;
    // }
    
    // public boolean startIntakeReset() {
    //     if(!handIntakeSubsystem.pieceDetectionSwitchFlipped()) {
    //         handClamperSubsystem.close();
    //     }
    //     handIntakeSubsystem.stop();
    //     startReset();
    //     return true;
    // }

    // public boolean resetState() {
    //     currentSequence = null;
    //     currentAction = null;
    //     currentGamePiece = null;
    //     positions = null;
    //     isResetting = false;
    //     elevatorResetDone = false;
    //     armResetDone = false;
    //     processComplete();
    //     return true;
    // }
}