package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.state.sequencer.positions.PositionsFactory;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;

public class CoralScoreCommand extends Command{
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private HandClamperSubsystem hand;
    private CommandXboxController controller;
    private GenericHID opController;
    private Positions positions;
    private boolean doneScoring = false;

    public CoralScoreCommand(ArmSubsystem arm, ElevatorSubsystem elevator, HandClamperSubsystem hand, CommandXboxController controller, GenericHID opController) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.controller = controller;
        this.opController = opController;

        addRequirements(arm, elevator, hand);

    }

    public void execute(){
        if (doneScoring == false){
            arm.moveArmNormalSpeed(SubsystemManager.getScorePosition().secondStageArmPosition);
        }
        else if (elevator.getElevatorPosition() < SubsystemManager.getScorePosition().lowerElevatorThreshold){
            arm.moveArmSlowSpeed(ArmConstants.armHomePosition);
        }
        if (arm.isAtPosition(SubsystemManager.getScorePosition().secondStageArmPosition) && doneScoring == false){
            elevator.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition);
            doneScoring = true;
        }

        // if (controller.getXButton()){
        //     positions = PositionsFactory.getCoralScoreL1Positions();
        // }
        // else if (controller.getAButton()){
        //     positions = PositionsFactory.getCoralScoreL2Positions();
        // }
        // else if (controller.getBButton()){
        //     positions = PositionsFactory.getCoralScoreL3Positions();
        // }
        // else {
        //     positions = PositionsFactory.getCoralScoreL4Positions();
        // }
        // elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition);
        // if (elevator.getElevatorPosition() > positions.raiseElevatorThreshold){
        //     arm.moveArmNormalSpeed(positions.firstStageArmPosition);
        // }
        // else {
        //     arm.moveArmSlowSpeed(ArmConstants.armHomePosition);
        // }
        // if (elevator.isAtPosition(positions.raiseElevatorPosition) && arm.isAtPosition(positions.firstStageArmPosition)){
        //     SubsystemManager.setAtScorePosition(true);
        // }
        // else {
        //     SubsystemManager.setAtScorePosition(false);
        // }

    }
}
