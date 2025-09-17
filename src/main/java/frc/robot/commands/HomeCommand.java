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
import frc.robot.subsystems.hand.HandIntakeSubsystem;

public class HomeCommand extends Command{
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private HandClamperSubsystem hand;
    private HandIntakeSubsystem intake;
    private boolean doneScoring = false;

    public HomeCommand(ArmSubsystem arm, ElevatorSubsystem elevator, HandClamperSubsystem hand, HandIntakeSubsystem intake) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.intake = intake;

        addRequirements(arm, elevator, hand);

    }

    public void execute(){

        elevator.moveElevatorNormalSpeed(ElevatorConstants.elevatorHomePosition);
        arm.moveArmNormalSpeed(ArmConstants.armHomePosition);
        hand.close();
        intake.stop();

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
