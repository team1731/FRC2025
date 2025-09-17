package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.state.sequencer.Level;
import frc.robot.state.sequencer.positions.Positions;
import frc.robot.state.sequencer.positions.PositionsFactory;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.hand.HandClamperSubsystem;

public class PrepCoralScoreCommand extends Command{
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private HandClamperSubsystem hand;
    private CommandXboxController controller;
    private GenericHID opController;
    private Positions positions;
    private Level targetLevel;

    public PrepCoralScoreCommand(ArmSubsystem arm, ElevatorSubsystem elevator, HandClamperSubsystem hand, CommandXboxController controller, GenericHID opController) {
        this.arm = arm;
        this.elevator = elevator;
        this.hand = hand;
        this.controller = controller;
        this.opController = opController;

        addRequirements(arm, elevator, hand);

    }

    public void execute(){
        if (controller.x().getAsBoolean()){
            targetLevel = Level.L1;
            positions = PositionsFactory.getCoralScoreL1Positions();
        }
        else if (controller.a().getAsBoolean()){
            targetLevel = Level.L2;
            positions = PositionsFactory.getCoralScoreL2Positions();
        }
        else if (controller.b().getAsBoolean()){
            targetLevel = Level.L3;
            positions = PositionsFactory.getCoralScoreL3Positions();
        }
        else {
            targetLevel = Level.L4;
            positions = PositionsFactory.getCoralScoreL4Positions();
        }
        elevator.moveElevatorNormalSpeed(positions.raiseElevatorPosition);
        if (elevator.getElevatorPosition() > positions.raiseElevatorThreshold){
            arm.moveArmNormalSpeed(positions.firstStageArmPosition);
        }
        else {
            arm.moveArmSlowSpeed(ArmConstants.armHomePosition);
        }
        if (elevator.isAtPosition(positions.raiseElevatorPosition) && arm.isAtPosition(positions.firstStageArmPosition)){
            SubsystemManager.setAtScorePosition(true, positions, targetLevel);
        }
        else {
            SubsystemManager.setAtScorePosition(false, positions, targetLevel);
        }
    }
}
