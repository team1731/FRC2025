package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;

/**
 * Wrapper command that would add a timeout to the indicated command if in simulation
 */
public class SimulatedCommand extends WrapperCommand {
    public SimulatedCommand(Command commandToWrap, double simTime) {
        super(
            Robot.isSimulation() ? commandToWrap.withTimeout(simTime) : commandToWrap
        );
    }
}