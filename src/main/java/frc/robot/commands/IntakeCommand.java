/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ISInput;
import frc.robot.IntakeShootStateMachine;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.WristSubsystem;

/**
 * Command to fire into the speaker
 */
public class IntakeCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final IntakeShootStateMachine m_intakeShootStateMachine;
    private final WristSubsystem m_wristSubsystem;

	/**
	 * Creates a new Fire into the speaker
	 *
	 * @param IntakeSubsystem     
	 * @param seqSubsystem        
	 *  
	 */
	public IntakeCommand(IntakeShootStateMachine intakeStateMachine, WristSubsystem wristSubsystem) {
		m_intakeShootStateMachine = intakeStateMachine;
		m_wristSubsystem = wristSubsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		if ( wristSubsystem != null) {
			addRequirements( wristSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	// If it is used as Default command then it gets call all the time
	@Override
	public void initialize() {
		m_wristSubsystem.retractTrapFlap();
		m_wristSubsystem.moveWrist(WristConstants.IntakePosition);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_intakeShootStateMachine.setCurrentInput(ISInput.START_INTAKE);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        m_intakeShootStateMachine.setCurrentInput(ISInput.STOP_INTAKE);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	
	}

}