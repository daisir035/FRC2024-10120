package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;

public class IntakeCmd extends Command {
  private IntakeSubsystem m_IntakeSubsystem;

  public IntakeCmd(IntakeSubsystem intakeSubsystem,IntakeState state) {
    this.m_IntakeSubsystem = intakeSubsystem;
    this.m_IntakeSubsystem.intakeState = state;
    addRequirements(intakeSubsystem);
    schedule();
  }

  @Override
  public void execute() {
    m_IntakeSubsystem.update();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}

