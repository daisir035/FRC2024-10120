
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;

public class ElevatorCmd extends Command {
  private ElevatorSubsystem m_ElevatorSubsystem;

  public ElevatorCmd(ElevatorSubsystem elevatorSubsystem,ElevatorState state) {
    this.m_ElevatorSubsystem = elevatorSubsystem;
    this.m_ElevatorSubsystem.elevatorState = state;
    addRequirements(elevatorSubsystem);
    schedule();
  }

 
  @Override
  public void execute() {
    m_ElevatorSubsystem.update();
  }

  @Override
  public boolean isFinished(){
    return false;
  }

}
