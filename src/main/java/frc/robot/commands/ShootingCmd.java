package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootingSubsystem;
import frc.robot.subsystems.ShootingSubsystem.ShootingState;

public class ShootingCmd extends Command {
  private ShootingSubsystem m_ShootingSubsystem;
  public ShootingCmd(ShootingSubsystem shootingSubsystem,ShootingState state) {
    this.m_ShootingSubsystem = shootingSubsystem;
    this.m_ShootingSubsystem.shootingState = state;
    addRequirements(shootingSubsystem);
    schedule();
  }

  @Override
  public void execute() {
    m_ShootingSubsystem.update();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
