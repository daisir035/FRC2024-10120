package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCmd extends SequentialCommandGroup {

    public AutoCmd(DriveSubsystem m_DriveSubsystem,String path){
    addCommands(
        new InstantCommand(() -> m_DriveSubsystem.setPose(
                m_DriveSubsystem.inversePose2dUsingAlliance(m_DriveSubsystem.generatePath(path).getPreviewStartingHolonomicPose(),
            DriverStation.Alliance.Blue))),
        m_DriveSubsystem.followPathCommand(m_DriveSubsystem.generatePath(path))
        );
    }

}