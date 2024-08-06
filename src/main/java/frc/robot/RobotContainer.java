package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.DriveSubsystem.DriveState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.ShootingSubsystem.ShootingState;

public class RobotContainer {
    /* Subsystems */
    private  LEDSubsystem m_LedSubsystem = new LEDSubsystem();
    private  LimeLightSubsystem m_LimeLightSubsystem = new LimeLightSubsystem();
    private  DriveSubsystem s_Swerve = new DriveSubsystem(m_LimeLightSubsystem);
    private  FeedingSubsystem m_FeedingSubsystem = new FeedingSubsystem();
    private  IntakeSubsystem m_IntakeSubsystem  = new IntakeSubsystem(m_FeedingSubsystem);
    public  ShootingSubsystem m_ShootingSubsystem = new ShootingSubsystem(m_FeedingSubsystem,m_LimeLightSubsystem,m_LedSubsystem);
    private  ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem(m_LedSubsystem);


    /* Controllers */
    private  PS5Controller m_stick1 = new PS5Controller(0);
    private  PS5Controller m_stick2 = new PS5Controller(1);

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(m_stick1, 13);

    private final JoystickButton robotCentric = new JoystickButton(m_stick2,13);

    //Intake
    private final JoystickButton spit = new JoystickButton(m_stick1, 4);
    private final JoystickButton suction = new JoystickButton(m_stick1, 6);

     //Shooting
    private final JoystickButton hightPositin = new JoystickButton(m_stick2, 3);
    private final JoystickButton autoShooting = new JoystickButton(m_stick1, 5);
    private final JoystickButton amp = new JoystickButton(m_stick1, 8);
    private final JoystickButton pass = new JoystickButton(m_stick1, 7);
    private final JoystickButton source = new JoystickButton(m_stick1, 2);

    private final JoystickButton lowPositin = new JoystickButton(m_stick2, 1);


    //Elevator
    private final POVButton up = new POVButton(m_stick2, 270);
    private final POVButton release = new POVButton(m_stick2, 90);

    private final POVButton testUp = new POVButton(m_stick2, 0);
    private final POVButton testRelease = new POVButton(m_stick2, 180);
    private final JoystickButton lock = new JoystickButton(m_stick2, 10);
    private final JoystickButton lockrelease = new JoystickButton(m_stick2, 9);

    private  List<String> autonomousModes=List.of("1","1+3+1","1+2");
    private SendableChooser<Command> chooser;


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        NamedCommands.registerCommand("intake",new InstantCommand(() -> {                        
            new IntakeCmd(m_IntakeSubsystem, IntakeState.Suction);
          }));
    
        NamedCommands.registerCommand("shooting",new InstantCommand(() -> {                        
            new ShootingCmd(m_ShootingSubsystem, ShootingState.Auto);
          }));
        NamedCommands.registerCommand("preparing",new InstantCommand(() -> {                        
            new ShootingCmd(m_ShootingSubsystem, ShootingState.Preparing);
          }));
        NamedCommands.registerCommand("stop",new InstantCommand(() -> {                        
            new ShootingCmd(m_ShootingSubsystem, ShootingState.Stop);
            new IntakeCmd(m_IntakeSubsystem, IntakeState.Suction);
          }));
    

        s_Swerve.setDefaultCommand(
            new DriveCmd(
                s_Swerve, 
                () -> -m_stick1.getRawAxis(Constants.OIConstants.kDriverYAxis), 
                () -> -m_stick1.getRawAxis(Constants.OIConstants.kDriverXAxis), 
                () -> -m_stick1.getRawAxis(Constants.OIConstants.kDriverRotAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();

        chooser=new SendableChooser<>();
        for (String mode : autonomousModes){
        chooser.addOption(mode, new AutoCmd(s_Swerve,mode));
        }
        SmartDashboard.putData("Auto Selector", chooser);
    }

     public void teleopInit(){
      CommandScheduler.getInstance().schedule(                      
        new IntakeCmd(m_IntakeSubsystem, IntakeState.Stop),
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Stop)
      );
    }

    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        // Intake
        spit.onTrue(new InstantCommand(()->{
            new IntakeCmd(m_IntakeSubsystem, IntakeState.Spit);
        })).toggleOnFalse(new InstantCommand(()->{
            new IntakeCmd(m_IntakeSubsystem, IntakeState.Stop);
        }));
      
          suction.onTrue(new InstantCommand(()->{
          new IntakeCmd(m_IntakeSubsystem, IntakeState.Suction);
        })).toggleOnFalse(new InstantCommand(()->{
          new IntakeCmd(m_IntakeSubsystem, IntakeState.Stop);
        }));  
        

    //Shooting
        lowPositin.onTrue(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Lowposition);
      })).toggleOnFalse(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Default);
      }));

        hightPositin.onTrue(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Hightposition);
      })).toggleOnFalse(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Default);
      }));

        autoShooting.onTrue(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.AutoShooting);
        // s_Swerve.update(DriveState.SPEAKER);
      })).toggleOnFalse(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Default);
        // s_Swerve.update(DriveState.TELEOP);
      }));

        amp.onTrue(new InstantCommand(()->{
        // s_Swerve.update(DriveState.AMP);
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Amp);
      })).toggleOnFalse(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Default);
        //  s_Swerve.update(DriveState.TELEOP);
      }));

      source.onTrue(new InstantCommand(()->{
        // s_Swerve.update(DriveState.SOURCE);
      })).toggleOnFalse(new InstantCommand(()->{
        //  s_Swerve.update(DriveState.TELEOP);
      }));
        
        pass.onTrue(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Pass);
        // s_Swerve.update(DriveState.PASSING);
      })).toggleOnFalse(new InstantCommand(()->{
        new ShootingCmd(m_ShootingSubsystem, ShootingState.Default);
        // s_Swerve.update(DriveState.TELEOP);
      }));


    //Elevator
    testUp.onTrue(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.TestUp);
    })).toggleOnFalse(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Stop);
    }));

    testRelease.onTrue(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.TestRelease);
    })).toggleOnFalse(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Stop);
    }));

    up.onTrue(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Up);
    })).toggleOnFalse(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Up);
    }));

    release.onTrue(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Release);
    })).toggleOnFalse(new InstantCommand(()->{
      new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Release);
    }));

    lock.onTrue(new InstantCommand(()->{
        new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Lock);
      })).toggleOnFalse(new InstantCommand(()->{
        new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Lock);
        new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Stop);
      }));

    lockrelease.onTrue(new InstantCommand(()->{
        new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.LockRelease);
      })).toggleOnFalse(new InstantCommand(()->{
        new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.LockRelease);
        new ElevatorCmd(m_ElevatorSubsystem, ElevatorState.Stop);
      }));
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
