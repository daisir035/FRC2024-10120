package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanID;
import frc.robot.Constants.IntakeConstant;
import frc.robot.subsystems.FeedingSubsystem.FeedingState;

public class IntakeSubsystem extends SubsystemBase {
  private TalonFX intakeMotor = new TalonFX(CanID.intakeID);
 
  private TalonFXConfiguration intakeconfig = new TalonFXConfiguration();
  private VelocityVoltage intakeVol = new VelocityVoltage(0);

  FeedingSubsystem m_FeedingSubsystem;

  public IntakeSubsystem(FeedingSubsystem feedingSubsystem) {
    this.m_FeedingSubsystem = feedingSubsystem;

    intakeconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    intakeconfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    intakeconfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    intakeconfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    intakeconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

    intakeconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    intakeconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    intakeconfig.Slot0.kP = IntakeConstant.KP;
    intakeconfig.Slot0.kI = IntakeConstant.KI;
    intakeconfig.Slot0.kD = IntakeConstant.KD;
    intakeconfig.Slot0.kS = IntakeConstant.KS;
    intakeconfig.Slot0.kV = IntakeConstant.KV;
    intakeconfig.Slot0.kA = IntakeConstant.KA;

    intakeMotor.getConfigurator().apply(intakeconfig);

  }

  public enum IntakeState{
    Stop,
    Spit,
    Suction,
    Auto
  }

  public IntakeState intakeState = IntakeState.Stop;

  public void update(){
    this.update(this.intakeState);
  }

   public void update(IntakeState state){
    this.intakeState = state;
    switch (state) {
      case Stop:
        stopMotor();
        this.m_FeedingSubsystem.stopMotor();
      break;

      case Spit:
        setVelocity(IntakeConstant.spitVel);
        this.m_FeedingSubsystem.update(FeedingState.Spit);
        break;

      case Suction:
        if (!this.m_FeedingSubsystem.getDownOptoeSwitch()) {
          setVelocity(IntakeConstant.suctionVel);
        }else{
          stopMotor();
        }
        this.m_FeedingSubsystem.update(FeedingState.Suction);
        break;

      case Auto:
        setVelocity(IntakeConstant.autoVel);
        this.m_FeedingSubsystem.update(FeedingState.Auto);
        break;

      default:
        break;
  }
}

  public void setVelocity(double velocity){
    intakeVol.Velocity = velocity;
    intakeMotor.setControl(intakeVol);
  }

  public void stopMotor(){
    intakeMotor.stopMotor();
  }

  public double getVelocity(){
    return intakeMotor.getVelocity().getValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeVelocity", getVelocity());
  }

}
