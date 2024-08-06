package frc.robot.subsystems;


import javax.xml.transform.Source;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanID;
import frc.robot.Constants.FeedingConstant;
import frc.robot.Constants.FeedingConstant;

public class FeedingSubsystem extends SubsystemBase {
  private TalonFX feedingMotor = new TalonFX(CanID.feederID);

  private TalonFXConfiguration feedingconfig = new TalonFXConfiguration();
  private VelocityVoltage feedingVol = new VelocityVoltage(0);

  private DigitalInput up_OptoeSwitch = new DigitalInput(CanID.up_OptoeSwitchID);
  private DigitalInput down_OptoeSwitch = new DigitalInput(CanID.down_OptoeSwitchID);

  public FeedingSubsystem() {
    feedingconfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feedingconfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feedingconfig.MotorOutput.DutyCycleNeutralDeadband = 0.04;

    feedingconfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    feedingconfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    feedingconfig.CurrentLimits.SupplyCurrentThreshold = 60.0;
    feedingconfig.CurrentLimits.SupplyTimeThreshold = 0.02;

    feedingconfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    feedingconfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

    feedingconfig.Slot0.kP = FeedingConstant.KP;
    feedingconfig.Slot0.kI = FeedingConstant.KI;
    feedingconfig.Slot0.kD = FeedingConstant.KD;
    feedingconfig.Slot0.kS = FeedingConstant.KS;
    feedingconfig.Slot0.kV = FeedingConstant.KV;
    feedingconfig.Slot0.kA = FeedingConstant.KA;

    feedingMotor.getConfigurator().apply(feedingconfig);
  }

  public enum FeedingState{
    Stop,
    Spit,
    Suction,
    Shooting,
    Auto
  }

  public FeedingState feedingState = FeedingState.Stop;

  public void update(){
    this.update(this.feedingState);
  }

  public void update(FeedingState state){
    this.feedingState = state;
    switch (state) {
      case Stop:
        stopMotor();
        break;

      case Spit:
          setVelocity(FeedingConstant.spitVel);
        break;

      case Suction:
        if(getDownOptoeSwitch()&&getUpOptoeSwitch()){
          stopMotor();
        }else{
          setVelocity(FeedingConstant.suctionVel);
        }
        break;
    
      case Shooting:
        setVelocity(FeedingConstant.shootingVel);
      break;

      case Auto:
        setVelocity(FeedingConstant.autoVel);
        break;

      default:
        break;
    }
  }

  public void setVelocity(double velocity){
    feedingVol.Velocity = velocity;
    feedingMotor.setControl(feedingVol);
  }

  public void stopMotor(){
    feedingMotor.stopMotor();
  }

  public double getVelocity(){
    return feedingMotor.getVelocity().getValue();
  }

  public boolean getUpOptoeSwitch(){
    return !up_OptoeSwitch.get();
  }

  public boolean getDownOptoeSwitch(){
    return !down_OptoeSwitch.get();
  }

  @Override
  public void periodic() {
    //  SmartDashboard.putNumber("feedingVelocity", getVelocity());
    //  SmartDashboard.putBoolean("UpSwitch", getUpOptoeSwitch());
    //  SmartDashboard.putBoolean("DownSwitch", getDownOptoeSwitch());
  }
}

