package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.KalmanFilter;
import frc.robot.Constants;
import frc.robot.Constants.LimelightConstant;
import frc.lib.util.KalmanFilter_s;

public class LimeLightSubsystem extends SubsystemBase {
  NetworkTable limelight;

  public boolean limelightDetected = false;
  public boolean detectorDetected = false;

  public double x;
  public double y;
  public double area;
  public double v;

  public double apriltagHeight = LimelightConstant.apriltagHeight;
  public double cameraHeight = LimelightConstant.cameraHeight;
  public double rotationPointHeight = LimelightConstant.rotationPointHeight;
  public double spearkerHeight = LimelightConstant.spearkerHeight;

  public double rotationToCameraXdistance = LimelightConstant.rotationToCameraXdistance;

  public double limelightOffsetAngle = LimelightConstant.limelightOffsetAngle;
  public double rotationPointOffsetAngle = LimelightConstant.rotationPointOffsetAngle;

  public double limelightTargetSpeed;
  public double errolVel;
  
  public static LimeLightSubsystem instance;
  public static synchronized LimeLightSubsystem getInstance() {
    if (instance == null) {
      instance = new LimeLightSubsystem();
    }
    return instance;
  }

  public LimeLightSubsystem() {
   limelight = NetworkTableInstance.getDefault().getTable(Constants.LimelightConstant.tagLimelight);
  }

  public void LimelightReadValue(){
    // NetworkTableEntry targetpose = table.getEntry("targetpose_cameraspace");
    // double[] targetpose_arr = targetpose.getDoubleArray(new double[]{0.0,0.0,0.0,0.0,0.0});
    // Double tx = targetpose_arr[0];
    // Double ty = targetpose_arr[1];
    // Double tz = targetpose_arr[2];
    // Double rx = targetpose_arr[3];
    // Double ry = targetpose_arr[4];
    // Double rz = targetpose_arr[5];
  NetworkTableEntry tx = limelight.getEntry("tx");
  NetworkTableEntry ty = limelight.getEntry("ty");
  NetworkTableEntry ta = limelight.getEntry("ta");
  NetworkTableEntry tv = limelight.getEntry("tv");

  //read values periodically
  x = tx.getDouble(0.0);
  y = ty.getDouble(0.0);
  area = ta.getDouble(0.0);
  v= tv.getInteger(0);
  

  //post to smart dashboard periodically
  SmartDashboard.putNumber("LimelightX", x);
  SmartDashboard.putNumber("LimelightY", y);
  SmartDashboard.putNumber("LimelightArea", area);
  }

  KalmanFilter_s degreefilter = new KalmanFilter_s(0, 0.05, 0.01);

  public void setLimelightfocus(){
    Number[] ns = new Number[4];
    ns[0] = 0.5;
    ns[1] = 1;
    ns[2] = 1;
    ns[3] = 1;
    limelight.getEntry("crop").setNumberArray(ns);
  }

  
  public double getCamearXdegree() {
    x =  limelight.getEntry("tx").getDouble(0.0);
    return x;
  }
  public double getrawwCamearYdegree() {
    y =  limelight.getEntry("ty").getDouble(0.0);
    return y;
  }
  public double getApril() {
    return limelight.getEntry("ta").getDouble(0.0);
  }
  
  //Calculate the distance from the camera in the direction of Apriltag'Xdirection
  //Unit: meter
  public double getCamearXDistance() {
    y =  limelight.getEntry("ty").getDouble(0.0);
    x=limelight.getEntry("tx").getDouble(0.0);
    double yawAngle=x*Math.PI/180;
    double pitchAngle = (y+limelightOffsetAngle)*(Math.PI/180);
    double carmearToApriltagHeight = apriltagHeight-cameraHeight;
    double carmearXdistance = carmearToApriltagHeight/Math.tan(pitchAngle);
    // double errorDistance = LimelightConstant.errorDistance_k*carmearXdistance+LimelightConstant.errorDistance_b;
    // carmearXdistance = carmearXdistance+errorDistance;
    double distance=Math.abs(carmearXdistance/Math.cos(yawAngle));
    return distance;
  }

  public double getArmRatation(){
    double rotationToSpeakerXDistance = getCamearXDistance()+rotationToCameraXdistance;
    double rotationToSpeakerHeight = spearkerHeight-rotationPointHeight;
    double totalrotationRadian = Math.atan(rotationToSpeakerHeight/rotationToSpeakerXDistance);
    double armRatation = totalrotationRadian-(rotationPointOffsetAngle*(Math.PI/180));
    // double errorRatation = LimelightConstant.errorratation_k*armRatation+LimelightConstant.errorratation_b;
    // armRatation = armRatation+errorRatation;
    return armRatation;
  }



  @Override
  public void periodic() {
  }
}

