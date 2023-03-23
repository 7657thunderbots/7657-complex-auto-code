package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.kauailabs.vmx.AHRSJNI;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class DriveTrain  {
  
  public final WPI_TalonFX leftParent; // = new WPI_TalonFX(4);
  public final WPI_TalonFX leftChild;// = new WPI_TalonFX(5);
  public final WPI_TalonFX rightParent;// = new WPI_TalonFX(3);
  public final WPI_TalonFX rightChild; // = new WPI_TalonFX(2);

  // The robot's drive
  private final DifferentialDrive m_drive;// = new DifferentialDrive(leftParent, rightParent);

  // private final DifferentialDrive tdrive;

  // The gyro sensor
   //public  ADIS16470_IMU m_gyro;// = new ADIS16470_IMU();
   public AHRS m_gyro;

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDriveKinematics m_kinematics;
  //private final ChassisSpeeds m_chassis;

  // These classes help us simulate our drivetrain
 // public DrivebaseSimFX m_drivetrainSimulator;
  private final Field2d m_field;

  // Talon FX Motor Setup
  /*
   * Talon FX has 2048 units per revolution
   * 
   * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
   * sensor-resolution
   */
  final int kUnitsPerRevolution = 2048; /* this is constant for Talon FX */
  private final int kCountsPerRev = 2048;  //Encoder counts per revolution of the motor shaft.
  private final double kSensorGearRatio = 8.45; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
  private final double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
  private final double kWheelRadiusInches = 3;
  private final int k100msPerSecond = 10;
  private final double chassiswheelbase = 0.546;

  /** electic brake during neutral */
  final NeutralMode kBrakeDurNeutral = NeutralMode.Coast;
  final NeutralMode kBrakeMode = NeutralMode.Brake;
  private final Pose2d start_pos = new Pose2d(1.5,6.5,Rotation2d.fromDegrees(0));

  public double avencoder=0;

  /** Creates a new DriveSubsystem. */
  public DriveTrain() {
    
    leftParent  = new WPI_TalonFX(4);
    leftChild   = new WPI_TalonFX(5);
    rightParent = new WPI_TalonFX(3);
    rightChild  = new WPI_TalonFX(2);
    m_drive     = new DifferentialDrive(leftParent, rightParent);
    //tdrive = new DifferentialDrive(leftParent,rightParent);
    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      //ahrs = new AHRS(SPI.Port.kMXP);
      m_gyro = new AHRS(SerialPort.Port.kUSB1);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
    
    // Configure Talon Motors
    rightParent.configFactoryDefault();
    leftParent.configFactoryDefault();
    leftChild.configFactoryDefault();
    rightChild.configFactoryDefault();

    rightParent.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
     rightParent.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    rightParent.setNeutralMode(kBrakeDurNeutral);

    rightChild.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightChild.follow(rightParent);
    rightChild.setNeutralMode(kBrakeDurNeutral);

    leftParent.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    leftParent.setInverted(true);
     leftParent.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20);
    leftParent.setNeutralMode(kBrakeDurNeutral);

    leftChild.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftChild.follow(leftParent);
    leftChild.setInverted(true);
    leftChild.setNeutralMode(kBrakeDurNeutral);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistance(),
        getRightDistance());
    
    m_kinematics = new DifferentialDriveKinematics(chassiswheelbase);

  

    m_field = new Field2d();
    m_field.setRobotPose(start_pos);
    SmartDashboard.putData("Field",m_field);

    if (RobotBase.isSimulation()) { // If our robot is simulated
      //The Robot is in Simulation.  Initialize sim code.
      // System.out.println("Simulation Mode Detected, Initializing simulated drivetrain");
      // //m_drivetrainSimulator = new DrivebaseSimFX(leftParent, rightParent, m_gyro);
      // PhysicsSim.getInstance().addTalonFX(leftParent, 0.5, 6800);
      // PhysicsSim.getInstance().addTalonFX(leftChild, 0.5, 6800);
      // PhysicsSim.getInstance().addTalonFX(rightParent, 0.5, 6800);
      // PhysicsSim.getInstance().addTalonFX(rightChild, 0.5, 6800);
    }
    
  }

  public void run_drive() {
    
   
    avencoder =  getAverageEncoderDistance();
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistance(),
        getRightDistance());
    
    //create a transform with the position change from 0,0    
    Transform2d trans = new Transform2d(new Pose2d(), m_odometry.getPoseMeters());
    
    m_field.setRobotPose(start_pos.plus(trans));
  }

public void setbrake (boolean enable){

  if (enable) { 
  leftChild.setNeutralMode(kBrakeDurNeutral);
  leftParent.setNeutralMode(kBrakeDurNeutral);
  rightChild.setNeutralMode(kBrakeDurNeutral);
  rightParent.setNeutralMode(kBrakeDurNeutral);
  }
  else{
    leftChild.setNeutralMode(kBrakeMode);
    leftParent.setNeutralMode(kBrakeMode);
    rightChild.setNeutralMode(kBrakeMode);
    rightParent.setNeutralMode(kBrakeMode);

  }
}
  public void run_sim() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and
    // gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    // PhysicsSim.getInstance().run();
    // m_drivetrainSimulator.run();
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION
   * ONLY! If you want
   * it to work elsewhere, use the code in
   * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return 0; // m_drivetrainSimulator.getCurrentDrawAmps();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftParent.getSelectedSensorVelocity(),
        rightParent.getSelectedSensorVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    //m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        getLeftDistance(),
        getRightDistance(),
        pose);

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using tankdrive controls.
   *
   * @param left  the commanded left side
   * @param right the commanded right side
   * @param b
   */
  public void tankDrive(double left, double right, boolean b) {

    m_drive.tankDrive(left, right, false);

  }
  public void mywatchdog() {

    m_drive.feedWatchdog();

  }

  public void tdrive(double left, double right, boolean b){
    m_drive.tankDrive(left, right, false);
  }
  

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDrisveVolts(double leftVolts, double rightVolts) {
    leftParent.setVoltage(leftVolts);
    rightParent.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftParent.setSelectedSensorPosition(0);
    rightParent.setSelectedSensorPosition(0);
    leftChild.setSelectedSensorPosition(0);
    rightChild.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getRightDistance() + getLeftDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
   // m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return 0;//m_gyro.getAngle();
  }

    /**
   * get the distance in meters of the left side
   *
   * @return the distance in meters of the left side
   */
  public double getLeftDistance() {
    return  nativeUnitsToDistanceMeters(leftParent.getSelectedSensorPosition());
  }

     /**
   * get the distance in meters of the right side
   *
   * @return the distance in meters of the right side
   */
  public double getRightDistance() {
    return  nativeUnitsToDistanceMeters(rightParent.getSelectedSensorPosition());
  }


  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
    return positionMeters;
  }
public void arcadeDrive(double d, double e, boolean b) {
}

}