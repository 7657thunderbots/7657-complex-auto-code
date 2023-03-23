 package frc.robot;
 import org.opencv.core.Mat;

import edu.wpi.first.wpilibj.Timer;
public class Auto2_balance{
    final double akP = 0.5;
     final double akI = 0.05;
  final double akD = 0.0;
  final double aiLimit = 0;
    public double setpoint=0;
    private double errorSum = 0;
    private double lastError=0;
     private double lastTimestamp = 0;
    public boolean onchargestation=false;
     private Balancing balancing;
     private final Timer timer = new Timer();
     double dsensorPosition=0;
    //private turnadjust turnadjust;
    private boolean placed = false;
    //private DriveTrain drivetrain;
    public double outputSpeed;
    private Robot robot;
  
     public Auto2_balance(){
    timer.reset();
    timer.start();
    placed=false;
    
    robot = new Robot();
     }
     public void Run_Auto2_balance(){
      //turnadjust.turnadjust_run();
         // if (timer.get()<2){
         // balancing.Speedvar=.2;
         placed = false;
         if (placed==false){
         // get sensor position
        // dsensorPosition = robot.drivetrain.getAverageEncoderDistance();
        // calculations
        double error = setpoint -  robot.drivetrain.avencoder;
        double dt = Timer.getFPGATimestamp() - lastTimestamp;

        if (Math.abs(error) < aiLimit) {
          errorSum += error * dt;
        }

        double errorRate = (error - lastError) / dt;

         outputSpeed = akP * error + akI * errorSum + akD * errorRate;

        // output to motors
       double Speedvar = outputSpeed;
       double directionL= Speedvar;
       double directionR= Speedvar;

      robot.drivetrain.tankDrive (directionL,directionR, false);

        // update last- variables
        lastTimestamp = Timer.getFPGATimestamp();
        lastError = error;
        // if ((dsensorPosition<.1)){
        //   placed=true;
        //   errorSum = 0;
        //   lastTimestamp = 0;
        //   lastError = 0;

        //  }
      }
     
    else {
    //balancing.BalancingRun();
   }
  
}
}
