package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public class Balancing {
private DriveTrain driveTrain;
    
    public double Speedvar=0.0;
    
    public boolean chargestationbalance= false;
   
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    double berror=0;
    double errorRate=0;
   
    final double bkP = -0.008;
    final double bkI = -0.005;
    final double bkD = -0.001;
    final double biLimit = 3;

    public Balancing() {   

    }   


    public void BalancingRun(){
    //     driveTrain.run_drive();
    //     if (driveTrain.m_gyro.getYComplementaryAngle()<3 && driveTrain.m_gyro.getYComplementaryAngle()>-3){
    //         chargestationbalance=true;
    //         driveTrain.setbrake(true);
    //         Speedvar=0;}
        
    //         else {
    //             setpoint = 0;
        
    //             // get sensor position
    //             Double sensorPosition = driveTrain.m_gyro.getYComplementaryAngle();
        
    //             // calculations
    //             berror = setpoint - sensorPosition;
    //             double dt = Timer.getFPGATimestamp() - lastTimestamp;
        
    //             if (Math.abs(berror) < biLimit) {
    //             errorSum += berror * dt;
    //             }
        
    //             errorRate = (berror - lastError) / dt;
        
    //             Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
        
    //             // output to motors
    //             Speedvar=(outputSpeed);
        
    //             // update last- variables
    //             lastTimestamp = Timer.getFPGATimestamp();
    //             lastError = berror;
    //     }
    //             if (Speedvar>.2){
    //                 Speedvar=.2;
    //             }
                
    //             if (Speedvar<-.2){
    //                     Speedvar=-.2;
    //             }

        

     }
}
