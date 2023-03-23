package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Auto1{
    private DriveTrain drivetrain;
    private Elbow elbow;
    private Wrist wrist;
    private Hand hand;
    private Shoulder shoulder;
    private Pneumatics pneumatics;

    //true or false values used
    private boolean start_position;
    private boolean ipotpp1;
    private boolean wip;
    private boolean wipc;
    private boolean rc1;
    private boolean rc2;
    private boolean backtolowerarm;
    private boolean armlowered;
    private boolean taxi;
    private boolean Balancing;
    
    private double dpos;
    private double dtmos;
    private double dtmot;
    private double direction;
    private double turndirection;
    private double wait;
    
    public Auto1(){

        // This works in tangent with the other files
        drivetrain = new DriveTrain();
        elbow = new Elbow();
        hand = new Hand();
        wrist = new Wrist();
        shoulder = new Shoulder();
        pneumatics = new Pneumatics();
        // this starts false to say we have not put up our arm yet
        start_position = false;
        
        
        // dpov is short for drive motor encoder position average
        dpos = drivetrain.getAverageEncoderDistance();
        
        //ipovpp1 is short for in position to place piece 1
        ipotpp1 = false;
        
        //dtmos is the drive train motor output straight
        dtmos = 0;
        
        //dtmot is drive train motor output turn
        dtmot = 0;
        
        //direction MUST BE 1 OR -1 because it changes all of the directions of going foward
        direction = 1; 
        
        // turndirection changes if it is not correcting in the right direction ONLY -1 OR 1 should be used 
        turndirection = 1;

        // wip means wrist position to place
        wip = false;
        // wipc is wip comand tells wirst to move to place.
        wipc = false;

        // this is the release command
        rc1 = false;

        // back up to lower arm gives the command to back up because the piece has been released
        backtolowerarm = false;

        // armlowered tells us if the arm is in position to rip across the field
        armlowered = false;

        // taxied from comunity
        taxi = false;

        //this command tells us to balance
        Balancing = false;
    }

    public void run_Auto1(){
       
       if (Balancing == false ){
       
       
            //this tells the arm at the start of auto 1 to go up
            if (start_position==false){
                elbow.Esetpoint=-39.071121;
                shoulder.Ssetpoint = 150.377;
                elbow.EkP=0.05;
                wrist.Wsetpoint=2.8;
                start_position=true;
                hand.hsetpoint=-20;
                pneumatics.mdoubleSolenoid.set(Value.kForward);
            }
            
            // this tells it to drive from are starting position till we get up next to the Nodes and it tells it when it is in position
            if ( start_position == true && ipotpp1 == false ) {
                
                if (dpos>.1){
                    dtmos=.2*direction;
                    }
                
                else if (dpos < .1) {
                    dtmos=0;
                    ipotpp1=true;
                }

            
            }

                // this tells the arm to go up
            if (wipc == false  && elbow.Elbowencoder.getPosition()<-30 && ipotpp1 == true){
                wrist.Wsetpoint = -20;
                wipc = true;
            }
            
            // this tells the hand to open.
            if (wipc == true && wrist.wriste.getPosition() < -13 && rc1 == false){
                hand.hsetpoint = 0;
                pneumatics.mdoubleSolenoid.set(Value.kReverse);
                rc1 = true;
            }
            
            // This command tells it to flip up the wrist because we have released
            if (rc1 == true && hand.hande.getPosition() > -3){
                wrist.Wsetpoint = 0;
                backtolowerarm = true;
            }
            
            // This command tells us to back up and then lower the arm
            if (backtolowerarm == true && armlowered == false){
                if (dpos > .35){
                    elbow.Esetpoint = 0;
                    shoulder.Ssetpoint = 0;
                    elbow.EkP=0.015;
                }
                
                if (dpos < 3.8 ){
                    dtmos = -.2 * direction;
                }
                if (elbow.Elbowencoder.getPosition() >- 2){
                    armlowered = true;
                }

            }

                //This speads us up when the arm lowers
            if (armlowered == true && taxi == false){
                if (dpos < 3.8){
                    dtmos = -.3 * direction;
                    }
                else if (dtmos >= 3.8){
                    dtmos = 0;
                    Commands.waitSeconds(2);
                    taxi = true;
                }

            }
            
                // this tells us to go back onto the charge station
            if (taxi == true && Balancing == false){
                if ( dpos > 1.5 ){
                    dtmos = .3 * direction;
                }
                else if (dpos < 1.5){
                    dtmos = 0;
                    Balancing = true;
                }
            }

            //this is used to keep us straight line up with the nodes ****In the future will add a button on drive station to turn it on and off
                
            if (drivetrain.m_gyro.getYaw() > 3){
                dtmot = -.2 * turndirection;
                }
            else if (drivetrain.m_gyro.getYaw() < -3){
                dtmos = .2 * turndirection;
                }
            else if (drivetrain.m_gyro.getYaw() > -3 && drivetrain.m_gyro.getYaw()<.3){
                dtmot = 0;
            }

        }
        drivetrain.tankDrive(dtmot-dtmos, dtmot+dtmos, false);
    }
}