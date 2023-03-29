package frc.robot;
 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends TimedRobot {
  final double bkP = -0.01;
    //final double bkI = -0.005;
     final double bkD = -0.0009;
     final double bkI = -0.00;
     //final double bkD = -0.00;
    final double biLimit = 3;
    double setpoint = 0;
    double errorSum = 0;
    double lastTimestamp = 0;
    double lastError = 0;
    private double Speedvar=0.0;
    private double turnerror =0.0;
    double berror=0;
    double errorRate=0;
    public final Timer wait = new Timer();


    private static final String kDefaultSpeed = "Demo";
        private static final String kCompetitionSpeed = "Competition";
        private String speed_selected;
        private final SendableChooser<String> speed_chooser = new SendableChooser<>();


    //limelight readings
    private double tv = 0.0;
    private double tx = 0.0;
    private double ty = 0.0;
    private double ta = 0.0;
    private double distanceToTarget = -1.0;  //distance to target in inches, -1.0 means no target in sight
    private boolean autoTargeting = false;
    private boolean targetSighted = false;
    private boolean targetInRange = false;
    private boolean targetAimed = false;
    private boolean targetLocked = false;
    private boolean floppywrist = false;
    
    //These need to be set to the height of the limelight, the angle of the limelight, and the height of the target
    static final double limelightHeight = 7.5;  // measure in inches to center of lens
    static final double limelightAngle = 25.0;    // measure in degrees leaned back from vertical
    static final double targetHeight = 42;     // measure in inches
    static final double aimAdjust = -0.1;
    static final double distanceAdjust = -0.1;
    static final double min_aim_command = 0.05;
    static final double MinTargetRange = 36.0;  // no closer than 10 feet
    static final double MaxTargetRange = 40.0;  // no farther than 11 feet
    
    // the following values are used to compute a drive speed relative to the distance away from the target, the closer we get the slower we want to go
    public double MaxDriveSpeed = 0.3;  // this is the max speed we want a drive motor to run at
    static final double MinDriveSpeed = 0.1;  // this is the min speed we want a drive motor to run at
    static final double MaxDriveDistance = 144.0;  // this is the distance where we want the drive motors to be at max speed
    static final double MinDriveDistance = 1.0;  // this is the distance where we want the drive motors to be at min speed


    // the following values are used to compute a turning speed relative to the degrees we are offset from the target, the more in line we get the slower we want to turn
    static final double MaxTurnSpeed = 0.1;  // this is the max speed we want a drive motor to run at
    static final double MinTurnSpeed = 0.0;  // this is the min speed we want a drive motor to run at
    static final double MaxTurnDegrees = 27.0;  // this is the distance where we want the drive motors to be at max speed
    static final double MinTurnDegrees = 1.0;  // this is the distance where we want the drive motors to be at min speed
    
    public double TargetCenter = (MaxTargetRange + MinTargetRange)/2.0;    // the center of the target range
    double dsensorPosition=0;

  private double doutputSpeed;

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private static final String kCustomAuto2 = "my Auto 2";
  private static final String kCustomAuto3 = "my Auto 3";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private boolean start_position;
  double derror;
  double derrorRate;
  double dt;
  double sensorPosition;
  double turn;
  double tsensorPosition;
  private boolean turned=false;
  private boolean armdown=false;
  private boolean turncommand=false;
  private boolean grabpiece2=false;
    int timer;
    
      //true or false values used
  
      private boolean ipotpp1;
      private boolean wip;
      private boolean wipc;
      private boolean rc1;
      private boolean rc2;
      private boolean backtolowerarm;
      private boolean armlowered;
      private boolean taxi;
      private boolean Balancing;
      private boolean turn1;
      private boolean inpositionpickup;
      private boolean squezed;
      private boolean turn2;
      private boolean ppiece2;
      private boolean inpositiontoplacepiece;
      private boolean placedpiece;

      private double dpos;
      private double dtmos;
      private double dtmot;
      private double direction;
      private double turndirection;
      

    public double speedMult;

    private final Timer m_timer = new Timer();
    public Joystick left;
    public Joystick right;
    public XboxController controller2;
    private boolean onchargestation= false;
    
    public DriveTrain drivetrain;
    
    private Hand Hand;

    private Wrist wrist;

    private Elbow elbow;

    private Shoulder shoulder;

   // private Balancing balancing;

  // private turnadjust turn;
    
   private Pneumatics pneumatics;

   private color_sensor color_sensor;

  //  private Auto1 auto1;

   //private Auto2_balance auto2_balance;

   private Auto3 auto3;

  //  private final double kDriveTick2Feet = 1.0 / 128 * 6 * Math.PI / 12;

   

    @Override
  public void robotInit() {
    
    speed_chooser.setDefaultOption("DemoSpeed", kDefaultSpeed);
    speed_chooser.addOption("Competition Speed", kCompetitionSpeed);
    SmartDashboard.putData("Speed choices", speed_chooser);
    speedMult = .7;
    m_chooser.setDefaultOption("Red 2 piece auto", kDefaultAuto);
    m_chooser.setDefaultOption("1 piece backup auto", kCustomAuto3);
    m_chooser.addOption("Balancing Auto", kCustomAuto);
    m_chooser.addOption("blue 2 piece", kCustomAuto2);
    SmartDashboard.putData("Auto choices", m_chooser);
     // This creates our drivetrain subsystem that contains all the motors and motor control code
     
     drivetrain = new DriveTrain();
      
     elbow = new Elbow();

     wrist = new Wrist();

      shoulder = new Shoulder();

      Hand = new Hand();
     
     pneumatics = new Pneumatics();

    left = new Joystick(0);
		right = new Joystick(1);
		controller2 = new XboxController(2);
   drivetrain.m_gyro.reset();
   pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
   
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
   direction = -1; 
   
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

   // this command tells us to turn
   turn1 = false;

   // this command tells us if we are in position to pick up
   inpositionpickup = false;

   // this command tells us if we are squezing the second piece or not
   squezed = false;

   // this tells us if we have turned to face the cube node
   turn2 = false;

   // this tell us we are ready to place piece 2
   ppiece2 = false;

   inpositiontoplacepiece = false;

   placedpiece = false;

  }

  

  @Override
  public void robotPeriodic() { 
   shoulder.Shoulder_Run();
    elbow.ElbowRun();  
  SmartDashboard.getNumber("elbow", elbow.Elbowencoder.getPosition());
  SmartDashboard.getNumber("Shoulder", shoulder.shouldere.getPosition());
   Hand.Hand_Run();
   wrist.Wrist_Run();
  drivetrain.run_drive();
  drivetrain.getAverageEncoderDistance();
  pneumatics.Run_Pneumatics();
  SmartDashboard.putNumber("tilt angle",drivetrain.m_gyro.getRoll());
  SmartDashboard.putNumber("foward distance", drivetrain.getAverageEncoderDistance());
  SmartDashboard.putNumber("Turn angle", drivetrain.m_gyro.getAngle());
  SmartDashboard.putNumber("distance", dsensorPosition);
  SmartDashboard.putNumber("output", doutputSpeed);
  dsensorPosition=drivetrain.getAverageEncoderDistance();
  }
  


  
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    //  m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    drivetrain.m_gyro.reset();
    m_timer.reset();
		m_timer.start();
    drivetrain.setbrake(false);
    pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
     Hand.hsetpoint=-20;
    onchargestation=false;
    start_position = false;
    turned=false;
    turncommand=false;
   grabpiece2=false;
   Balancing=false;
   ipotpp1 = false;
    
  }


  @Override
  public void autonomousPeriodic() {
    DataLogManager.start();
    
    switch (m_autoSelected) {
        case kCustomAuto:
        dpos = drivetrain.getAverageEncoderDistance();
        if (Balancing == false ){
       
       
          //this tells the arm at the start of auto 1 to go up
          if (start_position==false){
              elbow.Esetpoint=-39.071121;
              shoulder.Ssetpoint = 150.377;
              elbow.EkP=0.05;
              wrist.Wsetpoint=2.8;
              start_position=true;
              Hand.hsetpoint=-20;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
          }
          
          // this tells it to drive from are starting position till we get up next to the Nodes and it tells it when it is in position
          if ( start_position == true && ipotpp1 == false ) {
              
              if (dpos>.1){
                  dtmos=.21*direction;
                  }
              
              else if (dpos < .1) {
                  dtmos=0;
                  ipotpp1=true;
              }

          
          }

              // this tells the wrist to go down
          if (wipc == false  && elbow.Elbowencoder.getPosition()<-30 && ipotpp1 == true){
              wrist.Wsetpoint = -20;
              wipc = true;
          }
          
          // this tells the hand to open.
          if (wipc == true && wrist.wriste.getPosition() < -17 && rc1 == false){
              Hand.hsetpoint = 1;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
              rc1 = true;
          }
          
          // This command tells it to flip up the wrist because we have released
          if (rc1 == true && Hand.hande.getPosition() > -2){
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
                  dtmos = -.21* direction;
              }
              if (elbow.Elbowencoder.getPosition() > -4){
                  armlowered = true;
              }

          }

              //This speads us up when the arm lowers
          if (armlowered == true && taxi == false){
              if (dpos < 3.8){
                  dtmos = -.21 * direction;
                  }
              else if (dpos >= 3.8){
                  dtmos = 0;
                 // Commands.waitSeconds(5);
                  taxi = true;
              }

          }
          
              // this tells us to go back onto the charge station
          if (taxi == true && Balancing == false){
              if ( dpos > 2 ){
                  dtmos = .21 * direction;
              }
              else if (dpos < 2){
                  dtmos = 0;
                  Balancing = true;
              }
          }

          dtmot=0;

      }
      
      //This is our auto balance
      if(Balancing == true){
        
        if (drivetrain.m_gyro.getRoll()<3 && drivetrain.m_gyro.getRoll()>-3){
          //chargestationbalance=true;
          Speedvar=0;
          drivetrain.setbrake(false);
        }

          // get sensor position
           double sensorPosition = drivetrain.m_gyro.getRoll();
  
          // calculations
          berror = setpoint -sensorPosition;
          double dt = Timer.getFPGATimestamp() - lastTimestamp;
  
          if (Math.abs(berror) < biLimit) {
            errorSum += berror * dt;
          }
  
          errorRate = (berror - lastError) / dt;
  
          Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
  
          // output to motors
          Speedvar=-outputSpeed;
  
          // update last- variables
          lastTimestamp = Timer.getFPGATimestamp();
          lastError = berror;
          

          if (Speedvar>.2){
            Speedvar=.2;
            }
          if (Speedvar<-.2){
              Speedvar=-.2;}
  
              dtmos= Speedvar;
      }

      drivetrain.tankDrive(-dtmot+dtmos, dtmot+dtmos, false);
      
          break;
        
        





          case kDefaultAuto:
          default:
          dpos = drivetrain.getAverageEncoderDistance();
       
          //this tells the arm at the start of auto 1 to go up
          if (start_position==false){
              elbow.Esetpoint=-39.071121;
              shoulder.Ssetpoint = 150.377;
              elbow.EkP=0.05;
              wrist.Wsetpoint=2.8;
              start_position=true;
              Hand.hsetpoint=-20;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
          }
          
          // this tells it to drive from are starting position till we get up next to the Nodes and it tells it when it is in position
          if ( start_position == true && ipotpp1 == false ) {
              
              if (dpos > .04){
                  dtmos=.2*direction;
                  }
              
              else if (dpos < .04) {
                  dtmos=0;
                  ipotpp1=true;
              }

          
          }

              // this tells the wrist to go down
          if (wipc == false  && elbow.Elbowencoder.getPosition()<-30 && ipotpp1 == true){
              wrist.Wsetpoint = -20;
              wipc = true;
          }
          
          // this tells the hand to open.
          if (wipc == true && wrist.wriste.getPosition() < -17 && rc1 == false){
              Hand.hsetpoint = 1;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
              rc1 = true;
          }
          
          // This command tells it to flip up the wrist because we have released
          if (rc1 == true && Hand.hande.getPosition() > -2){
              wrist.Wsetpoint = 0;
              backtolowerarm = true;
          }
          
          // This command tells us to back up and then lower the arm
          if (backtolowerarm == true && armlowered == false){
              if (dpos > .35){
                  elbow.Esetpoint = 0;
                  shoulder.Ssetpoint = 0;
                  elbow.EkP=0.012;
              }
              
              if (dpos < 3.8 ){
                  dtmos = -.21* direction;
              }
              if (elbow.Elbowencoder.getPosition() > -15){
                  armlowered = true;
              }

                if (drivetrain.m_gyro.getAngle() > .5 ){
               dtmot = -.02 * turndirection;
                }
               else if ( drivetrain.m_gyro.getAngle()  < -.5 ){
               dtmot = .02 * turndirection;
               
                }
                else {
                  dtmot = 0;
                }
          }

              //This speads us up when the arm lowers
          if (armlowered == true && taxi == false){
              if (dpos < 3.9){
                  dtmos = -.35 * direction;
                  }
              else if (dpos >= 3.9){
                  dtmos = 0;
                  taxi = true;
              }

              if (drivetrain.m_gyro.getAngle() > .5 ){
                dtmot = -.02 * turndirection;
                 }
                else if ( drivetrain.m_gyro.getAngle()  < -.5 ){
                dtmot = .02 * turndirection;
                
                 }
                 else {
                   dtmot = 0;
                 }

          }
          
              // this tells us to turn toward the second piece 
          if (taxi == true && turn1 == false){
              if (drivetrain.m_gyro.getAngle() < 134){
                  dtmot = .25 * turndirection;
              }
              else if (drivetrain.m_gyro.getAngle() >= 134){
                  dtmot = 0;
                  turn1 = true;
              }
          }
          dpos = drivetrain.getAverageEncoderDistance();
            // this tell us to go toward the piece
          if (turn1 == true && inpositionpickup== false){
              if (dpos > 3.1){
                dtmos = .3 * direction;
                }
              else if (dpos <= 3.1){
                dtmos = 0;
                inpositionpickup = true;
              }
              
          }

          if (inpositionpickup == true && squezed == false){
            Hand.hsetpoint=-20;
            pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
            squezed = true;
            
          }

          if (squezed == true && Hand.hande.getPosition() < -3 && turn2 == false){
            wrist.Wsetpoint = 3.8;
            
            if (drivetrain.m_gyro.getAngle() < 337 ){
              dtmot = .21 * turndirection;
          }
           if (drivetrain.m_gyro.getAngle() >= 337 ){
              dtmot = 0;
              turn2 = true;
              
          }
            // if (drivetrain.m_gyro.getYaw() > 2 ){
            //   dtmot = -.21 * turndirection;
            //   }
            //   else if ( drivetrain.m_gyro.getYaw() <= 2){
            //   dtmot = 0;
            //   turn2 = true;
            // }

          }

          if (turn2 == true){
           if (inpositiontoplacepiece == false){
            if (dpos >-2.45){
              dtmos=.35*direction;
              }
          
              else if (dpos  <= -2.45 ) {
              dtmos=0;
              inpositiontoplacepiece = true; 
            }

            if (drivetrain.m_gyro.getAngle() > 361 ){
              dtmot = -.02 * turndirection;
               }
              else if ( drivetrain.m_gyro.getAngle()  < 359 ){
              dtmot = .02 * turndirection;
              
               }
               else {
                 dtmot = 0;
               }
          }
            //-1.2
            if(dpos < 0 && ppiece2 == false){
              elbow.Esetpoint=-39.071121;
              shoulder.Ssetpoint = 150.377;
              elbow.EkP=0.05;
              wrist.Wsetpoint=2.8;
              Hand.hsetpoint=-20;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
              ppiece2 = true;
            }
            if (ppiece2 == true && elbow.Elbowencoder.getPosition()<-30 && inpositiontoplacepiece == true){
              wrist.Wsetpoint = -20;
            }

            if (ppiece2 == true &&  wrist.wriste.getPosition() < -17 ){
              Hand.hsetpoint = 1;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
            }

            if (ppiece2 == true &&Hand.hande.getPosition() > -2){
              placedpiece = true;
            }
            if (placedpiece == true) {
              if (dpos >= -2){
                elbow.Esetpoint = 0;
                shoulder.Ssetpoint = 0;
                elbow.EkP=0.012;
                dtmos = 0;
                wrist.Wsetpoint = 0;
                }
            
                if (dpos < -1.5 ){
                    dtmos = -.21* direction;
              }

            }
            


          }
      
         
            drivetrain.tankDrive(-dtmot+dtmos, dtmot+dtmos, false);
       
        break;

        




















        

        case kCustomAuto2:
          dpos = drivetrain.getAverageEncoderDistance();
       
          //this tells the arm at the start of auto 1 to go up
          if (start_position==false){
              elbow.Esetpoint=-39.071121;
              shoulder.Ssetpoint = 150.377;
              elbow.EkP=0.05;
              wrist.Wsetpoint=2.8;
              start_position=true;
              Hand.hsetpoint=-20;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
          }
          
          // this tells it to drive from are starting position till we get up next to the Nodes and it tells it when it is in position
          if ( start_position == true && ipotpp1 == false ) {
              
              if (dpos > .04){
                  dtmos=.2*direction;
                  }
              
              else if (dpos < .04) {
                  dtmos=0;
                  ipotpp1=true;
              }

          
          }

              // this tells the wrist to go down
          if (wipc == false  && elbow.Elbowencoder.getPosition()<-30 && ipotpp1 == true){
              wrist.Wsetpoint = -20;
              wipc = true;
          }
          
          // this tells the hand to open.
          if (wipc == true && wrist.wriste.getPosition() < -17 && rc1 == false){
              Hand.hsetpoint = 1;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
              rc1 = true;
          }
          
          // This command tells it to flip up the wrist because we have released
          if (rc1 == true && Hand.hande.getPosition() > -2){
              wrist.Wsetpoint = 0;
              backtolowerarm = true;
          }
          
          // This command tells us to back up and then lower the arm
          if (backtolowerarm == true && armlowered == false){
              if (dpos > .35){
                  elbow.Esetpoint = 0;
                  shoulder.Ssetpoint = 0;
                  elbow.EkP=0.012;
              }
              
              if (dpos < 3.8 ){
                  dtmos = -.21* direction;
              }
              if (elbow.Elbowencoder.getPosition() > -15){
                  armlowered = true;
              }

                if (drivetrain.m_gyro.getAngle() > .5 ){
               dtmot = -.02 * turndirection;
                }
               else if ( drivetrain.m_gyro.getAngle()  < -.5 ){
               dtmot = .02 * turndirection;
               
                }
                else {
                  dtmot = 0;
                }
          }

              //This speads us up when the arm lowers
          if (armlowered == true && taxi == false){
              if (dpos < 3.9){
                  dtmos = -.35 * direction;
                  }
              else if (dpos >= 3.9){
                  dtmos = 0;
                  taxi = true;
              }

              if (drivetrain.m_gyro.getAngle() > .5 ){
                dtmot = -.02 * turndirection;
                 }
                else if ( drivetrain.m_gyro.getAngle()  < -.5 ){
                dtmot = .02 * turndirection;
                
                 }
                 else {
                   dtmot = 0;
                 }

          }
          
              // this tells us to turn toward the second piece 
          if (taxi == true && turn1 == false){
              if (drivetrain.m_gyro.getAngle() > -128){
                  dtmot = -.25 * turndirection;
              }
              else if (drivetrain.m_gyro.getAngle() <= -128){
                  dtmot = 0;
                  turn1 = true;
              }
          }
          dpos = drivetrain.getAverageEncoderDistance();
            // this tell us to go toward the piece
          if (turn1 == true && inpositionpickup== false){
              if (dpos > 3.1){
                dtmos = .3 * direction;
                }
              else if (dpos <= 3.1){
                dtmos = 0;
                inpositionpickup = true;
              }
              
          }

          if (inpositionpickup == true && squezed == false){
            Hand.hsetpoint=-20;
            pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
            squezed = true;
            
          }

          if (squezed == true && Hand.hande.getPosition() < -3 && turn2 == false){
            wrist.Wsetpoint = 3.8;
            
            if (drivetrain.m_gyro.getAngle() > -336 ){
              dtmot = -.21 * turndirection;
          }
           if (drivetrain.m_gyro.getAngle() <= -336 ){
              dtmot = 0;
              turn2 = true;
              
          }
            // if (drivetrain.m_gyro.getYaw() > 2 ){
            //   dtmot = -.21 * turndirection;
            //   }
            //   else if ( drivetrain.m_gyro.getYaw() <= 2){
            //   dtmot = 0;
            //   turn2 = true;
            // }

          }

          if (turn2 == true){
           if (inpositiontoplacepiece == false){
            if (dpos >-2.45){
              dtmos=.35*direction;
              }
          
              else if (dpos  <= -2.45 ) {
              dtmos=0;
              inpositiontoplacepiece = true; 
            }

            if (drivetrain.m_gyro.getAngle() < -360 ){
              dtmot = .02 * turndirection;
               }
              else if ( drivetrain.m_gyro.getAngle()  > -358 ){
              dtmot = -.02 * turndirection;
              
               }
               else {
                 dtmot = 0;
               }
          }
            //-1.2
            if(dpos < 0 && ppiece2 == false){
              elbow.Esetpoint=-39.071121;
              shoulder.Ssetpoint = 150.377;
              elbow.EkP=0.05;
              wrist.Wsetpoint=2.8;
              Hand.hsetpoint=-20;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
              ppiece2 = true;
            }
            if (ppiece2 == true && elbow.Elbowencoder.getPosition()<-30 && inpositiontoplacepiece == true){
              wrist.Wsetpoint = -20;
            }

            if (ppiece2 == true &&  wrist.wriste.getPosition() < -17 ){
              Hand.hsetpoint = 1;
              pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
            }

            if (ppiece2 == true &&Hand.hande.getPosition() > -2){
              placedpiece = true;
            }
            if (placedpiece == true) {
              if (dpos >= -2){
                elbow.Esetpoint = 0;
                shoulder.Ssetpoint = 0;
                elbow.EkP=0.012;
                dtmos = 0;
                wrist.Wsetpoint = 0;
                }
            
                if (dpos < -1.5 ){
                    dtmos = -.21* direction;
              }

            }
            


          }
      
         
            drivetrain.tankDrive(-dtmot+dtmos, dtmot+dtmos, false);
       
        break;











        case kCustomAuto3:
        dpos = drivetrain.getAverageEncoderDistance();
     
        //this tells the arm at the start of auto 1 to go up
        if (start_position==false){
            elbow.Esetpoint=-39.071121;
            shoulder.Ssetpoint = 150.377;
            elbow.EkP=0.05;
            wrist.Wsetpoint=2.8;
            start_position=true;
            Hand.hsetpoint=-20;
            pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
        }
        
        // this tells it to drive from are starting position till we get up next to the Nodes and it tells it when it is in position
        if ( start_position == true && ipotpp1 == false ) {
            
            if (dpos > .04){
                dtmos=.2*direction;
                }
            
            else if (dpos < .04) {
                dtmos=0;
                ipotpp1=true;
            }

        
        }

            // this tells the wrist to go down
        if (wipc == false  && elbow.Elbowencoder.getPosition()<-30 && ipotpp1 == true){
            wrist.Wsetpoint = -20;
            wipc = true;
        }
        
        // this tells the hand to open.
        if (wipc == true && wrist.wriste.getPosition() < -17 && rc1 == false){
            Hand.hsetpoint = 1;
            pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
            rc1 = true;
        }
        
        // This command tells it to flip up the wrist because we have released
        if (rc1 == true && Hand.hande.getPosition() > -2){
            wrist.Wsetpoint = 0;
            backtolowerarm = true;
        }
        
        // This command tells us to back up and then lower the arm
        if (backtolowerarm == true && armlowered == false){
            if (dpos > .35){
                elbow.Esetpoint = 0;
                shoulder.Ssetpoint = 0;
                elbow.EkP=0.012;
            }
            
            if (dpos < 3.8 ){
                dtmos = -.21* direction;
            }
            if (elbow.Elbowencoder.getPosition() > -15){
                armlowered = true;
            }

              if (drivetrain.m_gyro.getAngle() > .5 ){
             dtmot = -.02 * turndirection;
              }
             else if ( drivetrain.m_gyro.getAngle()  < -.5 ){
             dtmot = .02 * turndirection;
             
              }
              else {
                dtmot = 0;
              }
        }

            //This speads us up when the arm lowers
        if (armlowered == true && taxi == false){
            if (dpos < 3.9){
                dtmos = -.35 * direction;
                }
            else if (dpos >= 3.9){
                dtmos = 0;
                taxi = true;
            }

            if (drivetrain.m_gyro.getAngle() > .5 ){
              dtmot = -.02 * turndirection;
               }
              else if ( drivetrain.m_gyro.getAngle()  < -.5 ){
              dtmot = .02 * turndirection;
              
               }
               else {
                 dtmot = 0;
               }

        }
      break;
    
    }
  }




@Override
public void teleopInit(){
drivetrain.setbrake(true);
} 

@Override
  public void teleopPeriodic() {
    DataLogManager.start();
    
    //drivetrain.setbrake(false);
    speed_selected = speed_chooser.getSelected();
                SmartDashboard.putString("Speed Chosen", speed_selected);


                if (speed_selected == kDefaultSpeed) {
                        MaxDriveSpeed = 0.3;
                        speedMult = .4;
                } else {
                        MaxDriveSpeed = 0.6;
                        speedMult= .6;
                }


                Update_Limelight_Tracking();


                if (autoTargeting){
                        SmartDashboard.putString("autotargeting", "true");
                }else {
                        SmartDashboard.putString("autotargeting", "false");
            }


                if (targetSighted){
                        SmartDashboard.putString("targetSighted", "true");
                }else {
                        SmartDashboard.putString("targetSighted", "false");
            }


                if (targetAimed){
                        SmartDashboard.putString("targetAimed", "true");
                }else {
                        SmartDashboard.putString("targetAimed", "false");
            }


                if (targetInRange){
                        SmartDashboard.putString("targetInRange", "true");
                }else {
                        SmartDashboard.putString("targetInRange", "false");
            }


                if (targetLocked){
                        SmartDashboard.putString("targetLocked", "true");
                }else {
                        SmartDashboard.putString("targetLocked", "false");
            }
    
      if(right.getThrottle()>.5){
        drivetrain.setbrake(false);
      }
      else {
        drivetrain.setbrake(true);
      }
      if (controller2.getRightStickButton()){
          setpoint = 0;
     
          if (drivetrain.m_gyro.getRoll()<3 && drivetrain.m_gyro.getRoll()>-3){
            //chargestationbalance=true;
            Speedvar=0;
            drivetrain.setbrake(false);
          }

            // get sensor position
             double sensorPosition = drivetrain.m_gyro.getRoll();
    
            // calculations
            berror = setpoint -sensorPosition;
            double dt = Timer.getFPGATimestamp() - lastTimestamp;
    
            if (Math.abs(berror) < biLimit) {
              errorSum += berror * dt;
            }
    
            errorRate = (berror - lastError) / dt;
    
            Double outputSpeed = bkP * berror + bkI * errorSum + bkD * errorRate;
    
            // output to motors
            Speedvar=-outputSpeed;
    
            // update last- variables
            lastTimestamp = Timer.getFPGATimestamp();
            lastError = berror;
            
            // if (drivetrain.m_gyro.getYaw()>3){
            //   turn = .05;
            //   }
            //    if (drivetrain.m_gyro.getYaw()<2.5 && drivetrain.m_gyro.getYaw() >-2.5){
            //   turn=0;
            //  }
            //   else if (drivetrain.m_gyro.getYaw()<-3){
            //     turn =-.05;
            // }
  
            if (Speedvar>.2){
              Speedvar=.2;
              }
            if (Speedvar<-.2){
                Speedvar=-.2;}
    
               double directionL= Speedvar;
               double directionR= Speedvar;
               drivetrain.tankDrive(directionL, directionR, false);
            }
            
          else if(left.getTrigger()){
            drivetrain.arcadeDrive(left.getY()*speedMult,right.getX()*speedMult, false);
          }
          else {
            drivetrain.tankDrive(left.getY()*speedMult, right.getY()*speedMult, false);
            
          }
 
      





          
      // // Hand controlled by left and right triggers
      if (controller2.getXButton()) {
        Hand.hsetpoint = 0;
      pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      } 
        else if (controller2.getYButton()) {
          Hand.hsetpoint=-20;
    pneumatics.mdoubleSolenoid.set(DoubleSolenoid.Value.kForward);
      }
    
    if (controller2.getAButton()) {
      elbow.Esetpoint=-23;
      elbow.EkP=0.05;
      } 
     if (controller2.getRightTriggerAxis()>.1) {
      wrist.Wsetpoint=0;
      elbow.EkP=0.012;
      elbow.Esetpoint = 0;
      shoulder.Ssetpoint=0;
    }
    if (controller2.getBButton()){
      elbow.Esetpoint=-28; 
      elbow.EkP=0.05;
      //drivetrain.m_gyro.reset();
      
    }
   
    if (controller2.getPOV()==0){
      wrist.Wsetpoint= 3.8;
    }

        if (controller2.getBackButton()){
         // drivetrain.m_gyro.reset();
        }
      
       //high cone
       if (controller2.getLeftTriggerAxis()>.1) {
          elbow.Esetpoint=-39.071121;
          shoulder.Ssetpoint = 150.377;
          elbow.EkP=0.05;
        } 
         
         
       
       if(controller2.getPOV()==180){
         wrist.Wsetpoint=0;}
         else if (controller2.getLeftBumper()){
         wrist.Wsetpoint=-20;  
       }

}








public void Update_Limelight_Tracking() {
  tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);  // 0 or 1
  tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);  // -27 to 27
  ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);  // -20.5 to 20.5
  ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);  // 0 to 100


  SmartDashboard.putNumber("LimelightX", tx);
  SmartDashboard.putNumber("LimelightY", ty);
  SmartDashboard.putNumber("LimelightArea", ta);
  SmartDashboard.putNumber("TargetSpotted", tv);


  if (tv < 1.0) {
          targetSighted = false;
          distanceToTarget = -1.0;
  } else {
          targetSighted = true;        


          //calculate distance to target
          double angleToTargetAsRadians = (limelightAngle + ty) * (3.14159 / 180.0);  
          distanceToTarget = (targetHeight - limelightHeight)/Math.tan(angleToTargetAsRadians);
  }


  SmartDashboard.putNumber("Distance", distanceToTarget);
  return;
}

}