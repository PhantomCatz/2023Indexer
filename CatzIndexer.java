package frc.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.DataLogger.DataCollection;

import com.revrobotics.ColorSensorV3;


public class CatzIndexer{
    //          SPARKMAX DEFS
    public CANSparkMax indexerMtrCtrlFRNT;

    private final int INDEXER_FRNT_MC_CAN_ID        = 40;//1;
    private final int INDEXER_FRNT_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FRNT_SPEED = 0.4;
    public final double INDEXER_FRNT_SPEED_FAST = 1;

    //
    public final double INDEXER_SPEED_OFF = 0.0;
    
    //          SECOND MOTOR DEFS
    public CANSparkMax indexerMtrCtrlBACK;

    private final int INDEXER_BACK_MC_CAN_ID        = 56;//2;
    private final int INDEXER_BACK_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_BACK_SPEED = 0.2;

    public CANSparkMax indexerMtrCtrlRGT_BLT_WHEEL;

    private final int INDEXER_MC_CAN_ID_RGT       = 37;//2;
    private final int INDEXER_MC_CURRENT_LIMIT_RGT = 60;

    public final double INDEXER_SIDE_WHEEL_SPEED = 0.5;

    public CANSparkMax indexerMtrCtrlLFT_BLT_WHEEL;

    private final int INDEXER_MC_CAN_ID_LFT       = 38;//2;
    private final int INDEXER_MC_CURRENT_LIMIT_LFT = 60;


    
    //          Flipper Defs
    public CANSparkMax indexerMtrCtrlFLIPPER;

    private final int INDEXER_FLIPPER_MC_CAN_ID        = 3;
    private final int INDEXER_FLIPPER_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FLIPPER_SPEED = 0.2;

    //          BEAMBREAK DEFS
  
    public DigitalInput beamBreak;
  
    private final int BEAM_BREAK_DIO_PORT_LFT = 4;//up and down?

    private boolean beamBreakContinuity = false;


    
    //            COLOR SENSOR DEFS
    private final I2C.Port i2cPort = I2C.Port.kMXP;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    private final int PROXIMITY_DISTANCE_THRESHOLD = 85;

    //          COLOR SENSOR COLOR CONSTANTS
    private final double CUBE_RED_THRESHOLD     = 0.25;
    private final double CUBE_GREEN_THRESHOLD   = 0.45;
    private final double CUBE_BLUE_THRESHOLD    = 0.28;

    private final double CONE_BLUE_THRESHOLD    = 0.2;

    //          SENSOR VARIABLES
    private double RedValue = -999.0;
    private double GreenValue = -999.0;
    private double BlueValue = -999.0;

    Color detectedColor;
    double IR;
    int proximity;

    public DigitalInput LimitSwitchRGT;
    public DigitalInput LimitSwitchLFT;

//    private final int LIMIT_SWITCH_DIO_PORT_RGT = 5; 
//    private final int LIMIT_SWITCH_DIO_PORT_LFT = 6;

    boolean limitSwitchPressedLFT;
    boolean limitSwitchPressedRGT;

    
    

    //          objectdetection and flags
    public boolean objectReady = false; // needs to be set back to false in elevator class
    
    private boolean objectDetected; 

    public static boolean objectCurrentlyIntaking;

    private boolean clawActive; // needs to be set off in elevator class

    public boolean flipperActive;
    private boolean flipperDeactivating;

    private boolean colorTestActive;

    public static boolean isConeDetected = false;

    public static boolean isCubeDetected = false;
    

    //
    private Thread Indexer;

    private Timer indexerTimer;
    private double indexerTime;

    private Timer flipperTimer;
    private double flipperTime;

    int finalStateINT;

    private String colorSelected = "None";


    //          CONSTANTS
    private double MAX_BELT_TIME = 4;
    private double CONE_TEST_DELAY_THRESHOLD = 4.8;
    private double FLIPPER_DELAY_THRESHOLD = 7;
   

    public CatzLog data;

    private RelativeEncoder m_encoder;
    private SparkMaxPIDController elevSpoolPid;
    private final double PID_ELEVATOR_SPOOL_KP = 0.00;
    private final double PID_ELEVATOR_SPOOL_KI = 0.00;
    private final double PID_ELEVATOR_SPOOL_KD = 0.00;



    //                      End of definitions

    public CatzIndexer() 
    {
        indexerMtrCtrlFRNT = new CANSparkMax(INDEXER_FRNT_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlFRNT.restoreFactoryDefaults();
        indexerMtrCtrlFRNT.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlFRNT.setSmartCurrentLimit(INDEXER_FRNT_MC_CURRENT_LIMIT);

      
        indexerMtrCtrlBACK = new CANSparkMax(INDEXER_BACK_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlBACK.restoreFactoryDefaults();
        indexerMtrCtrlBACK.setIdleMode(IdleMode.kBrake);
        indexerMtrCtrlBACK.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);


        indexerMtrCtrlFLIPPER = new CANSparkMax(INDEXER_FLIPPER_MC_CAN_ID, MotorType.kBrushless); 

        indexerMtrCtrlFLIPPER.restoreFactoryDefaults();
        indexerMtrCtrlFLIPPER.setIdleMode(IdleMode.kCoast);
        indexerMtrCtrlFLIPPER.setSmartCurrentLimit(INDEXER_FLIPPER_MC_CURRENT_LIMIT);
    
        indexerMtrCtrlRGT_BLT_WHEEL = new CANSparkMax(INDEXER_MC_CAN_ID_RGT, MotorType.kBrushless); 

        indexerMtrCtrlRGT_BLT_WHEEL.restoreFactoryDefaults();

        indexerMtrCtrlLFT_BLT_WHEEL = new CANSparkMax(INDEXER_MC_CAN_ID_LFT, MotorType.kBrushless); 

        indexerMtrCtrlLFT_BLT_WHEEL.restoreFactoryDefaults();
    
       

    
        indexerTimer = new Timer();

        flipperTimer = new Timer();


        beamBreak = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);

     //   LimitSwitchLFT = new DigitalInput(LIMIT_SWITCH_DIO_PORT_LFT);
     //   LimitSwitchRGT = new DigitalInput(LIMIT_SWITCH_DIO_PORT_RGT);

        
        m_encoder = indexerMtrCtrlFLIPPER.getEncoder();
        m_encoder.setPositionConversionFactor(60);
        m_encoder.setPosition(0);
        

        
        startIndexerThread();

      
    }

    public void startIndexerThread() 
    {
        Indexer = new Thread(() ->
        {
            while(true)
            {   
                
                if(objectReady = true) //needs to be set to false in the elevator class when object is on the elevator
                {
                collectColorValues();
                collectBeamBreakValues();
                collectColorSensorDistanceValues();
                collectLimitSwitchValues();
                }
                /* 
                if(Robot.intake.intakeActive == true) //needs intakeActive from intake class set on when the rollers are active
                {
                    runIndexerBelt();
                    runSideBelts();


                    indexerTime = indexerTimer.get();
                    indexerTimer.reset();
                    indexerTimer.start();
                }
                else
                {
                
                }
                if(indexerTime > 7)
                {
                    stopbelt();
                    stopSideBelts();
                }
                
                if(limitSwitchPressedLFT && limitSwitchPressedRGT)
                {
                    stopbelt();
                    stopSideBelts();
                    
                    objectReady = true; 
                }

                if(CubeDetectedMethod())
                {
                    colorSelected = "cube";
                    objectReady = true;
                }

                if(ConeDetectedMethod())
                {
                    colorSelected = "cone";
                }
                if(beamBreakContinuity == false)
                {
                    flipperActive = true;
                }
                if(flipperActive == true)
                {
                    if(m_encoder.getPosition() > -100)
                    {
                        indexerMtrCtrlFLIPPER.set(-0.1);
                    }
                    else
                    {
                        indexerMtrCtrlFLIPPER.set(0.0);
                        flipperActive = false;
                        flipperDeactivating = true;
                    }
                }
                if(flipperDeactivating == true)
                {
                    if(m_encoder.getPosition() < -20)
                    {
                        indexerMtrCtrlFLIPPER.set(0.1);
                    }
                    else
                    {
                        indexerMtrCtrlFLIPPER.set(0.0);
                        flipperDeactivating = false;
                        objectReady = true;
                        stopbelt();
                        stopSideBelts();
                    }
                }
                */
            Timer.delay(0.02);
            }
            
         });
        Indexer.start();
    }
    


   public void collectColorValues() 
   {

        RedValue    = detectedColor.red;
        GreenValue  = detectedColor.green;
        BlueValue   = detectedColor.blue;

        detectedColor    = m_colorSensor.getColor();
    }

    public void collectColorSensorDistanceValues()
    {
        IR               = m_colorSensor.getIR();
        proximity        = m_colorSensor.getProximity();
    }

   public void collectBeamBreakValues() 
   {
        beamBreakContinuity = beamBreak.get();

    }
    public void collectLimitSwitchValues() 
    {

        limitSwitchPressedRGT = LimitSwitchRGT.get();
        limitSwitchPressedLFT = LimitSwitchLFT.get();
    }

    public void runIndexerBeltREV() {
        indexerMtrCtrlFRNT.set(INDEXER_FRNT_SPEED);
        indexerMtrCtrlBACK.set(INDEXER_BACK_SPEED);
    }

    public void runIndexerBelt() {
        indexerMtrCtrlFRNT.set(-INDEXER_FRNT_SPEED);
        indexerMtrCtrlBACK.set(-INDEXER_BACK_SPEED);
    }

    public void runIndexerBeltFast(){
        indexerMtrCtrlFRNT.set(INDEXER_FRNT_SPEED_FAST);
        indexerMtrCtrlBACK.set(INDEXER_BACK_SPEED);
    }

    public void stopbelt() {
        indexerMtrCtrlFRNT.stopMotor();
    }

    public void runSideBelts() {
        indexerMtrCtrlRGT_BLT_WHEEL.set(INDEXER_SIDE_WHEEL_SPEED);
        indexerMtrCtrlLFT_BLT_WHEEL.set(-INDEXER_SIDE_WHEEL_SPEED);
    }
    public void stopSideBelts() {
        indexerMtrCtrlRGT_BLT_WHEEL.set(0);
        indexerMtrCtrlLFT_BLT_WHEEL.set(0);
    }


    public void SmartDashboardIndexer()
    {
        SmartDashboard.putString("object detected", colorSelected);


    }
    public void SmartDashboardIndexer_Debug()
    {
        SmartDashboard.putNumber("TraceID", finalStateINT);
        SmartDashboard.putNumber("Encoder Position", m_encoder.getPosition());
        SmartDashboard.putNumber("Encoder getpositionfactor", m_encoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putBoolean("BeamBreak", beamBreakContinuity);
    }

    public void DataCollectionINDEXER()
    {
        if (DataCollection.LOG_ID_INDEXER == DataCollection.chosenDataID.getSelected())
        {
            data = new CatzLog(Robot.currentTime.get(),detectedColor.red,detectedColor.blue,detectedColor.green,proximity,finalStateINT,0,0,0,0,0,0,0,0,0,1);  
            Robot.dataCollection.logData.add(data);
        }
    }

    public boolean objectDetectedMethod()
    {

        if (proximity >= PROXIMITY_DISTANCE_THRESHOLD){
            objectDetected = true;
        }
        else
        {
            objectDetected = false;
        }
        return objectDetected;
    }
    public boolean ConeDetectedMethod()
    {
        if (RedValue <= CUBE_RED_THRESHOLD && GreenValue <= CUBE_GREEN_THRESHOLD && BlueValue >= CUBE_BLUE_THRESHOLD)
        {
            isConeDetected = true;
        }
        else
        {
            isConeDetected = false;
        }
        return isConeDetected;
    }

    public boolean CubeDetectedMethod()
    {
        if (BlueValue <= CONE_BLUE_THRESHOLD)
        {
            isCubeDetected = true;
        }
        else
        {
            isCubeDetected = false;
        }
        return isCubeDetected;
    }
    
}