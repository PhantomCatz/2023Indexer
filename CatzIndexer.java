package frc.Mechanisms;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.DataLogger.CatzLog;
import frc.DataLogger.DataCollection;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.DataLogger.DataCollection;

import com.revrobotics.ColorSensorV3;


public class CatzIndexer {
    //          SPARKMAX DEFS
    public CANSparkMax inDexerMtrCtrlFRNT;

    private final int INDEXER_FRNT_MC_CAN_ID        = 1;
    private final int INDEXER_FRNT_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_FRNT_SPEED = 0.4;


    //
    public final double INDEXER_SPEED_OFF = 0.0;
    
    //          SECOND MOTOR DEFS
    public CANSparkMax inDexerMtrCtrlBACK;

    private final int INDEXER_BACK_MC_CAN_ID        = 2;
    private final int INDEXER_BACK_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_BACK_SPEED = 0.2;
    

    public double ydexerShootPower = 0.0;
    public double ydexerSecondShootPower = 0.0;

    //          BEAMBREAK DEFS
    public DigitalInput beamBreakRGT;
    public DigitalInput beamBreakLFT;
    private final int BEAM_BREAK_DIO_PORT_RGT = 1;
    private final int BEAM_BREAK_DIO_PORT_LFT = 2;
    
    //            COLOR SENSOR DEFS
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
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

    private boolean beamBreakLFTVALUE = false;
    private boolean beamBreakRGTVALUE = false;

    Color detectedColor;
    double IR;
    int proximity;

    //          objectdetection
    public boolean objectReady = false;
    
    private boolean objectDetected;

    private boolean objectNotInPosition;

    private boolean coneTestDelay;

    private boolean flipperDelay;
    

    //
    private Thread Indexer;

    private Timer indexerTimer;
    private double indexerTime;

    int traceID;


    //          CONSTANTS
    private double MAX_BELT_TIME = 4;

    public CatzLog data;
    //                      End of definitions

    public CatzIndexer() 
    {
        inDexerMtrCtrlFRNT = new CANSparkMax(INDEXER_FRNT_MC_CAN_ID, MotorType.kBrushless); 

        inDexerMtrCtrlFRNT.restoreFactoryDefaults();
        inDexerMtrCtrlFRNT.setIdleMode(IdleMode.kBrake);
        inDexerMtrCtrlFRNT.setSmartCurrentLimit(INDEXER_FRNT_MC_CURRENT_LIMIT);

        
        inDexerMtrCtrlBACK = new CANSparkMax(INDEXER_BACK_MC_CAN_ID, MotorType.kBrushless); 

        inDexerMtrCtrlBACK.restoreFactoryDefaults();
        inDexerMtrCtrlBACK.setIdleMode(IdleMode.kBrake);
        inDexerMtrCtrlBACK.setSmartCurrentLimit(INDEXER_BACK_MC_CURRENT_LIMIT);

        indexerTimer = new Timer();


        beamBreakLFT = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);
        beamBreakRGT = new DigitalInput(BEAM_BREAK_DIO_PORT_RGT);

        startIndexerThread();
      
    }

    void startIndexerThread() 
    {
        Indexer = new Thread(() ->
        {
            while(true)
            {
                collectColorValues(); 


                if (objectDetected())//if the ir detects an object has passed threshold
                {
                    resetFlags();
                    if(RedValue <= CUBE_RED_THRESHOLD && GreenValue <= CUBE_GREEN_THRESHOLD && BlueValue >= CUBE_BLUE_THRESHOLD)//test color values for cube
                    {
                        indexerTimer.reset();
                        indexerTimer.start();
                        indexerTime = indexerTimer.get();//reset timer

                        setBeltFWD();
                        while(objectNotInPosition == true)
                        {
                            collectBeamBreakValues();
                            if(((beamBreakLFTVALUE == false) || (beamBreakRGTVALUE == false)) || indexerTime == MAX_BELT_TIME)//if a beambreak is broken or if the time limit is reached
                            {
                                stopbelt();
                                objectReady = true; //need to communicate with claw and elevator to set this back to false
                                objectNotInPosition = false;
                                traceID = 3;
                            }
                        }
                    }
                   
                    if(BlueValue <= CONE_BLUE_THRESHOLD)//test color values for cone
                    {
                        indexerTimer.reset();
                        indexerTimer.start();
                        indexerTime = indexerTimer.get();//reset timer
                
                        setBeltFWD();
                        while(objectNotInPosition == true)
                        {
                            if(indexerTime >= MAX_BELT_TIME) //once the cone reaches the next 
                            {
                                stopbelt();
                                objectNotInPosition = false;
                                    while (coneTestDelay = true)
                                    {
                                        if(indexerTime >= 5.5)
                                        {
                                            coneTestDelay = false;
                                            collectBeamBreakValues();

                                            if((beamBreakLFTVALUE == true) || (beamBreakRGTVALUE == true))
                                            {
                                                startFlipper();
                                                while(flipperDelay = true)
                                                {
                                                    if(indexerTime > 7)

                                                        if ((beamBreakRGTVALUE == false) && (beamBreakRGTVALUE == false))
                                                        {
                                                            objectReady = true;
                                                            flipperDelay = false;

                                                            traceID = 1;
                                                        }
                                                }
                                            }      
                                            else if((beamBreakRGTVALUE == false) && (beamBreakRGTVALUE == false))//if already in correct position
                                            {
                                                    objectReady = true;
                                                    traceID = 2;
                                            }
                                        }
                                    }
                            }
                        }
                    }
                }
                
                SmartDashboardIndexer();
                DataCollectionINDEXER();
            }
        });
        Indexer.start();
    }


    void collectColorValues() {
        detectedColor    = m_colorSensor.getColor();
        IR               = m_colorSensor.getIR();
        proximity        = m_colorSensor.getProximity();

        RedValue    = detectedColor.red;
        GreenValue  = detectedColor.green;
        BlueValue   = detectedColor.blue;

       
    }

    void collectBeamBreakValues() {
        beamBreakLFTVALUE = beamBreakLFT.get();
        beamBreakRGTVALUE = beamBreakRGT.get();
    }

    void setBeltREV() {
        inDexerMtrCtrlFRNT.set(-INDEXER_FRNT_SPEED);
        inDexerMtrCtrlBACK.set(-INDEXER_BACK_SPEED);
    }
    void setBeltFWD() {
        inDexerMtrCtrlFRNT.set(INDEXER_FRNT_SPEED);
        inDexerMtrCtrlBACK.set(INDEXER_BACK_SPEED);

    }
    void stopbelt() {
        inDexerMtrCtrlFRNT.set(INDEXER_SPEED_OFF);
    }

    void startFlipper(){

    }

    void resetFlags(){
        coneTestDelay = false;
        objectNotInPosition = true;
        flipperDelay = true;
        
    }

    void SmartDashboardIndexer(){
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

    }

    void DataCollectionINDEXER(){
        if (DataCollection.LOG_ID_INDEXER == DataCollection.logDataID)
        {
            data = new CatzLog(Robot.currentTime.get(),traceID,0,0,0,0,0,0,0,0,0,0,0,0,0,1);  
            Robot.dataCollection.logData.add(data);
        }
    }

    boolean objectDetected(){

        if (proximity >= PROXIMITY_DISTANCE_THRESHOLD){
            objectDetected = true;
        }
        else
        {
            objectDetected = false;
        }
        return objectDetected;
    }
}