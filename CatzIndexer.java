package frc.Mechanisms;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

public class CatzIndexer {
    //          SPARKMAX DEFS
    public CANSparkMax inDexerMtrCtrl;

    private final int INDEXER_MC_CAN_ID        = 40;
    private final int INDEXER_MC_CURRENT_LIMIT = 60;

    public final double INDEXER_TESTSPEED = 0.2;
    public final double INDEXER_TESTSPEEDOFF = 0.0;

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

    public boolean objectReady = false;
    
    private Thread Indexer;

    
    //                      End of definitions
    CatzIndexer() {

        inDexerMtrCtrl = new CANSparkMax(INDEXER_MC_CAN_ID, MotorType.kBrushless); 

        inDexerMtrCtrl.restoreFactoryDefaults();
        inDexerMtrCtrl.setIdleMode(IdleMode.kBrake);
        inDexerMtrCtrl.setSmartCurrentLimit(INDEXER_MC_CURRENT_LIMIT);


        beamBreakLFT = new DigitalInput(BEAM_BREAK_DIO_PORT_LFT);
        beamBreakRGT = new DigitalInput(BEAM_BREAK_DIO_PORT_RGT);
        
        
        
    startIndexerThread();
    }
     void startIndexerThread() {
    
        Indexer = new Thread(() ->
        {
        
            while(true){

               collectColorBeambreakValues();
               

                if (proximity >= PROXIMITY_DISTANCE_THRESHOLD){
                    if(RedValue <= CUBE_RED_THRESHOLD && GreenValue <= CUBE_GREEN_THRESHOLD && BlueValue >= CUBE_BLUE_THRESHOLD)
                    {// cube
                        setBeltFWD();
                        Timer.delay(2);
                        stopbelt();
                        objectReady = true;
                    }
                   
                    if(BlueValue <= CONE_BLUE_THRESHOLD)
                    {//cone
                        if((beamBreakLFTVALUE == true) || (beamBreakRGTVALUE == true))
                        {
                            setBeltFWD();
                            Timer.delay(2);
                            startFlipper();
                            Timer.delay(1);
                            setBeltREV();
                            Timer.delay(2);
                            stopbelt();
                            objectReady = true;
                            
                        }
                        else if((beamBreakRGTVALUE == false) && (beamBreakRGTVALUE == false))//if already in correct position
                        {
                            setBeltFWD();
                            Timer.delay(2);
                            stopbelt();
                            objectReady = true;
                        }
                    }
                }
                else
                {

                }
                
                SmartDashboardIndexer();
            }
        });
        Indexer.start();
    }

    void collectColorBeambreakValues() {
        detectedColor    = m_colorSensor.getColor();
        IR               = m_colorSensor.getIR();
        proximity        = m_colorSensor.getProximity();

        RedValue    = detectedColor.red;
        GreenValue  = detectedColor.green;
        BlueValue   = detectedColor.blue;

        beamBreakLFTVALUE = beamBreakLFT.get();
        beamBreakRGTVALUE = beamBreakRGT.get();
        

    }

    void setVerticalMotorsOn() {

    }

    void setBeltREV() {

    }
    void setBeltFWD() {
        inDexerMtrCtrl.set(INDEXER_TESTSPEED);
    }
    void stopbelt() {
        inDexerMtrCtrl.set(INDEXER_TESTSPEEDOFF);
    }

    void startVerticalMotors() {


    }

    void startFlipper(){

    }

    void SmartDashboardIndexer(){
        SmartDashboard.putNumber("Proximity", proximity);
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);

    }

    
}
