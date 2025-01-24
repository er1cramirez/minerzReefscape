package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedStripe extends SubsystemBase {
    // CAN Configuration
    private static final int TEJUINO_MANUFACTURER = 8;
    private static final int TEJUINO_DEVICE_TYPE = 10;
    private static final int TEJUINO_LED_API_ID = 0;
    private static final int DEVICE_NUMBER = 0;
    
    // LED Strip IDs
    public static final int ONBOARD_LEDS = 0;
    public static final int EXTERNAL_LEDS = 1;
    public static final int EXTERNAL2_LEDS = 2;
    
    private int canHandle;
    private LedState currentState = LedState.OFF;
    
    public enum LedState {
        OFF, READY, ENABLED, COLLECTING, ERROR, SCORING
    }
    
    public LedStripe() {
        initializeCAN();
    }
    
    private void initializeCAN() {
        canHandle = CANAPIJNI.initializeCAN(
            TEJUINO_MANUFACTURER, 
            DEVICE_NUMBER, 
            TEJUINO_DEVICE_TYPE
        );
    }
    
    public void setState(LedState state) {
        if (state != currentState) {
            currentState = state;
            updateLeds();
        }
    }
    
    private void updateLeds() {
        switch (currentState) {
            case OFF:
                turnOffAllLeds(EXTERNAL_LEDS);
                break;
            case READY:
                setAllLedsGreen(EXTERNAL_LEDS);
                break;
            case ENABLED:
                setAllLedsBlue(EXTERNAL_LEDS);
                break;
            case COLLECTING:
                setRainbowEffect(EXTERNAL_LEDS);
                break;
            case ERROR:
                setAllLedsRed(EXTERNAL_LEDS);
                break;
            case SCORING:
                setAllLedsPurple(EXTERNAL_LEDS);
                break;
        }
    }
    
    // Core LED Control Methods
    private void writeCANPacket(byte[] data) {
        CANAPIJNI.writeCANPacket(canHandle, data, TEJUINO_LED_API_ID);
    }
    
    private void setAllLedsColor(int strip, int red, int green, int blue) {
        byte[] data = new byte[8];
        data[2] = (byte) red;
        data[3] = (byte) green;
        data[4] = (byte) blue;
        data[6] = 1;
        data[7] = (byte) strip;
        writeCANPacket(data);
    }
    
    private void setAllLedsRed(int strip) {
        setAllLedsColor(strip, 255, 0, 0);
    }
    
    private void setAllLedsGreen(int strip) {
        setAllLedsColor(strip, 0, 255, 0);
    }
    
    private void setAllLedsBlue(int strip) {
        setAllLedsColor(strip, 0, 0, 255);
    }
    
    private void setAllLedsPurple(int strip) {
        setAllLedsColor(strip, 153, 51, 255);
    }
    
    private void turnOffAllLeds(int strip) {
        setAllLedsColor(strip, 0, 0, 0);
    }
    
    private void setRainbowEffect(int strip) {
        byte[] data = new byte[8];
        data[5] = 1;
        data[7] = (byte) strip;
        writeCANPacket(data);
    }
}
