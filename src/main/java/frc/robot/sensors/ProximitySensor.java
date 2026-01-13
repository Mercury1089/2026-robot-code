package frc.robot.sensors;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

public class ProximitySensor extends Canandcolor {

    private double triggerValue;

    public ProximitySensor(int canID, double trigVal) {
        super(canID);
        resetFactoryDefaults();

        this.triggerValue = trigVal;
    }

    public boolean isTriggered() { // if the reported value is less than your trigger value, it returns true
        return isConnected() && getProximity() < triggerValue; // works because of shortcircuit eval
    }
}
