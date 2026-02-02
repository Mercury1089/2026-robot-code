package frc.robot.sensors;

public class ProximitySensor {

    private double triggerValue;

    public ProximitySensor(int canID, double trigVal) {
        // TODO: Re-implement with LaserCAN

        this.triggerValue = trigVal;
    }

    public boolean isTriggered() { // if the reported value is less than your trigger value, it returns true
        throw new UnsupportedOperationException("Unimplemented method 'isTriggered'");
    }
}
