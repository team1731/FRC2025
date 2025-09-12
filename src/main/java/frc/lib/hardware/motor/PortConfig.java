package frc.lib.hardware.motor;

/**
 * Helper class that holds the basic configuration for a motor
 */
public class PortConfig {
    public final String kBus;
    public final int kPort;
    public final boolean kInverted;

    public PortConfig(String bus, int port, boolean inverted) {
        this.kBus = bus;
        this.kPort = port;
        this.kInverted = inverted;
    }

    public PortConfig(String bus, int port) {
        this(bus, port, false);
    }

    public PortConfig(int port, boolean inverted) {
        this("", port, inverted);
    }

    public PortConfig(int port) {
        this("", port, false);
    }
}