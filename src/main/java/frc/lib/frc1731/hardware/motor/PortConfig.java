package frc.lib.frc1731.hardware.motor;

import com.ctre.phoenix6.signals.InvertedValue;

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

    public PortConfig(String bus, int port, InvertedValue inverted) {
        this(bus, port, inverted != InvertedValue.Clockwise_Positive);
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