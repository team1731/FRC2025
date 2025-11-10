package frc.lib.frc1731.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * A class that maps an enum constant to a value. Useful for logging and used in an automated manner within subsystems
 */
public class SubsystemDataMap<E extends Enum<E>> {
    private Map<E, Supplier<?>> dataMap =   new HashMap<E, Supplier<?>>();

    public SubsystemDataMap() {}

    /**
     * Maps an enum constant to a value of any type
     */
    public <T> void map(E key, T value) {
        dataMap.put(key, () -> value);
    }

    /**
     * Maps an enum constant to a value of any type if the condition is true, otherwise maps it to an alternative value.
     */
    public <T> void mapIf(E key, T value, T alternative, boolean condition) {
        if (condition) {
            dataMap.put(key, () -> value);
        } else {
            dataMap.put(key, () -> alternative);
        }
    }

    /**
     * Pulls the value associated with the given enum constant key in its native type
     */
    @SuppressWarnings("unchecked")
    public <T> T pull(E key) {
        try {
            return (T)dataMap.get(key).get();
        } catch (Exception e) {
            return null;
        }
    }
}