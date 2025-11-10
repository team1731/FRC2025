package frc.lib.frc1731;


/**
 * All potential modes of a robot
 */
public enum MatchMode {
    DISABLED,
    TELEOP,
    AUTONOMOUS,
    TEST;

    /**
     * If the current match mode is equal to the indicated mode
     */
    public boolean is(MatchMode mode) {
        return this.equals(mode);
    }

    /**
     * If the current match mode is equal to any of the indicated modes
     */
    public boolean isAny(MatchMode ... modes) {
        for (MatchMode mode : modes) {
            if (this.equals(mode)) return true;
        }
        return false;
    }
}