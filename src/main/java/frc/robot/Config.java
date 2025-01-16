package frc.robot;

public final class Config {
    public static final double LOOP_PERIOD_SEC = 0.02;

    public static final boolean IS_LIVE_DEBUG = false;

    public static final Mode MODE = Mode.SIM;

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }
}
