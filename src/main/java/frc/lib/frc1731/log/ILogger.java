package frc.lib.frc1731.log;

public interface ILogger {
    public void suspend();
	public void resume();
	public boolean isSuspended();
    public void add(Object value);
    public void flush();
}