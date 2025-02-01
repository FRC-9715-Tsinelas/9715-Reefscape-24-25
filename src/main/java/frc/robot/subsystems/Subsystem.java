package frc.robot.subsystems;

public interface Subsystem {

    // https://docs.google.com/document/d/1srF4KuMOi9FSqolFLZMJzhqANaGJcVNHETLWjoWdyeM/edit?tab=t.0
    public void periodic();
    public void teleopPeriodic();
    public void reset();
    public void stop();
    public void outputTelemetry();
    public void writeToLog();
    public void simulationPeriodic();
}
