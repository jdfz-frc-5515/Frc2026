package frc.robot.utils.Models;

public class CandidateTagInfo {
    public AprilTagCoordinate tag;
    public double distance; // meters
    public double angleToRobot; // radians
    public double turnRadian; // radians
    public CandidateTagInfo(AprilTagCoordinate tag, double distance, double angleToRobot,double turnRadian) {
        this.tag = tag;
        this.distance = distance;
        this.angleToRobot = angleToRobot;
        this.turnRadian = turnRadian;
    }
}
