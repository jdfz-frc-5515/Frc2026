package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Global {
    static int dt = -1;
    static int MAX_DT = 60;
    private static final StringPublisher infoPublisher = NetworkTableInstance.getDefault()
        .getStringTopic("INFO").publish();
    public static void update() {
        if (dt > 0) {
            dt--;
            if (dt < 0) {
                infoPublisher.set("00000");
            }
        }
    } 

    // public static void onFoundAp() {
    //     infoPublisher.set("11111");
    //     dt = MAX_DT;
    // }

    public static void onRotationAdjusted() {
        infoPublisher.set("22222");
        dt = MAX_DT;
    }
}
