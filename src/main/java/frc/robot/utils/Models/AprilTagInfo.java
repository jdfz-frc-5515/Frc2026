package frc.robot.utils.Models;

import frc.robot.Library.team19725.Point3D;

public class AprilTagInfo {
    public int id;
    public double x; // meters
    public double y; // meters
    public double z; // meters
    Point3D nVector; // 法线方向单位向量

    public AprilTagInfo(int id, double x, double y, double z, double nx, double ny, double nz) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.z = z;
        this.nVector = new Point3D(nx, ny, nz);
    }
    
}
