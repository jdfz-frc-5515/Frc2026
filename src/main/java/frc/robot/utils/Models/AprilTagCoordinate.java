package frc.robot.utils.Models;

import frc.robot.Library.team19725.Point3D;

public class AprilTagCoordinate {
    public int id;
    public double UnReadableDistance;
    public Point3D position; // 三维坐标
    public Point3D nVector; // 法线方向单位向量

    public AprilTagCoordinate(int id, double x, double y, double z, double nx, double ny, double nz) {
        this.id = id;
        this.position = new Point3D(x, y, z);
        this.nVector = new Point3D(nx, ny, nz);
        UnReadableDistance = 0;
    }
    public AprilTagCoordinate(int id, double x, double y, double z, double nx, double ny, double nz, double UnReadableDistance){
        this.id = id;
        this.position = new Point3D(x, y, z);
        this.nVector = new Point3D(nx, ny, nz);
        this.UnReadableDistance = UnReadableDistance;
    }
    public AprilTagCoordinate(int id, Point3D position, Point3D nVector) {
        this.id = id;
        this.position = position;
        this.nVector = nVector;
        UnReadableDistance = 0;
    }
    
}
