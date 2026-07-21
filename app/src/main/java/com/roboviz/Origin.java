package com.roboviz;

public class Origin {

    public float x;
    public float y;
    public float z;

    public float roll;
    public float pitch;
    public float yaw;
	
	public Origin(float X,float Y,float Z,float Roll,float Pitch,float Yaw){
		this.x=X;
		this.y=Y;
		this.z=Z;
		this.roll=Roll;
		this.pitch=Pitch;
		this.yaw=Yaw;
	}
	
    
    
	@Override
    public String toString() {
        return "Position: (" + x + ", " + y + ", " + z + ") RPY: (" + roll + ", " + pitch + ", " + yaw + ")";
    }
}
