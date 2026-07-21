package com.roboviz;

public class Axis {

    public float x;
    public float y;
    public float z;
	
	public Axis(float X,float Y,float Z){
		this.x=X;
		this.y=Y;
		this.z=Z;
	}
    
	@Override
    public String toString() {
        return "axis: (" + x + ", " + y + ", " + z + ")";
    }

}
