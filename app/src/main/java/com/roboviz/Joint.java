package com.roboviz;
import android.os.Parcel;
import android.os.Parcelable;
import java.io.Serializable;
import com.math.Vector3;
import android.net.wifi.aware.PublishDiscoverySession;

public class Joint implements Parcelable{

    public String name;

    public String type;

    public String parent;

    public String child;

    public Origin origin;

    public Axis axis;
	
	private double defaultPosition;
    private double minPosition;
    private double maxPosition;
    private double currentPosition;
    
    public Vector3 location;
	
	public Joint(){
		this.axis = new Axis(0,0,1);
		this.origin = new Origin(0,0,0,0,0,0);
	}
	public Joint(String name, double defaultPos, double minPos, double maxPos, String type) {
        this.name = name;
        this.defaultPosition = defaultPos;
        this.minPosition = minPos;
        this.maxPosition = maxPos;
        this.currentPosition = defaultPos;
        this.type = type;
		this.axis = new Axis(0,0,1);
		this.origin = new Origin(0,0,0,0,0,0);
    }
    
    protected Joint(Parcel in) {
        name = in.readString();
        defaultPosition = in.readDouble();
        minPosition = in.readDouble();
        maxPosition = in.readDouble();
        currentPosition = in.readDouble();
        type = in.readString();
    }
    
    public void reset(){
        currentPosition=defaultPosition;
    }
    public double getPos(){
        return currentPosition;
    }
    
    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeString(name);
        dest.writeDouble(defaultPosition);
        dest.writeDouble(minPosition);
        dest.writeDouble(maxPosition);
        dest.writeDouble(currentPosition);
        dest.writeString(type);
    }
    
    @Override
    public int describeContents() {
        return 0;
    }
    
    public static final Creator<Joint> CREATOR = new Creator<Joint>() {
        @Override
        public Joint createFromParcel(Parcel in) {
            return new Joint(in);
        }
        
        @Override
        public Joint[] newArray(int size) {
            return new Joint[size];
        }
    };

	public double getDefaultPosition() { return defaultPosition; }
    public double getMinPosition() { return minPosition; }
    public double getMaxPosition() { return maxPosition; }
    public double getCurrentPosition() { return currentPosition; }
    public void setCurrentPosition(double position) { this.currentPosition = position; }
}
