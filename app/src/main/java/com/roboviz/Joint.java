package com.roboviz;


public class Joint {

    public String name;

    public String type;

    public String parent;

    public String child;

    public Origin origin;

    public Axis axis;
	
	public Joint(){
		this.axis = new Axis(0,0,1);
		this.origin = new Origin(0,0,0,0,0,0);
	}

}
