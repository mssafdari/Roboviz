package com.dynamics;
import java.util.ArrayList;
import com.math.Matrix;
import com.math.Vector;

public class velPos
{
	private Vector thetaList;
	private Vector dthetaList;
	
	public velPos(Vector tList,Vector dtList){
		if(tList.hasSameDimensions(dtList)){
		thetaList=tList;
		dthetaList=dtList;
		}else{
			throw new IllegalArgumentException("theta and dtheta lists should have same dimentions");
		}
	}
	public Vector getThetaList(){
		return thetaList;
	}
	public Vector getDThetaList(){
		return dthetaList;
	}
	
}
