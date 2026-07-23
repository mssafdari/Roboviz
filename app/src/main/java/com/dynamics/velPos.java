package com.dynamics;
import java.util.ArrayList;
import com.math.Matrix;

public class velPos
{
	private Matrix thetaList;
	private Matrix dthetaList;
	
	public velPos(Matrix tList,Matrix dtList){
		if(tList.hasSameDimensions(dtList)){
		thetaList=tList;
		dthetaList=dtList;
		}else{
			throw new IllegalArgumentException("theta and dtheta lists should have same dimentions");
		}
	}
	public Matrix getThetaList(){
		return thetaList;
	}
	public Matrix getDThetaList(){
		return dthetaList;
	}
	
}
