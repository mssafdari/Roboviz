package com.kinematics;
import com.math.se3algebra;
import com.math.Matrix;
import java.util.ArrayList;
import com.math.se3group;

public class dynConfig
{ 
	public ArrayList<se3algebra> Alist;
	public Matrix A6nx6n;
	//public ArrayList<se3group> Tlist;
	public Matrix Ltheta6nx6n;
	public Matrix G6nx6n;
	
	public dynConfig(ArrayList<se3algebra> Al, Matrix A,Matrix L,Matrix G){
		A6nx6n=A;
		Ltheta6nx6n=L;
		G6nx6n=G;
		Alist=Al;
	}
}
