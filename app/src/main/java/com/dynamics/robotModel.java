package com.dynamics;
import java.util.ArrayList;
import com.math.se3group;
import com.math.se3algebra;
import com.math.spatialInertia;

public class robotModel
{
	public ArrayList<se3group> Mlist;
	public ArrayList<spatialInertia> Glist;
	public ArrayList<se3algebra> Slist;
	
	public robotModel(ArrayList<se3group> M,ArrayList<spatialInertia> G,ArrayList<se3algebra> S){
		Mlist=M;
		Glist=G;
		Slist=S;
	}
}
