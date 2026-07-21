package com.math;

class axis6theta{
	Vector6 axis;
	double theta;
}

public class Vector6 extends Matrix {
    public Vector3 omega;
	public Vector3 velocity;
    
    public Vector6(){
        super(6, 1);
        omega=new Vector3();
        velocity = new Vector3();
    }

    public Vector6(Vector3 omg,Vector3 vel) {
        super( to2D(omg,vel));
		omega=omg;
		velocity=vel;
    }

	public Vector6(Matrix vec6) {
        super(vec6.data);
		omega=new Vector3(vec6.getSubMatrix(0,0,3,1));
		velocity=new Vector3(vec6.getSubMatrix(3,0,3,1));
    }
	
	private static double[][] to2D(Vector3 omg,Vector3 vel){
		double[][] data2d = new double[6][1];
		data2d[0][0] = omg.x;
		data2d[1][0] = omg.y;
		data2d[2][0] = omg.z;
		data2d[3][0] = vel.x;
		data2d[4][0] = vel.y;
		data2d[5][0] = vel.z;
		return data2d;
		}
		
	public Vector6 screwToAxis(Vector3 q,Vector3 s,double h){
		Vector3 normalS = s.normalize();
		Vector3 vel = new Vector3(Matrix.scalarMulti(h,s).subtract( s.cross(q)));
		Vector6 axis = new Vector6(normalS,vel);
		return axis;
	}
	
	public static axis6theta axisAng6(Vector6 expc6){
		axis6theta at=new axis6theta();
		at.theta =expc6.omega.norm();
		if(at.theta==0){
            at.theta = expc6.velocity.norm();
			at.axis =new Vector6(new Vector3(0,0,0),
            new Vector3(expc6.velocity.x/at.theta,expc6.velocity.y/at.theta,expc6.velocity.z/at.theta));
		}
		else{
			at.axis =new Vector6(expc6.omega.normalize(),new Vector3(expc6.velocity.x/at.theta,
															 expc6.velocity.y/at.theta,expc6.velocity.z/at.theta));
		}
		return at;
	}
	
}
