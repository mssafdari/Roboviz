package com.math;

public class omgVel{
    public so3algebra omg;
    public Vector3 vel;
    public omgVel(Vector3 o,Vector3 v){
        omg=so3algebra.vecToSo3(o);
        vel=new Vector3(v.x,v.y,v.z);
    }
    public omgVel(){
        omg=new so3algebra( Matrix.zeros(3,3));
        vel= new Vector3(0,0,0);
    }
}
