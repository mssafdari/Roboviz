package com.math;


public class rotPos{
    public so3group rot;
    public Vector3 pos;
    public rotPos(so3group r,Vector3 v){
        rot=new so3group(r.matrix);
        pos=new Vector3(v.x,v.y,v.z);
    }
    public rotPos(){
        rot=new so3group( Matrix.identity(3));
        pos= new Vector3(0,0,0);
    }
}
