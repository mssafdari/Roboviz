package com.math;

public class so3opslog
{
    public so3algebra so3alg;
    public String log;
    public so3group so3g;
    public so3opslog(){
        so3alg= new so3algebra(Matrix.zeros(3,3));
        so3g = new so3group(Matrix.identity(3));
        log="";
    }
}
