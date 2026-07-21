package com.math;

public class se3opslog
{
    public se3algebra se3alg;
    public String log;
    public se3group se3g;
    public Matrix adj;
    public se3opslog(){
        se3alg= new se3algebra(Matrix.zeros(3,3));
        se3g = new se3group(Matrix.identity(3));
        log="";
        adj=Matrix.identity(6);
    }
}
