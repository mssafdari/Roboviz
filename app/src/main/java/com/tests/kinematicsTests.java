package com.tests;
import com.kinematics.forwardKinematics;
import com.math.se3group;
import com.math.Matrix;
import com.math.Vector;
import com.math.se3algebra;
import android.renderscript.Matrix2f;
import java.util.ArrayList;
import com.kinematics.IKresult;
import com.kinematics.inversekinematics;
import com.kinematics.jacobianBuilder;

public class kinematicsTests extends BaseTest
{
    private static final String TAG = "FK tests";
    private String errorReport="";
	
	private double[][] m,ans,s1,s2,s3,m2,screwList;
	double t1,t2,l1,l2;
	se3algebra se1,se2,se3;
	Matrix jacS;
	ArrayList<se3algebra> S_list;
	
    @Override
    protected String getTestTag()
    {
        return TAG;
    }

    @Override
    protected String[] getTestNames()
    {
        return new String[]{
            "testFKinBody",
            "testFKinSpace",
			"testIKinBody",
			"testIKinSpace",
			"testJacobianBody",
			"testJacobianSpace"
        };
    }

    @Override
    protected boolean[] runTests()
    {
        return new boolean[]{
			testFKinBody(),
            testFKinSpace(),
			testIKinBody(),
			testIKinSpace(),
			testJacobianBody(),
			testJacobianSpace()
        };
    }

    @Override
    protected String getError()
    {
        return errorReport;
    }

	private void initFK_IK(){
		m=new double[][]{
			{1,0,0,6},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1}
		};
		ans=new double[][]{
			{0,1,0,4},
			{-1,0,0,2},
			{0,0,1,0},
			{0,0,0,1}
		};
		s3=new double[][]{
			{0,-1,0,0},
			{1,0,0,-4},
			{0,0,0,0},
			{0,0,0,0}
		};
		s2=new double[][]{
			{0,-1,0,0},
			{1,0,0,-2},
			{0,0,0,0},
			{0,0,0,0}
		};
		s1=new double[][]{
			{0,-1,0,0},
			{1,0,0,0},
			{0,0,0,0},
			{0,0,0,0}
		};
	    se1=new se3algebra(new Matrix(s1));
		se2=new se3algebra(new Matrix(s2));
		se3=new se3algebra(new Matrix(s3));
	}
	
	private void initJacobian(){
		t1=Math.PI/6;t2=Math.PI/3;l1=2;l2=2;
	    jacS=new Matrix(jsDataFcn(t1,t2,l1,l2));
	    screwList=jsDataFcn(0,0,l1,l2);
	    S_list=se3algebra.doubleToSe3algebraArray(screwList);
		m2=new double[][]{
			{0,-1,0,2},
			{1,0,0,2},
			{0,0,1,0},
			{0,0,0,1}
		};
	}
	private double[][] jsDataFcn(double t1,double t2,double l1,double l2){
		double[][] jsData=new double[][]{
			{0,0,0,0},
			{0,0,0,0},
			{1,1,1,0},
			{0,l1*Math.sin(t1),l1*Math.sin(t1)+l2*Math.sin(t1+t2),0},
			{0,-1.0*l1*Math.cos(t1),-1.0*(l1*Math.cos(t1)+l2*Math.cos(t1+t2)),0},
			{0,0,0,1}
		};
		return jsData;
	}
	
    private boolean testFKinBody()
    {
        try
        {
			initFK_IK();
			
			ArrayList<se3algebra> Blist=new ArrayList<se3algebra>();
			se3group M=new se3group(new Matrix(m));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(se1.matrix)));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(se2.matrix)));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(se3.matrix)));
			double[][] thetaL=new double[][]{{0},{0},{Math.PI / 2}};
			Vector thetaList=new Vector(thetaL);
            se3group ee= forwardKinematics.FKinBody(M, Blist, thetaList);
            se3group answer=new se3group(new Matrix(ans));
            return answer.matrix.isEqual(ee.matrix);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from FKinBody\n";
            return false;
        }
    }

    private boolean testFKinSpace()
    {
        try
        {
			initFK_IK();
			ArrayList<se3algebra> Slist=new ArrayList<se3algebra>();
			Slist.add(se1);
			Slist.add(se2);
			Slist.add(se3);
			double[][] thetaL=new double[][]{{0},{0},{Math.PI / 2}};
			se3group M=new se3group(new Matrix(m));
			Vector thetaList=new Vector(thetaL);
            se3group ee= forwardKinematics.FKinSpace(M, Slist, thetaList);
            se3group answer=new se3group(new Matrix(ans));
            return answer.matrix.isEqual(ee.matrix);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from FKinSpace\n";
        }
        return false;
    }
	private boolean testIKinBody()
    {
        try
        {
			initFK_IK();

			ArrayList<se3algebra> Blist=new ArrayList<se3algebra>();
			se3group M=new se3group(new Matrix(m));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(se1.matrix)));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(se2.matrix)));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(se3.matrix)));
			double[][] thetaL=new double[][]{{0},{0},{Math.PI / 2}};
			Vector thetaList=new Vector(thetaL);
            se3group ee= forwardKinematics.FKinBody(M, Blist, thetaList);
			IKresult thetaListInv=inversekinematics.IKinBody(Blist,M,ee,new Vector(thetaList.size()));
			return thetaListInv.thetaList.isEqual(thetaList);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from IKBody\n";
        }
        return false;
    }
	
	private boolean testIKinSpace()
    {
        try
        {
			initFK_IK();
			ArrayList<se3algebra> Slist=new ArrayList<se3algebra>();
			Slist.add(se1);
			Slist.add(se2);
			Slist.add(se3);
			double[][] thetaL=new double[][]{{0},{0},{Math.PI / 2}};
			se3group M=new se3group(new Matrix(m));
			Vector thetaList=new Vector(thetaL);
            se3group ee= forwardKinematics.FKinSpace(M, Slist, thetaList);
            IKresult thetaListInv=inversekinematics.IKinSpace(Slist,M,ee,new Vector(thetaList.size()));
			return thetaListInv.thetaList.isEqual(thetaList);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from IKSpace\n";
        }
        return false;
    }
	
	private boolean testJacobianBody()
    {
        try
        {
			initJacobian();
			ArrayList<se3algebra> Blist=new ArrayList<se3algebra>();
			se3group M=new se3group(new Matrix(m2));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(S_list.get(0).matrix)));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(S_list.get(1).matrix)));
			Blist.add(new se3algebra(se3group.adjoint(M).adj.multiply(S_list.get(2).matrix)));
			double[][] thetaL=new double[][]{{Math.PI/6},{Math.PI/3},{0},{0}};
			jacobianBuilder.JacobianBody(Blist,new Matrix(thetaL));
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from IKSpace\n";
        }
        return false;
    }
	
	private boolean testJacobianSpace()
    {
        try
        {
			initJacobian();
			double[][] thetaL=new double[][]{{Math.PI/6},{Math.PI/3},{0},{0}};
			jacobianBuilder.JacobianSpace(S_list,new Matrix(thetaL));
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from IKSpace\n";
        }
        return false;
    }

}
