package com.tests;
import com.kinematics.forwardKinematics;
import com.math.se3group;
import com.math.Matrix;
import com.math.Vector;
import com.math.se3algebra;
import java.util.ArrayList;
import com.kinematics.inversekinematics;
import com.kinematics.jacobianBuilder;
import com.math.Vector6;
import com.math.Vector3;
import java.io.StringWriter;
import java.io.PrintWriter;
import com.roboviz.MainActivity;

public class kinematicsTests extends BaseTest
{
    private static final String TAG = "FK tests";
    private String errorReport="";
	private static final String el="\n";
	
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
			{0,-1,0,4},
			{1,0,0,2},
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
		for(int i=0;i<S_list.size();i++){
			errorReport+="s("+i+")"+S_list.get(i).matrix.toString()+el;
		}
		errorReport+="screwlist="+el+new Matrix(screwList).toString()+el;
		m2=new double[][]{
			{1,0,0,4},
			{0,1,0,0},
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
			se3group Minv=new se3group(M.matrix.inverse());
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(se1)))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(se2)))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(se3)))));
			double[][] thetaL=new double[][]{{0},{0},{Math.PI / 2}};
			Vector thetaList=new Vector(thetaL);
			errorReport+="before fk\n";
            se3group ee= forwardKinematics.FKinBody(M, Blist, thetaList);
			errorReport+="after fk\n"+ee.matrix.toString()+el;
            se3group answer=new se3group(new Matrix(ans));
			errorReport+=answer.matrix.toString();
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
			errorReport+="before fk\n";
            se3group ee= forwardKinematics.FKinSpace(M, Slist, thetaList);
			errorReport+="after fk\n"+ ee.matrix.toString()+el;
            se3group answer=new se3group(new Matrix(ans));
			errorReport+=answer.matrix+el;
			errorReport="";
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
            MainActivity.clearLog();
            MainActivity.doLog=true;
			initFK_IK();

			ArrayList<se3algebra> Blist=new ArrayList<se3algebra>();
			se3group M=new se3group(new Matrix(m));
			se3group Minv=new se3group(M.matrix.inverse());
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(se1)))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(se2)))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(se3)))));
            for(int i=0;i<Blist.size();i++){
             errorReport+="b("+i+")"+Blist.get(i).matrix.toString()+el;
             }
			double[][] thetaL=new double[][]{{0},{0},{Math.PI / 2}};
			Vector thetaList=new Vector(thetaL);
            se3group ee= forwardKinematics.FKinBody(M, Blist, thetaList);
			errorReport+="before ik body"+el+ee.matrix.toString()+el;
			Vector thetaListInv=inversekinematics.IKinBody(Blist,M,ee,new Vector(new double[][]{{.1},{.1},{.1}}));
			//errorReport+=thetaListInv.thetaList.toString()+el+thetaListInv.success+el;
			errorReport+="fk_ik="+el+forwardKinematics.FKinBody(M,Blist,thetaListInv).matrix.toString()+el;
            MainActivity.doLog=false;
			return thetaListInv.isEqual(thetaList);
        }
        catch (Exception e)
        {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            String stackTrace = sw.toString();

            errorReport += "Exception: " + e.getMessage() + " from IKBody\n";
            errorReport += "Stack trace:\n" + stackTrace + "\n";
        }
        MainActivity.doLog=false;
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
			errorReport+="before ik space"+el+ee.matrix.toString()+el;
            Vector thetaListInv=inversekinematics.IKinSpace(Slist,M,ee,new Vector(new double[][]{{.1},{.1},{.1}}));
			errorReport+=thetaListInv.toString()+el;
			errorReport+="fk_ik="+el+forwardKinematics.FKinBody(M,Slist,thetaListInv).matrix.toString()+el;
			return thetaListInv.isEqual(thetaList);
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
			double[][] thetaL=new double[][]{{Math.PI/6},{Math.PI/3},{0},{0}};
			se3group T= forwardKinematics.FKinSpace(M,S_list,new Vector(thetaL));
			//errorReport+="T="+el+T.matrix.toString()+el;
			se3group Tinv=new se3group(T.matrix.inverse());
			se3group Minv=new se3group(M.matrix.inverse());
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(S_list.get(0))))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(S_list.get(1))))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(S_list.get(2))))));
			Blist.add(new se3algebra(new Vector6(se3group.adjoint(Minv).multiply(se3algebra.se3ToVec(S_list.get(3))))));
			/*for(int i=0;i<Blist.size();i++){
				errorReport+="b("+i+")"+Blist.get(i).matrix.toString()+el;
			}*/
			
			ArrayList<Vector6> jacBody= jacobianBuilder.JacobianBody(Blist,new Vector(thetaL));
			Matrix jacB=se3group.adjoint(Tinv).multiply(jacS);
			//errorReport+="jacBody="+el+inversekinematics.ArrayListToMatrix(jacBody).toString()+el+"jacB"+el+jacB.toString()+el;
			
			return inversekinematics.ArrayListToMatrix(jacBody).isEqual(jacB);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from jacobian body\n";
        }
        return false;
    }
	
	private boolean testJacobianSpace()
    {
        try
        {
			initJacobian();
			double[][] thetaL=new double[][]{{Math.PI/6},{Math.PI/3},{0},{0}};
			ArrayList<Vector6> jacSpace= jacobianBuilder.JacobianSpace(S_list,new Vector(thetaL));
			//errorReport+="jacSpace="+el+inversekinematics.ArrayListToMatrix(jacSpace).toString()+el+jacS.toString()+el;
			return inversekinematics.ArrayListToMatrix(jacSpace).isEqual(jacS);
        }
        catch (Exception e)
        {
            errorReport += e.getMessage() + "from jacobian space\n";
        }
        return false;
    }

}
