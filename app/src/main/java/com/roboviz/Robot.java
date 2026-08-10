package com.roboviz;

import java.util.ArrayList;
import java.util.HashMap;
import com.math.se3algebra;
import com.math.Vector6;
import com.math.Vector3;
import com.math.se3group;
import com.math.so3group;
import com.math.Matrix;
import com.math.rotPos;
import com.kinematics.forwardKinematics;
import com.math.Vector;
import java.io.StringWriter;
import java.io.PrintWriter;


public class Robot
{
    public enum jointType
    {
        REVOLUTE,
        PRISMATIC
        }
    public String name;
	public String log="";
	private String nl="\n";

    public ArrayList<Link> links = new ArrayList<>();

    public ArrayList<Joint> joints = new ArrayList<>();

    ArrayList<se3group> Miminusonei= new ArrayList<>();
    public ArrayList<se3group> M0i =new ArrayList<>();
    public ArrayList<se3group> T0i =new ArrayList<>();

    ArrayList<String> jointTypes= new ArrayList<>();
    ArrayList<Vector3> jointAxisses= new ArrayList<>();
    public ArrayList<se3algebra> Slist=new ArrayList<>();
    public ArrayList<se3algebra> Blist=new ArrayList<>();

	public LinkNode root;

	public HashMap<String, LinkNode> nodes = new HashMap<String,LinkNode>();

    public void reset()
    {
        for (Joint joint:joints)
        {
            joint.reset();
        }
    }

	public String robot_init()
    {

		StringBuilder sb = new StringBuilder();

		sb.append("Robot: ").append(name).append('\n');

		sb.append("\nLinks\n");

		for (Link l : links)
			sb.append(l.name).append('\n');

		sb.append("\nJoints\n");

		for (Joint j : joints)
        {

			sb.append(j.name).append('\n');
			sb.append("  ").append(j.origin.toString()).append('\n');
			sb.append("  ").append(j.axis.toString()).append('\n');
			sb.append("  ").append(j.parent).append('\n');
			sb.append("  ").append(j.child).append('\n');
		}

		sb.append("\nTree\n");

		appendTree(root, sb, "");
        int depth=0;
        buildMiminusonei(root, depth);
        buildM0iArray();
		sb.append(log);
		return sb.toString();
	}
	private void appendTree(LinkNode node,
							StringBuilder sb,
							String indent)
    {

		sb.append(indent).append(node.link.name).append('\n');

		for (LinkNode child : node.children)
			appendTree(child, sb, indent + "  ");
	}
    private void buildMiminusonei(LinkNode link, int depth)
    {
        if (link.children != null && !link.children.isEmpty())
        {
            Axis axis=  link.children.get(0).parentJoint.axis;
            Origin origin=  link.children.get(0).parentJoint.origin;
            MainActivity.appendLog("origin=("+origin.x+","+origin.y+","+origin.z+")"+nl,true);
            String type=link.children.get(0).parentJoint.type;
            se3group Mim1i=originTotransform(origin);
            Miminusonei.add(Mim1i);
            jointTypes.add(type);
            jointAxisses.add(axisToVector3(axis));
            buildMiminusonei(link.children.get(0), depth + 1);
        }
    }
    private void buildM0iArray()
    {


        se3group M = new se3group(Matrix.identity(4));
        se3algebra S = new se3algebra(Matrix.zeros(4, 4));
        Vector6 Svec = new Vector6();
        Vector6 Bvec = new Vector6();


        M0i.add(new se3group(M.matrix));  // T0,0

        for (int i = 0; i < Miminusonei.size(); i++)
        {
			log += "Mim1i(" + i + "," + (i + 1) + ") =" + Miminusonei.get(i).matrix.toString() + nl;
            M.matrix = M.matrix.multiply(Miminusonei.get(i).matrix);
            so3group R0i=M.getRotation();
            Vector3 P0i=M.getPosition();
            Vector3 jAxis= jointAxisses.get(i);
            Vector3 A0i=new Vector3(R0i.matrix.multiply(jAxis));
            String jointT= jointTypes.get(i);
            jointType jt=getJointType(jointT);
            S = computeScrewInBase(A0i, P0i, jt);
			log += "M0,i(0," + i + ") =" + M.matrix + nl;

            M0i.add(new se3group(M.matrix));
            Slist.add(S);
        }
		//R = Rx(π/2) * Ry(0) * Rz(0) = Rx(π/2)
		/*se3group oo=originTotransform( new Origin(1,2,3,(float)(Math.PI/2),0,0));
         log+="xvxvx"+oo.matrix.toString()+nl;*/
		try
        {

            M = M0i.get(M0i.size() - 1);
            log += "M=" + M.matrix.toString() + nl;
            for (int i = 0; i < Miminusonei.size(); i++)
            {
                log += "slist(" + i + ")=" + nl + Slist.get(i).matrix.toString() + nl;
                Svec = se3algebra.se3ToVec(Slist.get(i));
                log += "Svec(" + i + ")=" + nl + Svec.toString() + nl;
                Bvec = new Vector6(se3group.adjoint(se3group.transInverse(M)).multiply(Svec));
                Blist.add(se3algebra.vecToSe3(Bvec));
                log += "Blist(" + i + ")=" + nl + Blist.get(i).matrix.toString() + nl;
                log += "Bvec(" + i + ")=" + nl + Bvec.toString() + nl;
            }
			//int j=4/0;
            calculateJointPositions(new Vector(M0i.size()));
            for (Joint j:joints)
            {
                log += nl + "joint location=" + j.name + nl + j.location.toString() + nl;
            }
		}
        catch (Exception e)
        {
			StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            String stackTrace = sw.toString();

            log += "Exception: " + e.getMessage() + " from IKBody\n";
            log += "Stack trace:\n" + stackTrace + "\n";
            throw new RuntimeException("error in build m0i array" + nl + log, e);
		}
    }

    public void calculateJointPositions(Vector thetalist)
    {
        try
        {
            T0i.clear();
            ArrayList<se3algebra> subSlist= new ArrayList<se3algebra>();
            for (int i=0;i < joints.size();i++)
            {
                if (i < Slist.size())
                {
                    subSlist.add(Slist.get(i));
                }
                if (i < M0i.size())
                {
                    // Create theta vector up to current joint
                    Vector thetaSub = new Vector(i + 1);
                    for (int j = 0; j <= i && j < thetalist.size(); j++)
                    {
                        thetaSub.set(j, thetalist.get(j));
                    }
                    se3group pose=forwardKinematics.FKinSpace(M0i.get(i), subSlist,
                                                              thetaSub, false);
                    T0i.add(pose);
                    log+="T0i("+i+")="+nl+T0i.get(i).matrix.toString()+nl;
                    joints.get(i).location = pose.getPosition();
                }

            }

        }
        catch (Exception e)
        {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            String stackTrace = sw.toString();

            log += "Exception: " + e.getMessage() + " from IKBody\n";
            log += "Stack trace:\n" + stackTrace + "\n";
            throw new RuntimeException("error in calculate joints position" +nl+log+nl);
        }
    }



    // utilities
    private se3algebra computeScrewInBase(Vector3 omega_0, Vector3 p_0, Enum jType)
    {
        if (jType == jointType.REVOLUTE)
        {
            Vector3 v = p_0.cross(omega_0);
            return new se3algebra(omega_0, v);
        }
        else if (jType == jointType.PRISMATIC)
        {
            return new se3algebra(new Vector3(), omega_0);
        }
        throw new IllegalArgumentException("bad arrrrgs...");
    }

// Fixed identity transform
    public static se3group identity()
    {
        return new se3group(Matrix.identity(4));
    }

    public Vector3 axisToVector3(Axis axis)
    {
        return new Vector3(axis.x, axis.y, axis.z);
    }
    public se3group originTotransform(Origin orig)
    {
		so3group so3g= new so3group(Matrix.identity(3));
		try
        {

            so3g = new so3group(so3group.roll(orig.roll).matrix.
                                multiply(so3group.pitch(orig.pitch).matrix.
                                         multiply(so3group.yaw(orig.yaw).matrix)));
            Vector3 pos= new Vector3(orig.x, orig.y, orig.z);
            return se3group.RpToTrans(so3g, pos);
		}
        catch (Exception e)
        {
			throw new RuntimeException("xxx" + so3g.matrix.toString(), e);
		}
    }

	private jointType getJointType(String type)
    {
        if (type == null)
        {
            return jointType.REVOLUTE;  // Default to revolute
        }

        String lowerType = type.toLowerCase();
        if (lowerType.equals("revolute"))
        {
            return jointType.REVOLUTE;
        }
        else if (lowerType.equals("prismatic"))
        {
            return jointType.PRISMATIC;
        }
        else
        {
            return jointType.REVOLUTE;  // Default for unknown types
        }
    }
}
