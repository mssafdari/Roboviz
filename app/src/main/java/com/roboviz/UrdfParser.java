package com.roboviz;
import java.io.*;
import org.xmlpull.v1.*;

public class UrdfParser
{

    public Robot parse(InputStream input)
	throws IOException, XmlPullParserException
	{

        Robot robot = new Robot();

        XmlPullParserFactory factory = XmlPullParserFactory.newInstance();
        XmlPullParser parser = factory.newPullParser();
        parser.setInput(input, null);

        Joint currentJoint = null;

        int event = parser.getEventType();

        while (event != XmlPullParser.END_DOCUMENT)
		{

            switch (event)
			{

                case XmlPullParser.START_TAG:
                    currentJoint = handleStartTag(parser, robot, currentJoint);
                    break;

                case XmlPullParser.END_TAG:
                    if ("joint".equals(parser.getName()) && currentJoint != null)
					{
                        robot.joints.add(currentJoint);
                    }
                    break;
            }

            event = parser.next();
        }

        buildTree(robot);

        return robot;
    }
	private Joint handleStartTag(
        XmlPullParser parser,
        Robot robot,
        Joint currentJoint) {

		String tag = parser.getName();

		switch (tag) {

			case "robot":
				robot.name = parser.getAttributeValue(null, "name");
				break;

			case "link":
				parseLink(parser, robot);
				break;

			case "joint":
				if (parser.getDepth() == 2)
					currentJoint = parseJoint(parser);
				break;

			case "parent":
				currentJoint.parent = parser.getAttributeValue(null, "link");
				break;

			case "child":
				currentJoint.child = parser.getAttributeValue(null, "link");
				break;

			case "origin":
				currentJoint.origin = parseOrigin(parser);
				break;

			case "axis":
				currentJoint.axis = parseAxis(parser);
				break;
		}

		return currentJoint;
	}
	private void parseLink(XmlPullParser parser, Robot robot) {

		Link link = new Link();
		link.name = parser.getAttributeValue(null, "name");

		robot.links.add(link);
	}
	private Joint parseJoint(XmlPullParser parser) {

		Joint joint = new Joint();
		joint.name = parser.getAttributeValue(null, "name");

		return joint;
	}
	private Origin parseOrigin(XmlPullParser parser) {
		String xyz = parser.getAttributeValue(null, "xyz");
		String rpy = parser.getAttributeValue(null, "rpy");

		// Default values if missing
		float x = 0, y = 0, z = 0;
		float roll = 0, pitch = 0, yaw = 0;

		// Parse xyz
		if (xyz != null && !xyz.isEmpty()) {
			String[] xyzParts = xyz.trim().split("\\s+");
			if (xyzParts.length >= 3) {
				x = Float.parseFloat(xyzParts[0]);
				y = Float.parseFloat(xyzParts[1]);
				z = Float.parseFloat(xyzParts[2]);
			}
		}

		// Parse rpy
		if (rpy != null && !rpy.isEmpty()) {
			String[] rpyParts = rpy.trim().split("\\s+");
			if (rpyParts.length >= 3) {
				roll = Float.parseFloat(rpyParts[0]);
				pitch = Float.parseFloat(rpyParts[1]);
				yaw = Float.parseFloat(rpyParts[2]);
			}
		}

		return new Origin(x, y, z, roll, pitch, yaw);
	}

	private Axis parseAxis(XmlPullParser parser) {
		String xyz = parser.getAttributeValue(null, "xyz");

		float x = 0, y = 0, z = 0;

		if (xyz != null && !xyz.isEmpty()) {
			String[] parts = xyz.trim().split("\\s+");
			if (parts.length >= 3) {
				x = Float.parseFloat(parts[0]);
				y = Float.parseFloat(parts[1]);
				z = Float.parseFloat(parts[2]);
			}
		}

		return new Axis(x, y, z);
	}
	private void buildTree(Robot robot) {

		for (Link link : robot.links) {

			LinkNode node = new LinkNode();
			node.link = link;

			robot.nodes.put(link.name, node);
		}

		for (Joint joint : robot.joints) {

			LinkNode child = robot.nodes.get(joint.child);
			LinkNode parent = robot.nodes.get(joint.parent);

			child.parentJoint = joint;
			child.parent = parent;

			parent.children.add(child);
		}

		for (LinkNode node : robot.nodes.values()) {

			if (node.parent == null) {

				robot.root = node;
				break;
			}
		}
	}
}
	
