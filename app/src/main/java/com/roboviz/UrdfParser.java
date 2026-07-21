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

		String[] parts = (xyz + " " + rpy).split(" ");

		return new Origin(
            Float.parseFloat(parts[0]),
            Float.parseFloat(parts[1]),
            Float.parseFloat(parts[2]),
            Float.parseFloat(parts[3]),
            Float.parseFloat(parts[4]),
            Float.parseFloat(parts[5]));
	}
	private Axis parseAxis(XmlPullParser parser) {

		String[] parts =
            parser.getAttributeValue(null, "xyz").split(" ");

		return new Axis(
            Float.parseFloat(parts[0]),
            Float.parseFloat(parts[1]),
            Float.parseFloat(parts[2]));
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
	
