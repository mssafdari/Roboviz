package com.roboviz;

import java.util.ArrayList;

public class LinkNode {

    public Link link;

    public Joint parentJoint;

    public LinkNode parent;

    public ArrayList<LinkNode> children =
	new ArrayList<>();

}
