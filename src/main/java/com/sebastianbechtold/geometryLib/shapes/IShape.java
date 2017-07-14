package main.java.com.sebastianbechtold.geometryLib.shapes;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public interface IShape {

	public Vector3D getCentroid();

	public AABB getAABB();

	double getIncidenceAngle_rad(Vector3D rayOrigin, Vector3D rayDir);

	public double[] getRayIntersection(Vector3D rayOrigin, Vector3D rayDir);
}
