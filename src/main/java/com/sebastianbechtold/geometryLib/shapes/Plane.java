package main.java.com.sebastianbechtold.geometryLib.shapes;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Plane implements IShape {

	Vector3D n = null;
	Vector3D d = null;

	public Plane(Vector3D d, Vector3D n) {
		this.d = d;
		this.n = n;
	}

	@Override
	public AABB getAABB() {
		return null;
	}

	@Override
	public Vector3D getCentroid() {
		return null;
	}

	@Override
	public double getIncidenceAngle_rad(Vector3D rayOrigin, Vector3D rayDir) {
		return 0;
	}

	@Override
	public double[] getRayIntersection(Vector3D rayOrigin, Vector3D rayDir) {
		return null;
	}

	

}
