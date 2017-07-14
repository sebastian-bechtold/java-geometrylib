package main.java.com.sebastianbechtold.geometryLib.shapes;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Sphere implements IShape {

	double radius = 0;
	Vector3D center;

	public Sphere(Vector3D center, double radius) {
		this.center = center;
		this.radius = radius;
	}

	@Override
	public double[] getRayIntersection(Vector3D rayOrigin, Vector3D rayDir) {

		// TODO 4: Source?

		Vector3D origin = rayOrigin.subtract(this.center);

		double[] result = new double[2];

		// Compute A, B and C coefficients:
		double a = rayDir.dotProduct(rayDir);
		double b = 2 * rayDir.dotProduct(origin);
		double c = origin.dotProduct(origin) - this.radius * this.radius;

		// Find discriminant:
		double disc = b * b - 4 * a * c;

		// if discriminant is negative there are no real roots, so return
		// false as ray misses sphere:
		if (disc < 0) {
			result[0] = -1;
			return result;
		}

		// compute q:
		double distSqrt = Math.sqrt(disc);

		double q = 0;

		if (b < 0) {
			q = (-b - distSqrt) / 2;
		} else {
			q = (-b + distSqrt) / 2;
		}

		double t0 = q / a;
		double t1 = c / q;

		if (t0 > t1) {
			double temp = t0;
			t0 = t1;
			t1 = temp;
		}

		// if t1 is less than zero, the object is in the ray's negative direction
		// and consequently the ray misses the sphere
		if (t1 < 0) {
			result[0] = -1;
			return result;
		}

		// if t0 is less than zero, the intersection point is at t1
		if (t0 < 0) {
			result[0] = t1;
			return result;
		}

		// else the intersection point is at t0
		else {
			result[0] = t0;
			return result;
		}
	}

	@Override
	public AABB getAABB() {

		Vector3D halfSize = new Vector3D(this.radius, this.radius, this.radius);
		AABB result = new AABB(this.center.subtract(halfSize), this.center.add(halfSize));
		return result;
	}


	@Override
	public Vector3D getCentroid() {
		return this.center;
	}

	@Override
	public double getIncidenceAngle_rad(Vector3D rayOrigin, Vector3D rayDir) {
		// TODO 2: Implement this
		return 0;
	}
}
