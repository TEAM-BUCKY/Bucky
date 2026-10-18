#ifndef BUCKY_GEOMETRY_H
#define BUCKY_GEOMETRY_H

#include <cmath>

#include <optimizations/optimizations.h>

namespace Geometry
{
	struct Point
	{
		float x;
		float y;
	};

	FORCE_INLINE Point fromBearing(const float bearing, const float distance)
	{
		return {distance * sinf(bearing), distance * cosf(bearing)};
	}

	FORCE_INLINE Point operator-(const Point a, const Point b)
	{
		return {a.x - b.x, a.y - b.y};
	}

	FORCE_INLINE Point operator*(const float s, const Point p)
	{
		return {s * p.x, s * p.y};
	}

	FORCE_INLINE float dot(const Point a, const Point b)
	{
		return a.x * b.x + a.y * b.y;
	}

	FORCE_INLINE float length(const Point p)
	{
		return sqrtf(p.x * p.x + p.y * p.y);
	}
} // namespace Geometry

#endif //BUCKY_GEOMETRY_H
