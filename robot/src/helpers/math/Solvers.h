#ifndef BUCKY_SOLVERS_H
#define BUCKY_SOLVERS_H

#include <cmath>

#include <optimizations/logic.h>
#include <optimizations/optimizations.h>

namespace Solvers
{
	template <typename F, typename DF>
	FORCE_INLINE float newton(float x, const float lo, const float hi, const int iterations, F f, DF slope)
	{
		for (int i = 0; i < iterations; i++)
		{
			x -= f(x) / slope(x);
			x = clamp(x, lo, hi);
		}
		return x;
	}

	template <typename F>
	FORCE_INLINE float forwardDifference(F f, const float x, const float step, const float f0)
	{
		return (f(x + step) - f0) / step;
	}

	struct NormalEquations2
	{
		float haa = 0.0f, hab = 0.0f, hbb = 0.0f;
		float ga = 0.0f, gb = 0.0f;

		FORCE_INLINE void add(const float da, const float db, const float residual, const float weight)
		{
			haa += weight * da * da;
			hab += weight * da * db;
			hbb += weight * db * db;
			ga += weight * da * residual;
			gb += weight * db * residual;
		}

		FORCE_INLINE bool solve(float& stepA, float& stepB) const
		{
			const float det = haa * hbb - hab * hab;
			if (fabsf(det) < 1e-12f) return false;
			stepA = (hbb * ga - hab * gb) / det;
			stepB = (haa * gb - hab * ga) / det;
			return true;
		}
	};

	struct Observation
	{
		float value;
		float weight;
	};

	template <typename Model, typename StepB, typename Bound>
	FORCE_INLINE void gaussNewton2(float& a, float& b, const int iterations, const Observation* observations,
	                               const int count, Model model, const float stepA, StepB stepB, Bound bound)
	{
		for (int iter = 0; iter < iterations; iter++)
		{
			NormalEquations2 normal;
			const float hB = stepB(b);
			for (int i = 0; i < count; i++)
			{
				const float predicted = model(i, a, b);
				const float da = forwardDifference([&](const float x) { return model(i, x, b); }, a, stepA, predicted);
				const float db = forwardDifference([&](const float x) { return model(i, a, x); }, b, hB, predicted);
				normal.add(da, db, observations[i].value - predicted, observations[i].weight);
			}
			float deltaA, deltaB;
			if (!normal.solve(deltaA, deltaB)) break;
			a += deltaA;
			b += deltaB;
			bound(a, b);
		}
	}
} // namespace Solvers

#endif //BUCKY_SOLVERS_H
