#ifndef BUCKY_MATH_H
#define BUCKY_MATH_H

#include <cstddef>
#include <cstdint>

#include <helpers/constants.h>
#include <optimizations/logic.h>
#include <optimizations/optimizations.h>


namespace Math
{
	template <std::size_t N>
	struct Vector
	{
		float v[N] = {};

		FORCE_INLINE float& operator[](const std::size_t i) { return v[i]; }
		FORCE_INLINE const float& operator[](const std::size_t i) const { return v[i]; }
	};

	template <std::size_t Rows, std::size_t Cols>
	struct Matrix
	{
		float m[Rows][Cols] = {};

		FORCE_INLINE float* operator[](const std::size_t i) { return m[i]; }
		FORCE_INLINE const float* operator[](const std::size_t i) const { return m[i]; }
	};

	template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
	FORCE_INLINE void multiplyMatrix(const Matrix<Rows, Inner>& lhs, const Matrix<Inner, Cols>& rhs, Matrix<Rows, Cols>& out)
	{
		for (std::size_t i = 0; i < Rows; ++i)
		{
			for (std::size_t j = 0; j < Cols; ++j)
			{
				float acc = 0.0f;
				for (std::size_t k = 0; k < Inner; ++k)
				{
					acc += lhs[i][k] * rhs[k][j];
				}
				out[i][j] = acc;
			}
		}
	}

	template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
	FORCE_INLINE void multiplyMatrixByTransposed(const Matrix<Rows, Inner>& lhs, const Matrix<Cols, Inner>& rhs,
	                                             Matrix<Rows, Cols>& out)
	{
		for (std::size_t i = 0; i < Rows; ++i)
		{
			for (std::size_t j = 0; j < Cols; ++j)
			{
				float acc = 0.0f;
				for (std::size_t k = 0; k < Inner; ++k)
				{
					acc += lhs[i][k] * rhs[j][k];
				}
				out[i][j] = acc;
			}
		}
	}

	template <std::size_t Rows, std::size_t Cols>
	FORCE_INLINE void multiplyMatrixVector(const Matrix<Rows, Cols>& lhs, const Vector<Cols>& rhs, Vector<Rows>& out)
	{
		for (std::size_t i = 0; i < Rows; ++i)
		{
			float acc = 0.0f;
			for (std::size_t j = 0; j < Cols; ++j)
			{
				acc += lhs[i][j] * rhs[j];
			}
			out[i] = acc;
		}
	}

	FORCE_INLINE float radiansToDegrees(const float radians)
	{
		return radians * (180.0f / PI_F);
	}

	FORCE_INLINE float degreesToRadians(const float degrees)
	{
		return degrees * (PI_F / 180.0f);
	}

	FORCE_INLINE float wrapRadians(float radians)
	{
		constexpr float kTwoPi = 2.0f * PI_F;
		if (radians >  PI_F) radians -= kTwoPi;
		else if (radians < -PI_F) radians += kTwoPi;
		if (radians >  PI_F) radians -= kTwoPi;
		else if (radians < -PI_F) radians += kTwoPi;
		return radians;
	}

	FORCE_INLINE float wrapSignedDegrees(float degrees)
	{
		if (degrees > 180.0f) degrees -= 360.0f;
		else if (degrees < -180.0f) degrees += 360.0f;
		return degrees;
	}

	FORCE_INLINE float wrapDegrees(float degrees)
	{
		if (degrees >= 360.0f) degrees -= 360.0f;
		else if (degrees < 0.0f) degrees += 360.0f;
		if (degrees >= 360.0f) degrees -= 360.0f;
		else if (degrees < 0.0f) degrees += 360.0f;
		return degrees;
	}

	FORCE_INLINE uint8_t ringPrev(const uint8_t i, const uint8_t n)
	{
		return (i + n - 1) % n;
	}

	FORCE_INLINE uint8_t ringNext(const uint8_t i, const uint8_t n)
	{
		return (i + 1) % n;
	}
} // namespace Math

#endif //BUCKY_MATH_H