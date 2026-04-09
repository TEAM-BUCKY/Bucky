#ifndef BUCKY_MATH_H
#define BUCKY_MATH_H

#include <cstddef>

#include "Constants.h"
#include "optimizations/optimizations.h"

constexpr float PI_F = _PI_F;
constexpr float INV_PI_F = _INV_PI_F;

namespace Math
{
	template <std::size_t N>
	struct Vector
	{
		float v[N] = {};

		FORCE_INLINE float& operator[](const std::size_t i) { return v[i]; }
		FORCE_INLINE const float& operator[](const std::size_t i) const { return v[i]; }
		FORCE_INLINE float* data() { return v; }
		FORCE_INLINE const float* data() const { return v; }
	};

	template <std::size_t Rows, std::size_t Cols>
	struct Matrix
	{
		float m[Rows][Cols] = {};

		FORCE_INLINE float* operator[](const std::size_t i) { return m[i]; }
		FORCE_INLINE const float* operator[](const std::size_t i) const { return m[i]; }
		FORCE_INLINE float (*data())[Cols] { return m; }
		FORCE_INLINE const float (*data() const)[Cols] { return m; }
	};

	using Vec2 = Vector<2>;
	using Vec4 = Vector<4>;
	using Mat22 = Matrix<2, 2>;
	using Mat24 = Matrix<2, 4>;
	using Mat42 = Matrix<4, 2>;
	using Mat44 = Matrix<4, 4>;

	FORCE_INLINE Vec2 vec2(const float x, const float y)
	{
		return {{x, y}};
	}

	FORCE_INLINE Vec4 vec4(const float x, const float y, const float z, const float w)
	{
		return {{x, y, z, w}};
	}

	FORCE_INLINE Mat22 mat2(const float a00, const float a01, const float a10, const float a11)
	{
		return {{{a00, a01}, {a10, a11}}};
	}

	FORCE_INLINE Mat24 mat2x4(const float a00, const float a01, const float a02, const float a03,
	                          const float a10, const float a11, const float a12, const float a13)
	{
		return {{{a00, a01, a02, a03}, {a10, a11, a12, a13}}};
	}

	FORCE_INLINE Mat44 mat4(const float a00, const float a01, const float a02, const float a03,
	                        const float a10, const float a11, const float a12, const float a13,
	                        const float a20, const float a21, const float a22, const float a23,
	                        const float a30, const float a31, const float a32, const float a33)
	{
		return {{{a00, a01, a02, a03}, {a10, a11, a12, a13}, {a20, a21, a22, a23}, {a30, a31, a32, a33}}};
	}

	FORCE_INLINE Mat22 diag2(const float d0, const float d1)
	{
		return mat2(d0, 0.0f, 0.0f, d1);
	}

	FORCE_INLINE Mat44 diag4(const float d0, const float d1, const float d2, const float d3)
	{
		Mat44 out = {};
		out[0][0] = d0;
		out[1][1] = d1;
		out[2][2] = d2;
		out[3][3] = d3;
		return out;
	}

	FORCE_INLINE Mat44 identity4()
	{
		return diag4(1.0f, 1.0f, 1.0f, 1.0f);
	}

	inline void zeroMatrix4(float m[4][4])
	{
		for (std::size_t i = 0; i < 4; ++i)
		{
			for (std::size_t j = 0; j < 4; ++j)
			{
				m[i][j] = 0.0f;
			}
		}
	}

	inline void identityMatrix4(float m[4][4])
	{
		zeroMatrix4(m);
		for (std::size_t i = 0; i < 4; ++i)
		{
			m[i][i] = 1.0f;
		}
	}

	inline void zeroMatrix2(float m[2][2])
	{
		for (std::size_t i = 0; i < 2; ++i)
		{
			for (std::size_t j = 0; j < 2; ++j)
			{
				m[i][j] = 0.0f;
			}
		}
	}

	inline void identityMatrix2(float m[2][2])
	{
		zeroMatrix2(m);
		m[0][0] = 1.0f;
		m[1][1] = 1.0f;
	}

	FORCE_INLINE void zeroMatrix4(Mat44& m)
	{
		m = {};
	}

	FORCE_INLINE void identityMatrix4(Mat44& m)
	{
		m = identity4();
	}

	FORCE_INLINE void zeroMatrix2(Mat22& m)
	{
		m = {};
	}

	FORCE_INLINE void identityMatrix2(Mat22& m)
	{
		m = diag2(1.0f, 1.0f);
	}

	FORCE_INLINE void setVector2(float out[2], const float x, const float y)
	{
		out[0] = x;
		out[1] = y;
	}

	FORCE_INLINE void setVector2(Vec2& out, const float x, const float y)
	{
		out = vec2(x, y);
	}

	FORCE_INLINE void setVector4(float out[4], const float x, const float y, const float z, const float w)
	{
		out[0] = x;
		out[1] = y;
		out[2] = z;
		out[3] = w;
	}

	FORCE_INLINE void setVector4(Vec4& out, const float x, const float y, const float z, const float w)
	{
		out = vec4(x, y, z, w);
	}

	FORCE_INLINE void setDiagonal2(float m[2][2], const float d0, const float d1)
	{
		zeroMatrix2(m);
		m[0][0] = d0;
		m[1][1] = d1;
	}

	FORCE_INLINE void setDiagonal2(Mat22& m, const float d0, const float d1)
	{
		m = diag2(d0, d1);
	}

	FORCE_INLINE void setDiagonal4(float m[4][4], const float d0, const float d1, const float d2, const float d3)
	{
		zeroMatrix4(m);
		m[0][0] = d0;
		m[1][1] = d1;
		m[2][2] = d2;
		m[3][3] = d3;
	}

	FORCE_INLINE void setDiagonal4(Mat44& m, const float d0, const float d1, const float d2, const float d3)
	{
		m = diag4(d0, d1, d2, d3);
	}

	FORCE_INLINE void copyMatrix4(float dst[4][4], const float src[4][4])
	{
		for (std::size_t i = 0; i < 4; ++i)
		{
			for (std::size_t j = 0; j < 4; ++j)
			{
				dst[i][j] = src[i][j];
			}
		}
	}

	FORCE_INLINE void copyMatrix4(Mat44& dst, const Mat44& src)
	{
		dst = src;
	}

	FORCE_INLINE void addVector4(Vec4& lhs, const Vec4& rhs)
	{
		lhs[0] += rhs[0];
		lhs[1] += rhs[1];
		lhs[2] += rhs[2];
		lhs[3] += rhs[3];
	}

	FORCE_INLINE void addMatrix2(Mat22& lhs, const Mat22& rhs)
	{
		lhs[0][0] += rhs[0][0];
		lhs[0][1] += rhs[0][1];
		lhs[1][0] += rhs[1][0];
		lhs[1][1] += rhs[1][1];
	}

	FORCE_INLINE void addMatrix4(Mat44& lhs, const Mat44& rhs)
	{
		for (std::size_t i = 0; i < 4; ++i)
		{
			for (std::size_t j = 0; j < 4; ++j)
			{
				lhs[i][j] += rhs[i][j];
			}
		}
	}

	FORCE_INLINE void subtractFromIdentity4(float out[4][4], const float rhs[4][4])
	{
		identityMatrix4(out);
		for (std::size_t i = 0; i < 4; ++i)
		{
			for (std::size_t j = 0; j < 4; ++j)
			{
				out[i][j] -= rhs[i][j];
			}
		}
	}

	FORCE_INLINE void subtractFromIdentity4(Mat44& out, const Mat44& rhs)
	{
		out = identity4();
		for (std::size_t i = 0; i < 4; ++i)
		{
			for (std::size_t j = 0; j < 4; ++j)
			{
				out[i][j] -= rhs[i][j];
			}
		}
	}

	template <std::size_t Rows, std::size_t Inner, std::size_t Cols>
	FORCE_INLINE void multiplyMatrix(const float (*lhs)[Inner], const float (*rhs)[Cols], float (&out)[Rows][Cols])
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
	FORCE_INLINE void multiplyMatrixByTransposed(const float (*lhs)[Inner], const float (*rhs)[Inner], float (&out)[Rows][Cols])
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
	FORCE_INLINE void multiplyMatrixVector(const float (*lhs)[Cols], const float *rhs, float (&out)[Rows])
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
		while (radians > PI_F) radians -= 2.0f * PI_F;
		while (radians < -PI_F) radians += 2.0f * PI_F;
		return radians;
	}

	FORCE_INLINE float wrapDegrees(float degrees)
	{
		while (degrees > 360.0f) degrees -= 360.0f;
		while (degrees < 0.0f) degrees += 360.0f;
		return degrees;
	}
} // namespace Math

#endif //BUCKY_MATH_H