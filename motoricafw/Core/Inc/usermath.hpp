/*
 * usermath.hpp
 *
 *  Created on: 22 авг. 2018 г.
 *      Author: tkhasanshin
 */

#pragma once

#include "math.h"
#include "stdarg.h"

namespace math
{
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795f
#endif
#define M_SMALLNUMBER 1e-9
#define FILTER_MAX_NUM 3
#define array_size(_arr_) (sizeof(_arr_) / sizeof(_arr_[0]))

	//Класс точки для математических операций и построения графиков
	template <typename T>
	class Point
	{
	private:
		T xval, yval;

	public:
		Point(T x = 0.0, T y = 0.0)
			: xval(x)
			, yval(y)
		{
		}

		Point(Point& p)
			: xval(p.xval)
			, yval(p.yval)
		{
		}

		T x()
		{
			return xval;
		}
		T y()
		{
			return yval;
		}

		void setX(T x)
		{
			xval = x;
		}

		void setY(T y)
		{
			yval = y;
		}

		T dist(Point& other)
		{
			return hypotf(xval - other.xval, yval - other.yval);
		}

		Point operator+(Point& p)
		{
			return Point(xval + p.xval, yval + p.yval);
		}

		Point operator-(Point& p)
		{
			return Point(xval - p.xval, yval - p.yval);
		}

		void move(T x, T y)
		{
			xval += x;
			yval += y;
		}

		float angle()
		{


			return 0;
		}
	};

	template<typename T, uint8_t rows, uint8_t cols>
	class Matrix
	{
	public:
		Matrix()
		{

		}

		void setValue(const T& value, uint8_t row, uint8_t column)
		{
			data[row][column] = value;
		}

		Matrix inverse()
		{

		}

		Matrix transpose()
		{

		}

		Matrix getIdentity()
		{

		}

		Matrix& toIdentity()
		{

		}

		uint8_t getRows()
		{
			return ySize;
		}

		uint8_t getCols()
		{
			return xSize;
		}

		void zero()
		{

		}

		Matrix operator*(const Matrix& m)
		{

		}

		Matrix operator+(const Matrix& m)
		{

		}

		Matrix operator-(const Matrix& m)
		{

		}

		Matrix& operator*=(const Matrix& m)
		{

		}

		Matrix& operator+=(const Matrix& m)
		{

		}

		Matrix& operator-=(const Matrix& m)
		{

		}

		Matrix& operator=(const Matrix& m)
		{

		}

		Matrix operator==(const Matrix& m)
		{

		}

		Matrix operator!=(const Matrix& m)
		{

		}

		Matrix operator*(const T& m)
		{

		}

		Matrix operator+(const T& m)
		{

		}

		Matrix operator-(const T& m)
		{

		}

		Matrix& operator*=(const T& m)
		{

		}

		Matrix& operator+=(const T& m)
		{

		}

		Matrix& operator-=(const T& m)
		{

		}

		Matrix overlap(const Matrix& m, uint8_t row, uint8_t column)
		{

		}

	private:
		T data[rows][cols];
		const uint8_t xSize = cols;
		const uint8_t ySize = rows;
	};

	//Возвращает состояние бита под номером num в word
	template<typename T>
	inline constexpr bool bit(T& word, uint8_t num)
	{
		return (word & (1 << num)) != 0;
	}

	//Устанавливает бит num в состояние state в слове word
	template<typename T>
	inline constexpr void bit(T& word, uint8_t num, bool state)
	{
		word = ((~(word & (1 << num))) & word) | ((T)state << num);
	}

	//Устанавливает num бит в биты из state в слове word
	template<typename T>
	inline constexpr void bits(T& word, uint8_t num, T state, uint8_t pos = 0)
	{
		word = ((~(word & (((1 << num) - 1) << pos))) & word) | (state << pos);
	}

	//Возвращает num бит из word начиная с pos
	template<typename T>
	inline constexpr T bits(T& word, uint8_t num, uint8_t pos = 0)
	{
		return (word & (((1 << num) - 1) << pos)) >> pos;
	}

	template<unsigned char bitNum>
	inline constexpr bool bitOR(unsigned int data)
	{
		return bit(data, bitNum);
	}

	template<unsigned char bitNum>
	inline constexpr bool bitAND(unsigned int data)
	{
		return bit(data, bitNum);
	}

	template<unsigned char bitNum>
	inline constexpr bool bitXOR(unsigned int data)
	{
		return bit(data, bitNum);
	}

	template<unsigned char bit1, unsigned char bit2, unsigned char... bit>
	inline constexpr bool bitOR(unsigned int data)
	{
		return bitOR<bit1>(data) | bitOR<bit2, bit...>(data);
	}

	template<unsigned char bit1, unsigned char bit2, unsigned char... bit>
	inline constexpr bool bitAND(unsigned int data)
	{
		return bitAND<bit1>(data) & bitAND<bit2, bit...>(data);
	}

	template<unsigned char bit1, unsigned char bit2, unsigned char... bit>
	inline constexpr bool bitXOR(unsigned int data)
	{
		return bitXOR<bit1>(data) ^ bitXOR<bit2, bit...>(data);
	}

	template<typename T>
	inline void swap(T& first, T& second)
	{
		T buf = first;
		first = second;
		second = buf;
	}

	template<typename T>
	constexpr T bitsAll1(unsigned char number)
	{
		return (1 << number) - 1;
	}

	template<typename T>
	inline T min(T& first, T& second)
	{
		return (first > second) ? second : first;
	}

	template<typename T>
	inline T max(T first, T second)
	{
		return (first < second) ? second : first;
	}

	//Ограничивает значение порогами
	template <typename T>
	void bound(T& value, T min, T max)
	{
		if (value < min)
			value = min;
		if (value > max)
			value = max;
	}

	//Возвращает значение ограниченное порогами
	template <typename T>
	T bound2(T value, T min, T max)
	{
		if (value < min)
			value = min;
		if (value > max)
			value = max;
		return value;
	}

	//Возвращает значение ограниченное по модулю
	template <typename T>
	T clampByAbs(T value, T maxAbs)
	{
		if (value < -maxAbs)
			value = -maxAbs;
		else if (value > maxAbs)
			value = maxAbs;
		return value;
	}

	//Проверяет значение на принадлежность отрезку с заданной точностью
	template <typename T>
	bool validateRange(T value, T min, T max, T tolerance = 0)
	{
		if (min > max)
		{
			T buf = min;
			min = max;
			max = buf;
		}
		return (value > (min - tolerance)) && (value < (max + tolerance));
	}

	//Возвращает линейно отображенное значение одного диапазона в другой
	template <typename T>
	T scale(T value, T src_min, T src_max, T dst_min, T dst_max)
	{
		return ((value - src_min) * (dst_max - dst_min)) / (src_max - src_min) + dst_min;
	}

	//Вычисляет среднее по массиву
	template <typename T>
	T average(T* data, unsigned int size)
	{
		T result = 0;
		for (unsigned int i = 0; i < size; i++)
			result += data[i];
		return result / size;
	}

	//Если число меньше M_SMALLNUMBER, то приравнивает его к M_SMALLNUMBER, сохраняя знак
	template <typename T>
	void boundSmall(T& value)
	{
		if ((value > 0) && (value < M_SMALLNUMBER))
			value += M_SMALLNUMBER;
		else if ((value < 0) && (value > -M_SMALLNUMBER))
			value -= M_SMALLNUMBER;
	}

	//Если число меньше M_SMALLNUMBER, то приравнивает его к M_SMALLNUMBER, сохраняя знак
	template <typename T>
	T boundSmall2(T value)
	{
		if ((value > 0) && (value < M_SMALLNUMBER))
			value += M_SMALLNUMBER;
		else if ((value < 0) && (value > -M_SMALLNUMBER))
			value -= M_SMALLNUMBER;
		return value;
	}

	template <typename T>
	T abs(T& x)
	{
		if (x < 0)
			return -x;
		else
			return x;
	}

	template <typename T>
	T sgn(T& x)
	{
		if (x < 0)
			return T(-1);
		else
			return T(1);
	}

	// Variable "clamp" must receive only positive values!!!
	template <typename T>
	constexpr T clampDecAbs(T& x, T clamp)
	{
		if ((x > 0) && (x > clamp))
		{
			x -= clamp;
			return clamp;
		}
		else if ((x < 0) && (x < -clamp))
		{
			x += clamp;
			return -clamp;
		}
		T lastValue = x;
		x = 0;
		return lastValue;
	}

	template <typename T>
	bool inRange(T& value, T min, T max)
	{
		return ((value >= min) && (value <= max));
	}

	auto func = [](float x)-> float { return 0; };
	auto derivativeAt = [](__typeof(func)f, float x, float step = 0.0001f)-> float
	{
		return (f(x + step / 2) - f(x - step / 2)) / step;
	};
	auto definiteIntegral = [](__typeof(func)f, float x0, float x1, float step = 0.0001f)-> float
	{
		float sum = 0;
		for (float x = x0; x < x1; x += step)
			sum += f(x);
		return sum * step;
	};

	//Возвращает значение кусочно-линейной функции в точке, заданной массивом точек
	template <typename T>
	T interpLinear(T inputValue, Point<T>* p, uint32_t pointsNum, bool extrapolateBounds = true, uint32_t* arrayCatchIndex = NULL)
	{
		uint32_t lastPoint = pointsNum - 1;
		uint32_t catchIndex = 0;

		if (arrayCatchIndex == NULL)
			arrayCatchIndex = &catchIndex;

		if (inputValue < p[0].x()) //Значение не попало и находится левее точек массива на координатной прямой
		{
			T dx = p[1].x() - p[0].x();
			T dy = p[1].y() - p[0].y();

			if (extrapolateBounds)
			{
				*arrayCatchIndex = pointsNum - 1;
				return p[0].y() + dy / dx * (inputValue - p[0].x());
			}
			else
			{
				*arrayCatchIndex = -1;
				return p[0].y();
			}
		}
		//Значение находится среди точек массива, можно вычислить координату Y пересечения с ломанной линией построенной по точкам массива
		else if ((inputValue >= p[0].x()) && (inputValue <= p[lastPoint].x()))
		{
			T result;
			for (uint32_t i = 1; i < pointsNum; i++)
			{
				if (inputValue <= p[i].x())
				{
					T dx = p[i].x() - p[i - 1].x();
					T dy = p[i].y() - p[i - 1].y();

					if (dx == 0)
						dx = M_SMALLNUMBER;

					*arrayCatchIndex = i - 1;
					result = p[i - 1].y() + dy / dx * (inputValue - p[i - 1].x());
					break;
				}
			}
			return result;
		}
		else //Значение не попало и находится правее точек массива на координатной прямой
		{
			T dx = p[lastPoint].x() - p[lastPoint - 1].x();
			T dy = p[lastPoint].y() - p[lastPoint - 1].y();

			if (extrapolateBounds)
			{
				*arrayCatchIndex = 0;
				return p[lastPoint - 1].y() + dy / dx * (inputValue - p[lastPoint - 1].x());
			}
			else
			{
				*arrayCatchIndex = -1;
				return p[lastPoint].y();
			}
		}
	}

	//Возвращает значение кусочно-линейной функции в точке, заданной массивом точек
	template <typename T>
	T interpLinear(T inputValue, T* x, T* y, uint32_t pointsNum, bool extrapolateBounds = true, uint32_t* arrayCatchIndex = NULL)
	{
		uint32_t lastPoint = pointsNum - 1;
		uint32_t catchIndex = 0;

		if (arrayCatchIndex == NULL)
			arrayCatchIndex = &catchIndex;

		if (inputValue < x[0]) //Значение не попало и находится левее точек массива на координатной прямой
		{
			T dx = x[1] - x[0];
			T dy = y[1] - y[0];

			if (extrapolateBounds)
			{
				*arrayCatchIndex = pointsNum - 1;
				return y[0] + dy / dx * (inputValue - x[0]);
			}
			else
			{
				*arrayCatchIndex = -1;
				return y[0];
			}
		}
		//Значение находится среди точек массива, можно вычислить координату Y пересечения с ломанной линией построенной по точкам массива
		else if ((inputValue >= x[0]) && (inputValue <= x[lastPoint]))
		{
			T result;
			for (uint32_t i = 1; i < pointsNum; i++)
			{
				if (inputValue <= x[i])
				{
					T dx = x[i] - x[i - 1];
					T dy = y[i] - y[i - 1];

					if (dx == 0)
						dx = M_SMALLNUMBER;

					*arrayCatchIndex = i - 1;
					result = y[i - 1] + dy / dx * (inputValue - x[i - 1]);
					break;
				}
			}
			return result;
		}
		else //Значение не попало и находится правее точек массива на координатной прямой
		{
			T dx = x[lastPoint] - x[lastPoint - 1];
			T dy = y[lastPoint] - y[lastPoint - 1];

			if (extrapolateBounds)
			{
				*arrayCatchIndex = 0;
				return y[lastPoint - 1] + dy / boundSmall2(dx) * (inputValue - x[lastPoint - 1]);
			}
			else
			{
				*arrayCatchIndex = -1;
				return y[lastPoint];
			}
		}
	}

	//Возвращает значение кусочно-линейной функции в точке, заданной массивом точек (вариант для константных массивов)
	template <typename T>
	T interpLinear(T inputValue, const T* x, const T* y, uint32_t pointsNum, bool extrapolateBounds = true, uint32_t* arrayCatchIndex = NULL)
	{
		uint32_t lastPoint = pointsNum - 1;
		uint32_t catchIndex = 0;

		if (arrayCatchIndex == NULL)
			arrayCatchIndex = &catchIndex;

		if (inputValue < x[0]) //Значение не попало и находится левее точек массива на координатной прямой
		{
			T dx = x[1] - x[0];
			T dy = y[1] - y[0];

			if (extrapolateBounds)
			{
				*arrayCatchIndex = pointsNum - 1;
				return y[0] + dy / dx * (inputValue - x[0]);
			}
			else
			{
				*arrayCatchIndex = -1;
				return y[0];
			}
		}
		//Значение находится среди точек массива, можно вычислить координату Y пересечения с ломанной линией построенной по точкам массива
		else if ((inputValue >= x[0]) && (inputValue <= x[lastPoint]))
		{
			T result;
			for (uint32_t i = 1; i < pointsNum; i++)
			{
				if (inputValue <= x[i])
				{
					T dx = x[i] - x[i - 1];
					T dy = y[i] - y[i - 1];

					if (dx == 0)
						dx = M_SMALLNUMBER;

					*arrayCatchIndex = i - 1;
					result = y[i - 1] + dy / dx * (inputValue - x[i - 1]);
					break;
				}
			}
			return result;
		}
		else //Значение не попало и находится правее точек массива на координатной прямой
		{
			T dx = x[lastPoint] - x[lastPoint - 1];
			T dy = y[lastPoint] - y[lastPoint - 1];

			if (extrapolateBounds)
			{
				*arrayCatchIndex = 0;
				return y[lastPoint - 1] + dy / dx * (inputValue - x[lastPoint - 1]);
			}
			else
			{
				*arrayCatchIndex = -1;
				return y[lastPoint];
			}
		}
	}

	/*
	 * Функция нахождения значения одной из интерполирующих координат билинейной интерполяции
	 * при заданном значении второй, заданном результате интерполяции в точке известного
	 * значения второй координаты и заданном массиве исходных данных для интерполяции размером xSize * ySize
	 */
	 //template <typename T>
	 //T invBilinearInterp(T x, T f, T *xCoords, T *yCoords, T *dataArray, uint32_t xSize, uint32_t ySize, bool extrapolation = true)
	 //{
	 //	T outputY, dy, dx, x1, x2, y1, y2, f11, f12, f21, f22;
	 //	uint32_t lastX = xSize - 1;   // последний элемент массива значений координат X
	 //	uint32_t lastY = ySize - 1;   // последний элемент массива значений координат Y
	 //	uint32_t ix1, iy1, ix2, iy2;  // итераторы массивов значений координат
	 //	if (x > xCoords[lastX])
	 //	{
	 //		ix1 = lastX - 1;
	 //		ix2 = lastX;
	 //	}
	 //	else if (x < xCoords[0])
	 //	{
	 //		ix1 = 0;
	 //		ix2 = 1;
	 //	}
	 //	else
	 //	{
	 //		for (uint32_t i = 0; i < xSize; i++)
	 //		{
	 //			if (x > xCoords[i])
	 //			{
	 //				ix1 = i;
	 //				ix2 = i + 1;
	 //				break;
	 //			}
	 //		}
	 //	}
	 //	if (f > dataArray[yCoords[ySize] * xSize + ix1].y())
	 //	{
	 //		iy1 = lastY - 1;
	 //		iy2 = lastY;
	 //	}
	 //	else if (f < dataArray[yCoords[0] * xSize + ix1].y())
	 //	{
	 //		iy1 = 0;
	 //		iy2 = 1;
	 //	}
	 //	else
	 //	{
	 //		for (uint32_t i = 0; i < ySize; i++)
	 //		{
	 //			if (f > xCoords[i])
	 //			{
	 //				iy1 = i;
	 //				iy2 = i + 1;
	 //				break;
	 //			}
	 //		}
	 //	}
	 //	outputY = (x * dy * dx + y1 * ((x2 - x) * f12 + (x - x1) * f22) - y2 * ((x2 - x) * f11 + (x - x1) * f21)) / ((x2 - x) * f12 + (x - x1) * f22 - (x2 - x) * f11 - (x - x1) * f21);
	 //}

	//Функция вычисления линейной регрессии данных, возвращает суммарную ошибку
	template <typename T>
	T linearRegression(T* X, T* Y, uint32_t size, T& a, T& b)
	{
		if (size == 0)
			return 0;

		T Sx = 0, Sy = 0, Sxy = 0, Sxx = 0, errorSum = 0;

		//Вычисление коэффициентов линейного уравнения
		for (int i = 0; i < size; i++)
		{
			Sx += X[i];
			Sy += Y[i];
			Sxy += X[i] * Y[i];
			Sxx += X[i] * X[i];
		}

		Sx /= size;
		Sy /= size;
		Sxy /= size;
		Sxx /= size;

		if ((Sx == 0) || ((Sx * Sx - Sxx) == 0))
			return 0;

		a = (Sx * Sy - Sxy) / (Sx * Sx - Sxx);
		b = (Sxy - a * Sxx) / Sx;

		//Вычисление ср. кв. отклонения:
		for (int i = 0; i < size; i++)
			errorSum += (X[i] * a + b - Y[i]) * (X[i] * a + b - Y[i]);

		return errorSum;
	}

	template <typename T, unsigned int pointsNum, unsigned int degree>
	T polynomialRegression(T* x, T* y, T* coeffs)
	{
		int i, j, k;
		T X[2 * degree + 1];		//Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		T B[degree + 1][degree + 2];	//B is the Normal matrix(augmented) that will store the equations
		T Y[degree + 1];	//Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		
		for (i = 0; i < 2 * degree + 1; i++)
		{
			X[i] = 0;
			for (j = 0; j < pointsNum; j++)
				X[i] = X[i] + pow(x[j], i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
		}

		for (i = 0; i <= degree; i++)
			for (j = 0; j <= degree; j++)
				B[i][j] = X[i + j];	//Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix

		for (i = 0; i < degree + 1; i++)
		{
			Y[i] = 0;
			for (j = 0; j < pointsNum; j++)
				Y[i] = Y[i] + pow(x[j], i) * y[j];	//consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
		}

		for (i = 0; i <= degree; i++)
			B[i][degree + 1] = Y[i];	//load the values of Y as the last column of B(Normal Matrix but augmented)
		
		unsigned int deg = degree + 1;	//n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
		
		for (i = 0; i < deg; i++)	//From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
			for (k = i + 1; k < deg; k++)
				if (B[i][i] < B[k][i])
					for (j = 0; j <= deg; j++)
					{
						T temp = B[i][j];
						B[i][j] = B[k][j];
						B[k][j] = temp;
					}

		for (i = 0; i < deg - 1; i++)	//loop to perform the gauss elimination
			for (k = i + 1; k < deg; k++)
			{
				T t = B[k][i] / B[i][i];
				for (j = 0; j <= deg; j++)
					B[k][j] = B[k][j] - t * B[i][j];	//make the elements below the pivot elements equal to zero or elimnate the variables
			}

		for (i = deg - 1; i >= 0; i--)	//back-substitution
		{	//x is an array whose values correspond to the values of x,y,z..
			coeffs[i] = B[i][deg];	//make the variable to be calculated equal to the rhs of the last equation
			for (j = 0; j < deg; j++)
				if (j != i)	//then subtract all the lhs values except the coefficient of the variable whose value is being calculated
					coeffs[i] = coeffs[i] - B[i][j] * coeffs[j];
			coeffs[i] = coeffs[i] / B[i][i];	//now finally divide the rhs by the coefficient of the variable to be calculated
		}
		return 0;
	}

	//Вычисляет радианы по градусам
	constexpr float rad(float deg)
	{
		return (M_PI / 180.0f) * deg;
	}

	//Вычисляет градусы по радианам
	constexpr float deg(float rad)
	{
		return (180.0f / M_PI) * rad;
	}

	//Вычисляет угол вектора в диапазоне 0..360 градусов по нормированным данным по отношению к Z плоскости
	template <typename T>
	T calcVectorAngleZ(T x, T y, T z)
	{
		T angle;
		if (!x && !z)
		{
			angle = (x > 0) ? 270.0f : 90.0f;
		}
		else
		{
			angle = deg(acosf(z / (x * x + y * y + z * z)));

			if (angle > 0) // Right semiplane
			{
				angle = (x > 0) ? (360.0f - angle) : (180.0f + angle);  // I or IV quater
			}
			else // Left semiplane
			{
				angle = (x > 0) ? fabsf(angle) : (180.0f + angle);  // II or III quater
			}
		}
		return angle;
	}

	/*
		FIR filter designed with
		http://t-filter.appspot.com

		sampling frequency: 330 Hz

		Фильтр, настроенный на подавление вибраций кузова от ДВС, скачков
		и прочего шума встречаемого обычно в грузовых спецмашинах
	*/
	static constexpr const float filterCoeffs[] = {
		0.025059906, 0.067343379, 0.015219092, 0.050648964, 0.037253808, 0.056544268,
		0.051281393, 0.061887313, 0.060008772, 0.063433684, 0.063433684, 0.060008772,
		0.061887313, 0.051281393, 0.056544268, 0.037253808, 0.050648964, 0.015219092,
		0.067343379, 0.025059906
	};

	constexpr float calcFilterSum()
	{
		float result = 0;
		for (int i = 0; i < sizeof(filterCoeffs) / sizeof(filterCoeffs[0]); i++)
			result += filterCoeffs[i];
		return result;
	}

	const float filterCoeffSum = calcFilterSum();

	template<typename T, unsigned int id>
	T filter(T value)
	{
		static T filterSum[(sizeof(filterCoeffs) / sizeof(float))];
		static unsigned char filterSumIterator = 0;
		T result = 0;

		filterSum[filterSumIterator++] = value;
		if (filterSumIterator == (sizeof(filterCoeffs) / sizeof(float)))
			filterSumIterator = 0;

		for (unsigned char i = 0; i < (sizeof(filterCoeffs) / sizeof(float)); i++)
		{
			if (filterSum[i] == 0)
				filterSum[i] = value;
			result += filterSum[i] * filterCoeffs[(i + filterSumIterator) % (sizeof(filterCoeffs) / sizeof(float))];
		}
		return result / filterCoeffSum;
	}

	class PID
	{
	public:
		PID() {}
		PID(float& in, float& out, float kp = 0, float ki = 0, float kd = 0, bool enable = true) :
			input(&in), output(&out), kP(kp), kI(ki), kD(kd)
		{
			enabled = enable;
			prevErr = in;
		}
		void calc(float target)
		{
			if (!((sumMin == 0) && (sumMax == 0) && (dstMin == 0) &&
				(dstMax == 0) && (srcMin == 0) && (srcMax == 0)))
			{
				err = math::bound2(target, srcMin, srcMax) - math::bound2((*input) + inOffset, srcMin, srcMax);
				sum = math::bound2(sum + err, sumMin, sumMax);

				P = err * kP;
				I = sum * kI;
				D = (err - prevErr) * kD;

				if (enabled)
					*output = math::bound2(P + I + D + outOffset, dstMin, dstMax);
				prevErr = err;
			}
		}
		void setOutMinMax(float min, float max)
		{
			dstMin = min;
			dstMax = max;
		}
		void setInMinMax(float min, float max)
		{
			srcMin = min;
			srcMax = max;
		}
		void setSumMinMax(float min, float max)
		{
			sumMin = min;
			sumMax = max;
		}
		void setInputOffset(float offset)
		{
			inOffset = offset;
		}
		void setOutputOffset(float offset)
		{
			outOffset = offset;
		}
		void setCoeffP(float p)
		{
			kP = p;
		}
		void setCoeffI(float i)
		{
			kI = i;
		}
		void setCoeffD(float d)
		{
			kD = d;
		}
		void reset()
		{
			prevErr = 0;
			err = 0;
			sum = 0;
		}
		void clearSum()
		{
			sum = 0;
		}
		void setEnabled(bool state)
		{
			enabled = state;
		}
		bool isEnabled()
		{
			return enabled;
		}
	private:
		float* input = NULL;
		float* output = NULL;
		float kP = 0;
		float kI = 0;
		float kD = 0;
		float err = 0;
		float prevErr = 0;
		float sum = 0;
		float dstMin = 0;
		float dstMax = 0;
		float srcMin = 0;
		float srcMax = 0;
		float sumMin = 0;
		float sumMax = 0;
		float inOffset = 0;
		float outOffset = 0;

		float P = 0, I = 0, D = 0;

		bool enabled = true;
	};

	template<typename T = float>
	class SDADC
	{
	public:
		SDADC(T minVal = 0, T maxVal = 1, T threshold = 0.5f, T hysteresis = 0) :
			sum(0), thr(threshold), min(minVal), max(maxVal), hyst(hysteresis), output(false) {}
		bool calc(T in)
		{
			sum = bound2<T>(sum + in - (T)((int)output) * (max - min), min, max);
			if (output)
			{
				if (((sum * 2) < (thr * 2 - hyst)))
					output = false;
			}
			else
			{
				if (((sum * 2) > (thr * 2 + hyst)))
					output = true;
			}
			return output;
		}
	private:
		T sum;
		T thr;
		T min;
		T max;
		T hyst;
		bool output;
	};

	/*
	1) По накопленным ранее данным и имеющимся моделям изменения
	вектора состояния объекта производит экстраполяцию вектора
	состояния на следующий шаг

	2) Сравнивает результаты экстраполяции с наблюдениями.
	При этом часто наблюдаются не сами компоненты вектора
	состояния, а некоторые функции от них.Поэтому используется
	дискриминатор — устройство, сигнал на выходе которого
	указывает на ошибку экстраполирования — разницу между
	ожидаемым (экстраполированным в шаге 1) вектором и наблюдаемым.

	3) Экстраполяция корректируется с помощью сигнала
	дискриминатора.При этом сигнал дискриминатора берется с
	некоторым весовым коэффициентом.Вся соль фильтра — как
	посчитать этот коэффициент, чтобы в итоге получить оптимальные
	оценки в смысле минимизации СКО. Фильтр Калмана использует
	коэффициент, являющийся решением уравнений Рикатти. Уравнение
	Рикатти оперирует следующими параметрами(упрощенно) :
	точностью уже имеющегося вектора состояния, моделью движения
	объекта, точностью наших средств измерения(наблюдения).
	Решение уравнения — оптимальный, в указанном смысле,
	коэффициент. + новая матрица дисперсий, говорящая, с какой
	точностью теперь мы будем знать измеряемые параметры.
	Скорректированная экстраполяция называется оценкой вектора
	состояния на данном шаге.Далее переходим к пункту 1.
	*/
	template<int Size>
	class Kalman
	{
	public:
		Kalman(float(**transforms)(float), float* coeffs)
		{
			linear = false;
			for (int i = 0; i < (Size * Size); i++)
				dataTransforms[i] = transforms[i];
			for (int i = 0; i < Size; i++)
				transformCoeffs[i] = coeffs[i];
		}
		Kalman(float* transforms, float* data, float* coeffs)
		{
			linear = true;
			for (int i = 0; i < (Size * Size); i++)
				((float*)dataTransforms)[i] = transforms[i];
			for (int i = 0; i < Size; i++)
				transformCoeffs[i] = coeffs[i];
		}
		void init(float* input)
		{
			prevEstimatePtr = estimateBuffer[0];
			currentEstimatePtr = estimateBuffer[1];
			initialization(input);
			prediction(prevEstimatePtr, currentEstimatePtr);
		}
		void calc(float* input)
		{
			update(input);
			prediction(prevEstimatePtr, currentEstimatePtr);
			swap(currentEstimatePtr, prevEstimatePtr);
		}
		float* getData()
		{
			return currentEstimatePtr;
		}
	private:
		void initialization(float* dataPtr)
		{
			for (int i = 0; i < Size; i++)
				prevEstimatePtr[i] = dataPtr[i];
		}
		void prediction(float* currentData, float* predictedData)
		{
			for (int i = 0; i < Size; i++)
			{
				float sum = 0;
				for (int j = 0; j < Size; j++)
				{
					if (linear)
						sum += ((float*)dataTransforms)[j] * currentData[j];
					else
						sum += ((float(**)(float))dataTransforms)[j](currentData[j]);
				}
				predictedData[i] = currentData[i] + sum;
			}
		}
		void update(float* dataPtr)
		{
			for (int i = 0; i < Size; i++)
				currentEstimatePtr[i] = prevEstimatePtr[i] + transformCoeffs[i] * (dataPtr[i] - prevEstimatePtr[i]);
		}

		float transformCoeffs[Size];
		float estimateBuffer[2][Size];
		float* prevEstimatePtr;
		float* currentEstimatePtr;
		void* dataTransforms[Size * Size];
		bool linear;
	};

} // namespace math
