#pragma once


#include <wMatrix33.h>
#include <wPosition3.h>
#include <polar.h>
#include <ed/string.h>

#include <array>
#include <iosfwd>
#include <limits>
#include <algorithm>


namespace ARF
{
	static constexpr const double dNaN = std::numeric_limits<double>::quiet_NaN();
	static constexpr const double dInf = std::numeric_limits<double>::infinity();
	static constexpr const float fNaN = std::numeric_limits<float>::quiet_NaN();

	template<typename T>
	T sq(T v) { return (v) * (v); }


	//Returns true if the number is valid (ie not NaN and not infinite)
	inline bool isvalid(double number) { return std::isfinite(number); }
	inline bool isvalidpolar(Math::PolarNormalized p) { return std::isfinite(p.elevation) && std::isfinite(p.azimuth); }
	inline bool isvalidpolar(Math::Polar p) { return std::isfinite(p.elevation) && std::isfinite(p.azimuth) && std::isfinite(p.distance); }
	inline bool isvalidvec(dVector v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
	inline bool isvalidvec(cVector v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }

	//Returns the unsigned modulo d : value of d cycles back to 0, value of -1 cycles to d-1
	inline int umod(int number, int d)
	{
		return number % d + (number < 0 ? d : 0);
	}


	//Returns the true (physical) modulo instead of the c++ modulo (which rounds towards zero)
	inline double dmod(const double value, const double start, const double end)
	{
		const double width = end - start;   // 
		const double offsetValue = value - start;   // value relative to 0
		return (offsetValue - (floor(offsetValue / width) * width)) + start;
	}


	//Linear interpolation
	template<typename T>
	inline T lerp(double x, double x0, double x1, T y0, T y1)
	{
		double t1 = std::clamp((x - x0) / (x1 - x0), 0.0, 1.0);
		double t0 = 1.0 - t1;
		return y0 * t0 + y1 * t1;
	}


	//Multipoint lerp
	template<typename T, size_t N>
	inline T lerp(double x, const std::array<double, N>& xt, const std::array<T, N>& yt)
	{
		static_assert(N >= 2);
		size_t i = 0;
		while ((i < N - 1) && x > xt[i + 1])
			++i;
		return lerp(x, xt[i], xt[i + 1], yt[i], yt[i + 1]);
	}


	//Various useful span(interval) functions
	template<typename T>
	struct span_T
	{
		static constexpr const T T_Inf = std::numeric_limits<T>::infinity();
		span_T() : min(T_Inf), max(-T_Inf) {}	//This is a null interval, contains nothing
		span_T(T min, T max) : min(min), max(max) { }
		T min;
		T max;
		T width() const { return std::max(max - min, (T)0); }
		T half_width() const { return width()*(T)0.5; }
		T center() const { return (max + min)*(T)0.5; }
		bool null() const { return !(max >= min); }		//Written like this to return true when anything is NaN
		bool contains(T val) const { return val >= min && val <= max; }
		bool overlaps(const span_T& other) const { return !(intersected(other).null()); }
		T coordinate(T val) const { return (val - min) / (max - min); }
		T from_coordinate(T x) const { return min + (max - min)*x; }
		T sample(T u) const { return from_coordinate(u); }
		T clamp(T val) const { return std::clamp(val, min, max); }
		T modulo(T val) const { return dmod(val, min, max); }
		span_T shifted(T val) const { return span_T(min+val, max+val); }
		span_T united(T val) const { return span_T(std::min(min, val), std::max(max, val)); }
		span_T united(const span_T& other) const { return span_T(std::min(min, other.min), std::max(max, other.max)); }
		span_T intersected(const span_T& other) const { return span_T(std::max(min, other.min), std::min(max, other.max)); }
		span_T & unite(T val) { *this = united(val); return *this; }
		span_T & unite(const span_T& other) { *this = united(other); return *this; }
		span_T & intersect(const span_T& other) { *this = intersected(other); return *this; }
		static span_T center_half(T center, T half) { return span_T(center - half, center + half); }
		static span_T center_width(T center, T width) { return span_T(center - width*(T)0.5, center + width* (T)0.5); }
		bool operator==(const span_T & other) const { return min == other.min && max == other.max; }
		bool operator!=(const span_T & other) const { return min != other.min || max != other.max; }
	};

	typedef span_T<double> span;
	typedef span_T<float> spanf;




	template<typename T, size_t N>
	class ringbuffer
	{
	public:
		ringbuffer() : _cur(data.begin()), _end(data.begin()) { }
		ringbuffer(const ringbuffer<T, N> & other)
			: data(other.data),
			_cur(data.begin() + std::distance<citer_t>(other.data.cbegin(), other._cur)),
			_end(data.begin() + std::distance<citer_t>(other.data.cbegin(), other._end))
		{ }

		ringbuffer<T, N> & operator=(const ringbuffer<T, N> & other)
		{
			data = other.data;
			_cur = data.begin() + std::distance<citer_t>(other.data.cbegin(), other._cur);
			_end = data.begin() + std::distance<citer_t>(other.data.cbegin(), other._end);
			return *this;
		}

		void push(T val)
		{
			*_cur = val;
			if (_cur++ == _end)
				++_end;

			if (_cur == data.end())
				_cur = data.begin();
		}

		size_t rank(size_t i) const
		{
			int d = std::distance((citer_t)_cur, cbegin()) + 1;
			return (d < 0) ? (size_t)(-d) : (N - d);
		}

		void clear() { _cur = _end = data.begin(); }
		size_t size() const { return std::distance(cbegin(), cend()); }
		T & operator[](size_t i) { return data[i]; }
		const T & operator[](size_t i) const { return data[i]; }

		typename std::array<T, N>::iterator begin() { return data.begin(); }
		typename std::array<T, N>::iterator end() { return _end; }
		typename std::array<T, N>::const_iterator begin() const { return data.begin(); }
		typename std::array<T, N>::const_iterator end() const { return _end; }
		typename std::array<T, N>::const_iterator cbegin() const { return data.begin(); }
		typename std::array<T, N>::const_iterator cend() const { return _end; }

	private:
		using arr_t = typename std::array<T, N>;
		using iter_t = typename std::array<T, N>::iterator;
		using citer_t = typename std::array<T, N>::const_iterator;

		arr_t data;
		iter_t _cur;
		iter_t _end;
	};





	template<typename value_type, size_t N> class nvec : public std::array<value_type, N>
	{
	public:
		using nvec_type = nvec <value_type, N>;

		nvec(value_type val = 0)
		{
			this->fill(val);
		}

		nvec_type & operator=(value_type scalar)
		{
			this->fill(scalar);
			return *this;
		}

		nvec_type operator+(const nvec_type & other) const
		{
			nvec_type result;
			for (size_t i = 0; i < N; ++i)
				result[i] = this->at(i) + other.at(i);
			return result;
		}

		nvec_type operator+(const value_type & scalar) const
		{
			nvec_type result;
			for (size_t i = 0; i < N; ++i)
				result[i] = this->at(i) + scalar;
			return result;
		}

		nvec_type & operator+=(const nvec_type & other)
		{
			for (size_t i = 0; i < N; ++i)
				this->at(i) += other.at(i);
			return *this;
		}

		nvec_type & operator+=(const value_type & scalar)
		{
			for (size_t i = 0; i < N; ++i)
				this->at(i) += scalar;
			return *this;
		}

		nvec_type operator-(const nvec_type & other) const
		{
			nvec_type result;
			for (size_t i = 0; i < N; ++i)
				result[i] = this->at(i) - other.at(i);
			return result;
		}

		nvec_type operator-(const value_type & scalar) const
		{
			nvec_type result;
			for (size_t i = 0; i < N; ++i)
				result[i] = this->at(i) - scalar;
			return result;
		}

		nvec_type & operator-=(const nvec_type & other)
		{
			for (size_t i = 0; i < N; ++i)
				this->at(i) -= other.at(i);
			return *this;
		}

		nvec_type & operator-=(const value_type & scalar)
		{
			for (size_t i = 0; i < N; ++i)
				this->at(i) -= scalar;
			return *this;
		}

		value_type operator*(const nvec_type & other) const
		{
			value_type result = 0;
			for (size_t i = 0; i < N; ++i)
				result += this->at(i) * other.at(i);
			return result;
		}

		nvec_type operator*(const value_type & factor) const
		{
			nvec_type result;
			for (size_t i = 0; i < N; ++i)
				result[i] = this->at(i) * factor;
			return result;
		}

		nvec_type & operator*=(const value_type & factor)
		{
			for (size_t i = 0; i < N; ++i)
				this->at(i) *= factor;
			return *this;
		}

		nvec_type & operator/=(const value_type & factor)
		{
			for (size_t i = 0; i < N; ++i)
				this->at(i) /= factor;
			return *this;
		}
	};



	template<typename value_type, size_t order> class polynomial_regressor
	{
	public:
		template <size_t N> using arrT = std::array<value_type, N>;
		template <size_t N> using arrD = std::array<double, N>;
		template <size_t N> using matD = std::array<std::array<double, N>, N>;

		struct point { double u; value_type v; double w = 1.0; };

		static inline const value_type zero = value_type()*0.0;

		template<size_t derivative = 0>
		value_type func(double u) const
		{
			value_type result = zero;

			double u_pow_j = 1.0;

			for (size_t i = derivative; i <= order; ++i)
			{
				int j = (int)i - derivative;
				int coef = 1;
				for (int k = 1; k <= derivative; ++k)
					coef *= (j + k);

				if (j > 0)
					u_pow_j *= (u - u_ref);

				result += polynom[i] * u_pow_j * coef;
			}
			return result;
		}

		template<typename container, typename item_type = container::value_type>
		void compute(const container & buffer, const double item_type::*u, const value_type item_type::*v, const double item_type::*w)
		{
			static constexpr const size_t m = order + 1;
			arrD<order * 2 + 1> S;
			arrT<order + 1> T;
			S.fill(0.0);
			T.fill(zero);
			polynom.fill(zero);
			samples = 0;
			correlation = 0;
			variance_u = 0;
			residual_variance_v = 0;
			total_weight = 0;
			u_ref = 0;

			if (buffer.cbegin() == buffer.cend())
			{
				return;
			}

			u_ref = (*buffer.cbegin()).*u;

			for (const item_type & p : buffer)
			{
				++samples;
				double u_pow_j = 1.0;
				for (size_t j = 0; j <= order * 2; ++j)
				{
					S[j] += u_pow_j * p.*w;
					u_pow_j *= (p.*u - u_ref);
				}

				u_pow_j = 1.0;
				for (size_t j = 0; j <= order; ++j)
				{
					T[j] += p.*v * u_pow_j * p.*w;
					u_pow_j *= (p.*u - u_ref);
				}
			}


			for (size_t l = 0; l <= order; ++l)
			{
				R[l] = T[order - l];
				for (size_t c = 0; c <= order; ++c)
				{
					M[l][c] = S[order * 2 - l - c];
				}
			}





			//Invert the linear system by pivot
			for (size_t c = 0; c <= order; ++c)
				for (size_t l = c + 1; l <= order; ++l)
					zeroify(l, c);

			for (size_t c = 0; c <= order; ++c)
				for (size_t l = 0; l < c; ++l)
					zeroify(l, c);

			for (size_t c = 0; c <= order; ++c)
			{
				polynom[order - c] = M[c][c] != 0.0 ? R[c] / M[c][c] : zero;
			}

			//Compute sum of residuals and total variance
			double SSR = 0.0;
			double SST = 0.0;
			for (size_t l = 0; l <= order; ++l)
				for (size_t c = 0; c <= order; ++c)
					SSR += (polynom[l] * polynom[c]) * S[c + l];

			for (size_t l = 0; l <= order; ++l)
				SSR -= 2.0*(polynom[l] * T[l]);

			for (const item_type & p : buffer)
			{
				SSR += (p.*v)*(p.*v)*(p.*w);
				SST += (p.*v)*(p.*v)*(p.*w);
			}

			SST -= T[0] * T[0] / S[0];

			correlation = 1.0 - SSR / SST;
			variance_u = S[2] / S[0] - sq(S[1] / S[0]);
			residual_variance_v = SSR / S[0];
			total_weight = S[0];
		}



		void zeroify(size_t l1, size_t l2)
		{
			if (M[l2][l2] == 0.0)
			{
				std::swap(R[l1], R[l2]);
				for (size_t c = 0; c <= order; ++c)
					std::swap(M[l1][c], M[l2][c]);
			}
			else
			{
				double f = -M[l1][l2] / M[l2][l2];
				R[l1] += R[l2] * f;
				for (size_t c = 0; c <= order; ++c)
					M[l1][c] += M[l2][c] * f;
				M[l1][l2] = 0;
			}
		}

		size_t samples = 0;
		double correlation = 0;
		double variance_u = 0;
		double residual_variance_v = 0;
		double total_weight = 0;
		double u_ref;
		arrT<order + 1> polynom;
		matD<order + 1> M;
		arrT<order + 1> R;
	};


	template<typename value_type, size_t N, size_t order = 1> class regressor_smoother
	{
	public:
		regressor_smoother() : weight_rampup(N / 10), weight_rampdown(N / 2) {}

		template<size_t derivative = 0>
		value_type smoothed_value(double time) const
		{
			return regressor.func<derivative>(time);
		}

		void add_value(value_type value, double time)
		{
			buffer.push({ value, time, 1.0 });

			for (size_t i = 0; i < buffer.size(); ++i)
			{
				size_t j = buffer.rank(i);
				double w1 = std::min((double)(i + 1) / (weight_rampup + 1), 1.0);
				double w2 = std::min((double)(N - i) / (weight_rampdown + 1), 1.0);
				buffer[i].weight = w1 * w2;
			}

			regressor.compute(buffer, &buffer_entry::time, &buffer_entry::value, &buffer_entry::weight);
		}

		void reset()
		{
			buffer.clear();
			regressor.compute(buffer, &buffer_entry::time, &buffer_entry::value, &buffer_entry::weight);
		}

	private:

		struct buffer_entry
		{
			value_type value;
			double time;
			double weight;
		};

		size_t weight_rampup;
		size_t weight_rampdown;
		ringbuffer<buffer_entry, N> buffer;
		polynomial_regressor<value_type, order> regressor;
	};




	template<typename value_type, size_t N> class averager
	{
	public:
		void add_value(value_type value)
		{
			buffer.push(value);

			mean = 0.0f;

			for (value_type v : buffer)
				mean += v;

			mean *= 1.0f / N;
		}

		void reset()
		{
			buffer.clear();
			mean = 0;
		}

		value_type mean;

	private:
		ringbuffer<value_type, N> buffer;
	};


	template<typename value_type, size_t N> class averager_sigma
	{
	public:
		void add_value(value_type value)
		{
			buffer.push(value);

			mean = 0;
			sigma = 0;
			for (value_type v : buffer)
			{
				mean += v;
				sigma += v * v;
			}

			mean = mean / N;
			sigma = std::sqrt(sigma / N - mean * mean);
		}

		void reset()
		{
			buffer.clear();
			mean = 0;
			sigma = 0;
		}

		value_type mean;
		value_type sigma;

	private:
		ringbuffer<value_type, N> buffer;
	};
	
	inline double dB_coef(double dB) { return std::pow(10.0, dB / 20.0); }
	inline double to_dB(double coef) { return 20.0*std::log10(coef); }

}