#pragma once

#ifdef ARF_DLL
 #ifdef ARF_EXPORTS
  #define ARF_API __declspec(dllexport)
 #else
  #define ARF_API __declspec(dllimport)
 #endif
#else
 #define ARF_API
#endif

#include "arf_math.h"

#include <vector>
#include <unordered_map>
#include <string_view>
#include <memory>

#include <dPosition.h>


namespace ARF
{
	class Antenna;

	void ARF_API enable_debug(bool value);
	void ARF_API assert_version_runtime(std::string_view v);
	static constexpr const std::string_view version = "1.32";

	typedef uint64_t scan_signal_id_t;
	typedef uint64_t impl_track_id_t;

	struct ScanResult
	{
		struct Item
		{
			dVector bodyCoords;
			double closingVelocity = -dInf;
            double power_dBm = -dInf;
            bool isValid() const { return isfinite(power_dBm); }
            int nctr_id = -1;
			uint64_t signal_id = 0;
		};

		std::vector<Item> contacts;
		double jam_dB = 0.0;
		bool valid = false;
	};

	struct TrackResult
	{
		dVector localPosition;
		double closingVelocity = 0;
		bool valid = false;
		int nctrId = -1;
		int numObjects = 0;
		double SNR_dB = 0;
		double power_dBm = 0;
		double jam_dB = 0;
	};

	struct IFFResult
	{
		std::vector<dVector> body_coordinates;
	};




	struct RawSample
	{
		cVector subpos;
		float strength = 0;
	};

	struct RawMeasure
	{
		std::vector<RawSample> data;
		dPosition localAntennaPos;
		dPosition insAntennaPos;
		double doppler_vel;
		double doppler_stddev;
		double total_power;
	};



	struct RawResult
	{
		std::vector<RawMeasure> measures;
		int scanSegment;
		span range;
		float az1;
		float az2;
		float el1;
		float el2;

		typedef std::shared_ptr<RawResult> SPtr;
		typedef std::shared_ptr<const RawResult> SCPtr;
	};






	struct SARResult
	{
		typedef std::shared_ptr<std::vector<float>> buffer_t;

		buffer_t buffer;
		int imageWidth;
		int imageHeight;
		bool finalized = false;

		bool valid() const {
			return finalized
			&& buffer
			&& buffer->size()!=0
			&& buffer->size() == imageWidth * imageHeight;
		}
	};


	enum ObjectFilter
	{
		AirUnits = 0x1,
		SeaUnits = 0x2,
		GroundUnits = 0x4,
		TrafficUnits = 0x8,
		MapMovingObjects = 0x16
	};


	struct FreqParams
	{
		//Doppler parameters
		double dop_alt_reject_hwidth = 50.0;
		double dop_alt_reject_att = 1E-20;
		double dop_prim_reject_hwidth = 36.0;
		double dop_prim_reject_att = 1E-20;
		double CFAR_adjust = 1.0;
		double dop_bin_resolution = 10.0;

		span dop_limits = { -dInf, dInf };

		void disableAltitudeNotch()		{ dop_alt_reject_hwidth = 0; }
		
		void disablePrimaryNotch()		{ dop_prim_reject_hwidth = 0; }
		void setAltitudeVel(double vel)	{ dop_alt_reject_center = vel; }
		void setPrimaryVel(double vel)	{ dop_prim_reject_center = vel;	}
		bool in_primary_notch(double doppler_vel) const
		{
			return std::abs(ambiguous_doppler(doppler_vel, dop_prim_reject_center) - dop_prim_reject_center) < dop_prim_reject_hwidth;
		}

		bool in_altitude_notch(double doppler_vel) const
		{
			return std::abs(ambiguous_doppler(doppler_vel, dop_alt_reject_center) - dop_alt_reject_center) < dop_alt_reject_hwidth;
		}


		bool doppler_accept(double dopplerVel) const
		{
			bool gnd_notch = in_primary_notch(dopplerVel);
			bool alt_notch = in_altitude_notch(dopplerVel);
			return !gnd_notch && !alt_notch;
		}

		double filter_gain(double doppler_vel) const
		{
			return (in_primary_notch(doppler_vel) ? dop_prim_reject_att : 1.0)
				*  (in_altitude_notch(doppler_vel) ? dop_alt_reject_att : 1.0);
		}

		double ambiguous_doppler(double true_doppler) const
		{
			return span::center_half(dop_limits.center(), prf_doppler_ambiguity()*0.5).modulo(true_doppler);
		}

		double ambiguous_doppler(double true_doppler, double track_doppler) const
		{
			return span::center_half(track_doppler, prf_doppler_ambiguity()*0.5).modulo(true_doppler);
		}

		double ambiguous_range_prf1(double true_range, double track_range) const
		{
			return span::center_half(track_range, prf1_range_ambiguity()*0.5).modulo(true_range);
		}

		double ambiguous_range_prf2(double true_range, double track_range) const
		{
			return span::center_half(track_range, prf2_range_ambiguity()*0.5).modulo(true_range);
		}

		//PRF parameters
		enum Mode { MultiFrequency, InterleavedFrequency };
		Mode mode = MultiFrequency;
		double carrier_ghz = 10.0;
		double base_prf_khz = 100.0;
		double commutation_period_s = 0.25;
		double delta_prf_khz = 4.0;
		double sampling_rate_m = 30.0;
		int prf_count = 2;
		span dist_resolve_span;
		bool auto_notch = true;


		//Computed automatically by AbstractRadar
		double dop_alt_reject_center = 0;
		double dop_prim_reject_center = 0;
		double prf1_khz = 100;
		double prf2_khz = 101;
		static constexpr const double c = 299792458.0;

		double prf1_range_ambiguity() const { return c / (prf1_khz*1E3); }
		double prf2_range_ambiguity() const { return c / (prf2_khz*1E3); }
		double prf_doppler_ambiguity() const { return c * base_prf_khz*1E3 / (carrier_ghz*1E9 * 2); }
	};

	class AbstractScanPattern;
	
	struct missile_link_t
	{
		impl_track_id_t implementation_track_id = 0;
		scan_signal_id_t scan_signal_id = 0;
	};

	typedef std::vector<missile_link_t> missile_links_t;


	class SplatImager;
}
