#pragma once

#include <unordered_map>
#include <unordered_set>
#include <dPosition.h>
#include <ed/string.h>
#include <Avionics/avDevice.h>
#include <Avionics/Sensors/avLinkToTargetResponder.h>

#include "arf.h"
#include "arf_math.h"





class MovingObject;
class wsType;
struct RWR_event;



namespace ARF
{
	class BaseRadar;
	class SplatImager;

	class ARF_API AbstractRadar : public cockpit::avDevice, public cockpit::avLinkToTargetResponder
	{
	public:
		AbstractRadar();
		AbstractRadar(const AbstractRadar * shared_context_radar);
		~AbstractRadar();

		void update() override final;
		void initialize(unsigned char ID, const ed::string& Name, const ed::string& script_name) override final;
		void post_initialize() override final;

		virtual void radar_initialize() {}
		virtual void radar_post_initialize() {}
		virtual void pre_update() = 0;
		virtual void post_update() = 0;

		void set_ins_data(const dPosition & insPosition, const dVector & insVelocity);

		//Configuration - can only be called upon initialization
		void setPerformance(float dish_area, float radius3db_rad, float power, float distance, float rcs, float base_CFAR);
		void setGimbalSpeed(double rad_per_seconds);
		void clearGimbalLimits();
		void addGimbalConeLimit(double angle_rad, dVector axis = { 1,0,0 });
		void addGimbalPlanarLimit(double angle_rad, double orientation_rad);	//orientation : 0 = down, rad(90)=right, rad(180)=up, rad(270)=left
		void addNCTRSignature(int integerId, double dB, double angleDeg, const ed::string & type);
        void enableNCTR(bool track, bool scan);
        void disableGatePuloff(const ed::string & contact_type);

		const dPosition & getLocalAntennaPos() const;
		const dPosition & getInsAntennaPos() const;
		double insAntennaElevation() const;		//Replaced antennaElevation() in 1.12
		double insAntennaAzimuth() const;		//Replaced antennaAzimuth() in 1.12
		double localAntennaElevation() const { return Math::PolarNormalized(getLocalAntennaPos().x).elevation; }
		double localAntennaAzimuth() const { return Math::PolarNormalized(getLocalAntennaPos().x).azimuth; }
		bool isAntennaCloseToLimit(double margin_rad) const;
		bool isLocalDirectionCloseToLimit(const dVector & direction, double margin_rad) const;
		bool isWorldPointCloseToLimit(const dPoint & direction, double margin_rad) const;

		//Ground ranging
		bool hasGroundLock() const;
		double getGroundLockRange() const;
		double getGroundLockLevel() const;
		dPoint getGroundLockPoint() const;

		//Interferences
		void setInterferenceChannel(unsigned char code);
		void checkInterference(const RWR_event & rwr_event);

		//Video/Image objects
		std::unique_ptr<SplatImager> createSplatImager(const std::string & name, int size);

	protected:
		ObjectID launchFox1(int station, int substation, bool log_telemetry = false) const;
		ObjectID launchFox3(uint64_t track_id, int station, int substation, bool log_telemetry = false) const;
		ObjectID launchMadDog(int station, int substation, bool log_telemetry = false) const;

		bool hasTrackResult() const;
		bool hasScanResult() const;
		const ScanResult & lastScanResult() const;
		const TrackResult & lastTrackResult() const;
		const IFFResult & lastIFFResult() const;
		const RawResult::SCPtr & lastRawResult() const;
		const SARResult & lastSARResult() const;


		void setScanPattern(AbstractScanPattern * pattern);
		void clearScanPattern() { setScanPattern(nullptr); }
		int getScanSubSegment() const;
		int getScanTotalSegment() const;
		bool isEmitting() const;
		void setClipRange(double range);
		void enableRawMapObjects(bool enable);
		void enableHighQualityClutter(bool enable);
		void enableFullAsynchronous(bool enable);

		//Slaved direction function
		void setFixedWorldDirection(dVector worldDir);
		void setFixedLocalDirection(dVector localDir = { 1,0,0 });

		//Mode setting
		void disableAllModes();
		void setModeSTT(const FreqParams & frparams, span track_cvel, double track_range, double track_angle, double track_power);
		void setModeTWS(const FreqParams & scan_frparams, double inhibition_time, const FreqParams & track_frparams, span track_cvel, double track_range, double track_angle, double track_power);
		void setModeSCAN(const FreqParams & frparams, double inhibition_time, double angleResolution);
		void setModeRAW(double range, int timeSampling);
		void setModeRNG(double range);
		void setModeSAR_StartPatch(span range_span, span angle_span, double map_y, double displacement, std::array<double, 4> mask);
		void setModeSAR_Continue(span range_span, span angle_span);
		void setScanIFF();
		void setTrackIFF();
		void setRawMonopulseSharpening(double azimuth_rad, double top_rad, double bottom_rad);
		void setRawSARSharpening(double integration_time);
		void setMissileLinkTrack(uint64_t implementation_track_id, uint64_t arf_scan_signal_id);
		void setUnitsDetectable(int filter);

		//Debug
		void toggle_debug_graphs();

	private:
		//Missile tracking
		bool is_tracking(ObjectID ID, unsigned weapon_type) const override;
		ObjectID get_target_to_engage() const override;
		double   get_target_range() const override;

		std::unique_ptr<BaseRadar>  impl;
	};


}
