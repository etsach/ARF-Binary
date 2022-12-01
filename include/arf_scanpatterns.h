#pragma once

#include "arf.h"
#include <algorithm>
#include <ed/vector.h>




namespace ARF
{
	class Antenna;


	class ARF_API AntennaContext
	{
	public:
		AntennaContext(const Antenna * antenna) : p(antenna) {}
		const dPosition & localPos() const;
		bool isGimbalCloseToLimit(double marginDeg) const;
		bool isSpeedLimited(const dVector & newDirection, double dt) const;
		bool isInsideLimits(const dVector & newDirection) const;
		std::pair<double, double> getConePatternGimbalLimits(const wMatrix33d & basis, double alpha) const;

	private:
		const Antenna * p;
	};



	class ARF_API AbstractScanPattern
	{
	public:
		AbstractScanPattern() { }

		struct ScanState
		{
			dVector direction;
			int total_segment = 0;
			bool emit = false;
			int twsId = -1;
		};

		virtual ScanState update(AntennaContext ant, const dPosition & insPosition, double dt) = 0;
		virtual int getSubSegment() const { return 0; }
		virtual int getTotalSegment() const { return 0; }
		virtual void reset() = 0;
	};


	class ARF_API AbstractLineScanPattern : public AbstractScanPattern
	{
	public:
		AbstractLineScanPattern(const wMatrix33d & orientation, bool stab_pitch, bool stab_roll) : orientation(orientation), stab_pitch(stab_pitch), stab_roll(stab_roll) { }
		ScanState update(AntennaContext ant, const dPosition & insPosition, double dt) override;
		int getSubSegment() const override { return currentSubSegment; }
		int getTotalSegment() const override { return currentSegment; }

		void reset() override { jumpingToLine = true; endOfLine = false; currentSegment = 0; currentSubSegment = 0; }

		int getCurrentSegment() const { return currentSegment; }
		bool isEndOfLine() const { return endOfLine; }
		bool isEndOfPattern() const { return endOfFrame; }
		int getLineCount() const { return std::max((int)line_angles.size(), 1); }
		int getFrame() const { return currentFrame; }
		int getBar() const { return currentSubSegment; }
		int getAbsoluteBar() const { return currentSegment; }

	protected:
		static constexpr const double deg_to_rad = M_PI / 180.0;
		ed::vector<double> line_angles;
		span scan_angle_span = { -60.0*deg_to_rad, 60.0*deg_to_rad };
		double scanSpeed = 100.0 * deg_to_rad;

		struct TWS_ScanAngles { double scan_angle = 0; int line = 0; bool done = false; };
		ed::vector<TWS_ScanAngles> tws_scan_angles;

		void setOrientation(const wMatrix33d & new_orientation) { this->orientation = new_orientation; }

	private:
		wMatrix33d orientation;
		bool stab_pitch;
		bool stab_roll;
		bool jumpingToLine = true;
		bool endOfLine = false;
		bool endOfFrame = false;
		size_t currentSegment = 0;
		size_t currentSubSegment = 0;
		size_t currentFrame = 0;

		virtual wMatrix33d getConeBasis(const dPosition & insPosition) const;
	};


	class ARF_API HorizontalScanPattern : public AbstractLineScanPattern
	{
	public:
		HorizontalScanPattern() : AbstractLineScanPattern(wMatrix33d(), true, true) { }

		void setLines(ed::vector<double> && angles);
		void setLines(int line_count, double elev_center, double elev_step);
		void setAzimuthBounds(double leftRad, double rightRad) { scan_angle_span = { leftRad, rightRad }; }
		void setAzimuthSpan(span scan_span) { scan_angle_span = scan_span; }
		void setScanSpeed(double rads) { scanSpeed = rads; }
		double getAzimuthCenter() const { return scan_angle_span.center(); }
		double getAzimuthWidth() const { return scan_angle_span.width(); }
		double getAzimuthLeft() const { return scan_angle_span.min; }
		double getAzimuthRight() const { return scan_angle_span.max; }
		double getElevationCenter() const { return (line_angle_max+line_angle_min)*0.5; }
		double getElevationTop() const { return line_angle_max; }
		double getElevationBottom() const { return line_angle_min; }
		double getSpeed() const { return scanSpeed; }

		void setNumTrackedDirections(size_t n) { if (tws_scan_angles.size() != n) tws_scan_angles.resize(n); }
		void setTrackedDirection(size_t i, double scan_angle, int line = 0);

	protected:
		double line_angle_min = 0;
		double line_angle_max = 0;
	};



	class ARF_API VerticalScanPattern : public AbstractLineScanPattern
	{
	public:
		VerticalScanPattern() : AbstractLineScanPattern(wMatrix33d({ 1,0,0 }, { 0,0,-1 }, { 0,1,0 }), false, false) { line_angles = { 0 }; }

		void setLines(int line_count, double angle_step);
		void setElevationRange(double elev_min, double elev_max) { scan_angle_span = { elev_min, elev_max }; }
		void setScanSpeed(double rads) { scanSpeed = rads; }
		double getElevationMin() const { return scan_angle_span.min; }
		double getElevationMax() const { return scan_angle_span.max; }
		double getSpeed() const { return scanSpeed; }
	};




	class ARF_API WorldFixedPattern : public AbstractScanPattern
	{
	public:
		WorldFixedPattern() { }
		ScanState update(AntennaContext ant, const dPosition & insPosition, double dt) override;
		void reset() override {}

		void setFixedWorldDirection(dVector dir) { fixedDirection = dir; world = true; }
		void setFixedLocalDirection(dVector dir) { fixedDirection = dir; world = false; }

	private:
		dVector fixedDirection;
		bool world = true;
	};
}