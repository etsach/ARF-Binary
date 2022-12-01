#pragma once

#include <arf.h>

#include <array>
#include <memory>



namespace render
{
	class Texture;
	class LockedTexture;
}

class IndicationBaseMaterial;

namespace ARF
{
	class ARF_API LineImager
	{
	public:
		static constexpr const int mapWidth = 512;
		static constexpr const int mapHeight = 512;

		LineImager(const std::string baseName, size_t planes);
		~LineImager();
		void clear();
		void clear(float thmin, float thmax);
		void clear_outside(float thmin, float thmax);
		void clear_older_segments();
		void commit() { commit_request = true; }
		void setAutoCommit(bool enabled) { autocommit = enabled; }
		void assignToMaterial(IndicationBaseMaterial * material) const;
		void updateTextures() const;
		void saveToFile(const char* path) const;

		void updateRawBeam(ARF::RawResult::SCPtr rawResult);

		void setGain(float _gain, float _exponent = 1.0f) { gain = _gain; exponent = _exponent; }
		void setProjectionPPI(span range, float aspectRatio);
		void setProjectionBScope(span range);
		void setProjectionEScope(span range);
		void setProjectionRHI(span range);


		void updateAzimuthShift(float azs) { az1_shift = az2_shift; az2_shift = azs; }
		void setAzimuthShift(float az1s, float az2s) { az1_shift = az1s; az2_shift = az2s; }
		void setActivePlaneCount(size_t value);
		int activePlaneCount() const { return activePlanes; }
		void setHeightBounds(float h0, float h1) { height0 = h0; height1 = h1; }
		void setHeightGradient(float sa, float sb, osg::Vec3f top, osg::Vec3f bottom, float gamma = 1.0f);
		void setHeightGradient(osg::Vec3f top, osg::Vec3f bottom, float gamma = 1.0f);
		void setHeightStepping(float ha, float hb, osg::Vec3f color, float gamma = 1.0f);
		void disableHeight() { heightEnabled = false; }
		void setStrengthGradient(float sa, float sb, osg::Vec3f ca, osg::Vec3f cb, float gamma = 1.0f);
		void setStrengthGradient(osg::Vec3f full, float gamma = 1.0f);
		void setStrengthStepping(float sa, float sb, osg::Vec3f color, float gamma = 1.0f);
		void disableStrength() { strengthEnabled = false; }
		void setClearColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a) {	clearColor = { r,g,b,a }; }
		void setSlantCorrection(bool enable) { slantCorrection = enable; }


	private:
		enum Projection { PPI, BScope, EScope, RHI };
		typedef std::array<osg::Vec3f, 256> ColorLUT;

		void setGradient(ColorLUT * lut, float sa, float sb, osg::Vec3f ca, osg::Vec3f cb, float gamma = 1.0f) const;
		void signalChanges();

		bool autocommit = true;
		const size_t planeCount;
		size_t activePlanes = 0;

		struct HistData
		{
			unsigned char plane = 255;
			unsigned char strength = 0;
		};

		struct Color
		{
			//Default to black
			unsigned char R = 0;
			unsigned char G = 0;
			unsigned char B = 0;
			unsigned char A = 0;
		};

		std::array< Color, mapHeight*mapWidth> pixels;

		static constexpr const float k = 2.78389f;
		static const float rangeFunc(float x) { return (std::expf(x*k) - 1.0f) / (std::expf(k) - 1.0f); }


		void updateFromBFR_PPI(const ARF::RawResult & bfrResult);
		void clear_PPI(float azmin, float azmax);
		void updateFromBFR_RHI(const ARF::RawResult & bfrResult);
		void clear_RHI(float azmin, float azmax);
		void updateFromBFR_EScope(const ARF::RawResult & bfrResult);
		void clear_EScope(float azmin, float azmax);
		void updateFromBFR_BScope(const ARF::RawResult & bfrResult);
		void clear_BScope(float azmin, float azmax);
		void paint(const ARF::RawResult & bfrResult, int x, int y, float r, float azimuth);



		std::vector<HistData> histData;
		std::vector<cVector> heightVectors;
		std::unique_ptr<render::Texture> texture_RAM;
		std::unique_ptr<render::Texture> texture_GPU;
		mutable bool gpu_texture_outdated = true;
		mutable bool commit_request = true;

		float gain = 1;
		float exponent = 1;
		span imageRange = { 0,1 };
		bool slantCorrection = true;

		Projection projection = PPI;
		float PPI_aspectRatio = 1.0f;


		int lastSegment = 0;
		bool clear_needed = false;
		float painted_thmin = 0;
		float painted_thmax = 0;
		float curseg_thmin = 0;
		float curseg_thmax = 0;
		float th1 = 0;
		float th2 = 0;
		float az1_shift = 0;
		float az2_shift = 0;
		int current_plane = 0;




		float threshold = 0.01f;
		ColorLUT strengthLUT;
		bool strengthEnabled = true;

		ColorLUT heightLUT;
		bool heightEnabled = false;
		float height1 = 0.0f;
		float height0 = -1000.0f;

		Color clearColor = Color();
	};
}