#pragma once

#include <arf.h>

#include <array>
#include <memory>
#include <atomic>
#include <mutex>


namespace render
{
	class Texture;
	class LockedTexture;
}

class IndicationBaseMaterial;

namespace ARF
{
	class ThreadPool;

	class ARF_API SplatImager
	{
	public:
		const int mapWidth = 512;
		const int mapHeight = 512;

		SplatImager(std::shared_ptr<ThreadPool> threadPool,  const std::string baseName, const int size = 1024);
		~SplatImager();

		void setPages(size_t num_pages);
		void clear();
		void resetPage(size_t page);
		void assignToMaterial(IndicationBaseMaterial * material) const;
		void updateTextures() const;
		void saveToFile(const char * path) const;

		void updateRawBeam(size_t page, const ARF::RawResult::SCPtr & rawResult, float gain, double az1_shift, double az2_shift);
		void updateSAR_Patch(size_t page, const ARF::SARResult & sarResult);
		void finalize(size_t page, bool clear_after = false);





		void setGain(float _gain, float _exponent = 1.0f) { gain = _gain; exponent = _exponent; }
		void setProjectionPPI(span range, float aspectRatio);


		void setStrengthGradient(float sa, float sb, osg::Vec3f ca, osg::Vec3f cb, float gamma = 1.0f);
		void setStrengthGradient(osg::Vec3f full, float gamma = 1.0f);
		void setStrengthStepping(float sa, float sb, osg::Vec3f color, float gamma = 1.0f);
		void setClearColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a) {	clearColor = { r,g,b,a }; }


	private:
		enum Projection { PPI };
		typedef std::array<osg::Vec3f, 256> ColorLUT;

		void setGradient(ColorLUT * lut, float sa, float sb, osg::Vec3f ca, osg::Vec3f cb, float gamma = 1.0f) const;


		struct Color
		{
			//Default to black
			unsigned char R = 0;
			unsigned char G = 0;
			unsigned char B = 0;
			unsigned char A = 0;
		};

		void accumulatePix(size_t page, float x, float y, float power);
		void accumulateRaw_PPI(size_t page, ARF::RawResult::SCPtr bfrResult, float gain, double az1_shift, double az2_shift);
		void accumulateSAR_Patch(size_t page, ARF::SARResult sarResult);
		void finalizeChunk(size_t page, int y, int chunks);
		void finalizeLine(size_t page, int y);
		void clear_internal();
		void reset_internal(size_t page);



		std::shared_ptr<ThreadPool> threadPool;

		struct page_t
		{
			std::vector<float> buffer;
			std::unique_ptr<std::mutex> mutex;
			float auto_gain = 0;
			bool reset_needed = true;
		};

		std::vector<page_t> pages;

		std::unique_ptr<render::Texture> texture_RAM;
		std::unique_ptr<render::Texture> texture_GPU;
		std::vector<Color> pixels;
		mutable std::mutex pixels_mutex;
		static constexpr const size_t no_page = ~(size_t)0;
		size_t displayed_gpu_page = no_page;
		mutable bool gpu_texture_outdated = true;

		float gain = 1;
		float exponent = 1;
		span yRange = { 0,1 };
		span xRange = { -0.5,0.5 };

		Projection projection = PPI;
		float PPI_aspectRatio = 1.0f;



		int lastSegment = 0;
		ColorLUT strengthLUT;
		Color clearColor = Color();
	};
}