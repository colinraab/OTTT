#ifndef COLIN_COMPRESSOR_H
#define COLIN_COMPRESSOR_H

#include "WrappedFX.h"

/*
*/

namespace Colin
{
	template <class sample, class knob>
	class UpwardCompressor 
	{
	public:
		UpwardCompressor() = default;
		~UpwardCompressor() = default;
		UpwardCompressor(const UpwardCompressor& uc) = default;

		virtual bool reset(double _sampleRate)
		{
			sidechainInputSample = 0.0;
			detector.reset(_sampleRate);
			AudioDetectorParameters detectorParams = detector.getParameters();
			detectorParams.clampToUnityMax = false;
			detectorParams.detect_dB = true;
			detector.setParameters(detectorParams);
			return true;
		}

		void setParameters(const float& xratio, const float& threshold, const float& attack, const float& release, const float& outputGain)
		{
			ratio = xratio;
			threshold_dB = threshold;
			attackTime_mSec = attack;
			releaseTime_mSec = release;
			outputGain_dB = outputGain;

			AudioDetectorParameters detectorParams = detector.getParameters();
			detectorParams.attackTime_mSec = attack;
			detectorParams.releaseTime_mSec = release;
			detectorParams.detect_dB = true;
			detector.setParameters(detectorParams);
		}

		virtual double getThreshold()
		{
			return threshold_dB;
		}

		virtual double getGR()
		{
			return gainReduction;
		}

		virtual double processAudioSample(double xn)
		{
			// --- detect input
			double detect_dB = 0.0;

			// --- if using the sidechain, process the aux input
			if (enableSidechain)
				detect_dB = detector.processAudioSample(sidechainInputSample);
			else
				detect_dB = detector.processAudioSample(xn);

			// --- compute gain
			double gr = computeGain(detect_dB);

			// --- makeup gain
			double makeupGain = pow(10.0, outputGain_dB / 20.0);

			// --- do DCA + makeup gain
			return xn * gr * makeupGain;
		}

	private:
		AudioDetector detector; ///< the sidechain audio detector

		double ratio = 50.0;				///< processor I/O gain ratio
		double threshold_dB = -10.0;		///< threshold in dB
		//float kneeWidth_dB = 10.0;			///< knee width in dB for soft-knee operation
		bool hardLimitGate = false;			///< threshold in dB
		//bool softKnee = true;				///< soft knee flag
		bool enableSidechain = false;		///< enable external sidechain input to object
		double attackTime_mSec = 0.0;		///< attack mSec
		double releaseTime_mSec = 0.0;		///< release mSec
		double outputGain_dB = 0.0;			///< make up gain

		// --- outbound values, for owner to use gain-reduction metering
		double gainReduction = 1.0;			///< output value for gain reduction that occurred
		double gainReduction_dB = 0.0;		///< output value for gain reduction that occurred in dB

		double sidechainInputSample = 0.0; ///< storage for sidechain sample

		inline double computeGain(double detect_dB)
		{
			double output_dB = 0.0;
			// --- hard knee
			// --- NOTE: soft knee is not technically possible with a gate because there
			//           is no "left side" of the knee
			//if (!parameters.softKnee || parameters.hardLimitGate)
			//{
				// --- above threshold, unity gain
			if (detect_dB >= threshold_dB)
				output_dB = detect_dB;
			else
			{
				if (hardLimitGate) // --- inverse limiter (dB)
					output_dB = threshold_dB;
				else
					//output_dB = detect_dB + (parameters.threshold_dB - detect_dB) / parameters.ratio;
					//output_dB = parameters.threshold_dB + (detect_dB - parameters.threshold_dB) / parameters.ratio;
					output_dB = threshold_dB - (threshold_dB - detect_dB) / ratio;
			}
			gainReduction_dB = output_dB - detect_dB;
			gainReduction = pow(10.0, (gainReduction_dB) / 20.0);

			// --- the current gain coefficient value
			return gainReduction;
		}
	};
}

#endif