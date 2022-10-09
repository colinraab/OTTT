#ifndef AuxPort_AudioEffect_H
#define AuxPort_AudioEffect_H
#pragma once
/*
*			WrappedFX
			"I tried to make it easier" - inpinseptipin

			BSD 3-Clause License

			Copyright (c) 2022, Satyarth Arora, Graduate Teaching Assistant, University of Miami
			All rights reserved.

			Redistribution and use in source and binary forms, with or without
			modification, are permitted provided that the following conditions are met:

			1. Redistributions of source code must retain the above copyright notice, this
			   list of conditions and the following disclaimer.

			2. Redistributions in binary form must reproduce the above copyright notice,
			   this list of conditions and the following disclaimer in the documentation
			   and/or other materials provided with the distribution.

			3. Neither the name of the copyright holder nor the names of its
			   contributors may be used to endorse or promote products derived from
			   this software without specific prior written permission.

			THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
			AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
			IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
			DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
			FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
			DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
			SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
			CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
			OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
			OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <vector>
#include <math.h>
#include "plugincore.h"
#include "fxobjects.h"
#include "CustomClasses/UpwardCompressor.h"
namespace AuxPort
{
/*===================================================================================*/
/*
	[Class] Abstraction over AudioFilter class in FXObjects [Don't Mess with it]
*/
	template<class sample, class knob>
	class Filter
	{
	public:
		Filter() = default;
		Filter(const Filter& filter) = default;
		void setFilterType(const filterAlgorithm& type)
		{
			filterParameters.algorithm = type;
		}
		void setParameters(const knob& centerFrequency, const knob& QFactor)
		{
			filterParameters.Q = QFactor;
			filterParameters.fc = centerFrequency;
			filterParameters.boostCut_dB = 1;
			filter.setParameters(filterParameters);
		}
		sample process(const sample& frame)
		{
			return filter.processAudioSample(frame);
		}
		~Filter() = default;
	private:
		AudioFilter filter;
		AudioFilterParameters filterParameters;

	};

	/*===================================================================================*/
	/*
		[Class] A Custom Audio Delay class with Linear Interpolation
	*/
	class Delay
	{
	public:
		Delay() = default;
		~Delay() = default;
		Delay(const Delay& delay) = default;
		void setMaxDelay(const uint32_t& sampleRate, const uint32_t& maxDelay)
		{
			buffer.resize(sampleRate * maxDelay);
			this->sampleRate = sampleRate;
			std::fill(buffer.begin(), buffer.end(), 0);
			feedbackSample = 0;
			writeIndex = 0;
			delayHead = 0;
			delaySmooth = 0;
			delay = 0;
			dry = 0;
			wet = 0;
			feedback = 0;
			delayInSamples = 0;

		}
		void setParameters(const float& delay, const float& drywet, const float& feedback)
		{
			if (this->dry != drywet)
				this->dry = drywet;
			if (this->wet != 1 - drywet)
				this->wet = 1 - drywet;
			if (this->feedback != feedback / 100)
				this->feedback = feedback / 100;
			if (this->delay != delay)
			{
				this->delay = this->delaySmooth;
				this->delaySmooth = delay;
			}
		}
		float processAudioSample(const float& sample)
		{
			this->delayInSamples = sampleRate * this->delaySmooth;
			this->delaySmooth = this->delaySmooth - 0.0001 * (this->delaySmooth - this->delay);
			buffer[writeIndex++] = sample + feedbackSample;
			writeIndex %= buffer.size();
			delayHead = writeIndex - delayInSamples;
			if (delayHead < 0)
				delayHead += buffer.size();
			float polated = doPolation();
			feedbackSample = polated * this->feedback;
			return this->dry * sample + this->wet * polated;
		}
	private:
		float doPolation()
		{
			int32_t readIndex_0 = int32_t(delayHead);
			int32_t readIndex_1 = readIndex_0 + 1;
			if (readIndex_0 >= buffer.size())
				readIndex_0 -= buffer.size();
			if (readIndex_1 >= buffer.size())
				readIndex_1 -= buffer.size();
			return (buffer[readIndex_0] + buffer[readIndex_1]) / 2;
		}
		std::vector<float> buffer;
		int32_t writeIndex;
		float delayHead;
		float delayInSamples;
		uint32_t sampleRate;
		float dry;
		float wet;
		float feedback;
		float feedbackSample;
		float delay;
		float delaySmooth = { 0 };
	};
/*===================================================================================*/
/*
	[Struct] Simple Struct for handling two channel Frames [DON'T MESS WITH IT]
*/
	template<class sample>
	struct Frame
	{
		sample left;
		sample right;
	};

	template<class sample, class knob>
	class Effect
	{
	public:
/*===================================================================================*/
/*
	[Constructor] Safely Allocates the memory
*/
		Effect<sample, knob>() 
		{
			// initialize objects here!
			// filters
			lpf1[0].setFilterType(filterAlgorithm::kLWRLPF2);
			lpf1[1].setFilterType(filterAlgorithm::kLWRLPF2);
			lpf2[0].setFilterType(filterAlgorithm::kLWRLPF2);
			lpf2[1].setFilterType(filterAlgorithm::kLWRLPF2);
			lpf3[0].setFilterType(filterAlgorithm::kLWRLPF2);
			lpf3[1].setFilterType(filterAlgorithm::kLWRLPF2);
			hpf1[0].setFilterType(filterAlgorithm::kLWRHPF2);
			hpf1[1].setFilterType(filterAlgorithm::kLWRHPF2);
			hpf2[0].setFilterType(filterAlgorithm::kLWRHPF2);
			hpf2[1].setFilterType(filterAlgorithm::kLWRHPF2);
			hpf3[0].setFilterType(filterAlgorithm::kLWRHPF2);
			hpf3[1].setFilterType(filterAlgorithm::kLWRHPF2);

			lpf1[0].setParameters(80, 0.707);
			lpf1[1].setParameters(80, 0.707);
			lpf2[0].setParameters(2000, 0.707);
			lpf2[1].setParameters(2000, 0.707);
			lpf3[0].setParameters(10000, 0.707);
			lpf3[1].setParameters(10000, 0.707);
			hpf1[0].setParameters(80, 0.707);
			hpf1[1].setParameters(80, 0.707);
			hpf2[0].setParameters(2000, 0.707);
			hpf2[1].setParameters(2000, 0.707);
			hpf3[0].setParameters(10000, 0.707);
			hpf3[1].setParameters(10000, 0.707);

			// compressors
			DynamicsProcessorParameters params;
			params.kneeWidth_dB = 6;
			params.softKnee = true;
			params.calculation = dynamicsProcessorType::kCompressor;

			params.attackTime_mSec = 48;
			params.releaseTime_mSec = 282;
			params.ratio = 50;
			params.threshold_dB = -34;
			params.hardLimitGate = false;
			params.outputGain_dB = 10;
			compressorDownLows[0].setParameters(params);
			compressorDownLows[1].setParameters(params);

			params.attackTime_mSec = 22;
			params.releaseTime_mSec = 282;
			params.ratio = 50;
			params.threshold_dB = -36;
			params.hardLimitGate = true;
			params.outputGain_dB = 3;
			compressorDownLowMids[0].setParameters(params);
			compressorDownLowMids[1].setParameters(params);

			params.attackTime_mSec = 34;
			params.releaseTime_mSec = 200;
			params.ratio = 40;
			params.threshold_dB = -34;
			params.hardLimitGate = false;
			params.outputGain_dB = 10;
			compressorDownHighMids[0].setParameters(params);
			compressorDownHighMids[1].setParameters(params);

			params.attackTime_mSec = 13;
			params.releaseTime_mSec = 132;
			params.ratio = 50;
			params.threshold_dB = -32;
			params.hardLimitGate = true;
			params.outputGain_dB = 14;
			compressorDownHighs[0].setParameters(params);
			compressorDownHighs[1].setParameters(params);

			// upwards compressors
			compressorUpLows[0].setParameters(4.0, -41.0, 48.0, 282.0, 0);
			compressorUpLows[1].setParameters(4.0, -41.0, 48.0, 282.0, 0);
			compressorUpLowMids[0].setParameters(4.0, -41, 22.0, 282.0, 0);
			compressorUpLowMids[1].setParameters(4.0, -41, 22.0, 282.0, 0);
			compressorUpHighMids[0].setParameters(4.0, -41.0, 34.0, 200.0, 0);
			compressorUpHighMids[1].setParameters(4.0, -41.0, 34.0, 200.0, 0);
			compressorUpHighs[0].setParameters(4.0, -41.0, 13.0, 132.0, 0);
			compressorUpHighs[1].setParameters(4.0, -41.0, 13.0, 132.0, 0);

			// distortion
			ClassATubePreParameters tubeParams;
			tubeParams.lowShelf_fc = 150;
			tubeParams.highShelf_fc = 1000;
			tubeParams.highShelfBoostCut_dB = -60;
			tubePre[0].setParameters(tubeParams);
			tubePre[1].setParameters(tubeParams);

			TriodeClassAParameters triParams;
			triParams.waveshaper = distortionModel::kFuzzAsym;
			triParams.asymmetry = 0.5;
			triParams.enableHPF = true;
			triParams.hpf_Fc = 2000;
			exciter[0].setParameters(triParams);
			exciter[1].setParameters(triParams);
		}
/*===================================================================================*/
/*
	[Destructor] Safely Deallocates the memory
*/
		~Effect<sample, knob>() = default;
/*===================================================================================*/
/*
	[Copy Constructor] Safely Copies memory from one Effect Object to another
*/
		Effect<sample, knob>(const Effect<sample, knob>& kernel) = default;
/*===================================================================================*/
/*
	[Function] This function pushes the parameters to the WrappedFX class (DONT MESS WITH IT)
*/
		void push(void* parameterAddress, const boundVariableType& dataType,int controlNumber)
		{
			_controls.push_back({parameterAddress,dataType,controlNumber});
		}
/*===================================================================================*/
/*
	[Function] Use this to update your FX objects on a per sample basis [Heavy CPU Usage]
*/
		void updateParametersBySample()
		{
			
		}
/*===================================================================================*/
/*
	[Function] Use this to update your FX objects on a per buffer basis [Low CPU Usage]
*/
		void updateParametersByBuffer()
		{
			// use this
			DynamicsProcessorParameters params = compressorDownLows[0].getParameters();
			if (params.outputGain_dB != (10.0 + getParameter(controlID::lowsGain))) {
				params.outputGain_dB = 10.0 + getParameter(controlID::lowsGain);
				compressorDownLows[0].setParameters(params);
				compressorDownLows[1].setParameters(params);
			}

			params = compressorDownLowMids[0].getParameters();
			if (params.outputGain_dB != (3.0 + getParameter(controlID::lowMidsGain))) {
				params.outputGain_dB = 3.0 + getParameter(controlID::lowMidsGain);
				compressorDownLowMids[0].setParameters(params);
				compressorDownLowMids[1].setParameters(params);
			}

			params = compressorDownHighMids[0].getParameters();
			if (params.outputGain_dB != (10.0 + getParameter(controlID::highMidsGain))) {
				params.outputGain_dB = 10.0 + getParameter(controlID::highMidsGain);
				compressorDownHighMids[0].setParameters(params);
				compressorDownHighMids[1].setParameters(params);
			}

			params = compressorDownHighs[0].getParameters();
			if (params.outputGain_dB != (14.0 + getParameter(controlID::highsGain))) {
				params.outputGain_dB = 14.0 + getParameter(controlID::highsGain);
				compressorDownHighs[0].setParameters(params);
				compressorDownHighs[1].setParameters(params);
			}

			// left knob 
			ClassATubePreParameters tubeParams = tubePre[0].getParameters();
			//tubeParams.inputLevel_dB = leftMult / 10.0;
			//tubeParams.outputLevel_dB = leftMult * -1.0 / 10.0;
			tubeParams.lowShelfBoostCut_dB = (getParameter(controlID::leftMult) / 25.0) + 2;
			tubeParams.saturation = getParameter(controlID::leftMult) / 20.0;
			//tubeParams.highShelfBoostCut_dB = leftMult * -1.0 / 25.0;
			tubePre[0].setParameters(tubeParams);
			tubePre[1].setParameters(tubeParams);

			// right knob
			TriodeClassAParameters fuzzParams = exciter[0].getParameters();
			fuzzParams.saturation = getParameter(controlID::rightMult) / 20.0;
			fuzzParams.asymmetry = (getParameter(controlID::rightMult) / 200.0) + 0.5;
			exciter[0].setParameters(fuzzParams);
			exciter[1].setParameters(fuzzParams);

			// up knob
			double up = getParameter(controlID::upMult);
			compressorUpLows[0].setParameters(4.0, -41.0 + (up / 20.0), 48.0 - (up / 3.0), 282.0 - (up * 2.0), 0);
			compressorUpLows[1].setParameters(4.0, -41.0 + (up / 20.0), 48.0 - (up / 3.0), 282.0 - (up * 2.0), 0);
			compressorUpLowMids[0].setParameters(4.0, -41 + (up / 20.0), 22.0 - (up / 5.0), 282.0 - (up * 2.0), 0);
			compressorUpLowMids[1].setParameters(4.0, -41 + (up / 20.0), 22.0 - (up / 5.0), 282.0 - (up * 2.0), 0);
			compressorUpHighMids[0].setParameters(4.0, -41.0 + (up / 20.0), 34.0 - (up / 4.0), 200.0 - (up * 1.5), 0);
			compressorUpHighMids[1].setParameters(4.0, -41.0 + (up / 20.0), 34.0 + (up / 20.0), 200.0 - (up * 1.5), 0);
			compressorUpHighs[0].setParameters(4.0, -41.0 + (up / 20.0), 13.0 + (up / 10.0), 132.0 - up, 0);
			compressorUpHighs[1].setParameters(4.0, -41.0 + (up / 20.0), 13.0 + (up / 10.0), 132.0 - up, 0);

			// down knob
			double down = getParameter(controlID::downMult);
			params = compressorDownLows[0].getParameters();
			params.threshold_dB = -34.0 - (down / 20.0);
			params.attackTime_mSec = 48.0 - (down / 3.0);
			params.releaseTime_mSec = 282.0 - (down * 2.0);
			compressorDownLows[0].setParameters(params);
			compressorDownLows[1].setParameters(params);

			params = compressorDownLowMids[0].getParameters();
			params.threshold_dB = -36 - (down / 20.0);
			params.attackTime_mSec = 22 - (down / 5.0);
			params.releaseTime_mSec = 282 - (down * 2.0);
			compressorDownLowMids[0].setParameters(params);
			compressorDownLowMids[1].setParameters(params);

			params = compressorDownHighMids[0].getParameters();
			params.threshold_dB = -34 - (down / 20.0);
			params.attackTime_mSec = 34 - (down / 4.0);
			params.releaseTime_mSec = 200 - (down * 1.5);
			compressorDownHighMids[0].setParameters(params);
			compressorDownHighMids[1].setParameters(params);

			params = compressorDownHighs[0].getParameters();
			params.threshold_dB = -32 - (down / 20.0);
			params.attackTime_mSec = 13 - (down / 10.0);
			params.releaseTime_mSec = 132.0 - down;
			compressorDownHighs[0].setParameters(params);
			compressorDownHighs[1].setParameters(params);

			

			// reset knobs
			if (getParameter(controlID::resetButton)) {
				
			}
			
		}

/*===================================================================================*/
/*
	[Function] Use this to Reset your FX objects when you switch songs or load the plugin
*/
		void reset(const uint32_t& sampleRate)
		{
			/*
				Reset FX objects
			*/
			compressorDownLows[0].reset(sampleRate);
			compressorDownLows[1].reset(sampleRate);
			compressorDownLowMids[0].reset(sampleRate);
			compressorDownLowMids[1].reset(sampleRate);
			compressorDownHighMids[0].reset(sampleRate);
			compressorDownHighMids[1].reset(sampleRate);
			compressorDownHighs[0].reset(sampleRate);
			compressorDownHighs[1].reset(sampleRate);

			finalLimiter[0].reset(sampleRate);
			finalLimiter[1].reset(sampleRate);

			tubePre[0].reset(sampleRate);
			tubePre[1].reset(sampleRate);
			exciter[0].reset(sampleRate);
			exciter[1].reset(sampleRate);

			compressorUpLows[0].reset(sampleRate);
			compressorUpLows[1].reset(sampleRate);
			compressorUpLowMids[0].reset(sampleRate);
			compressorUpLowMids[1].reset(sampleRate);
			compressorUpHighMids[0].reset(sampleRate);
			compressorUpHighMids[1].reset(sampleRate);
			compressorUpHighs[0].reset(sampleRate);
			compressorUpHighs[1].reset(sampleRate);

			this->sampleRate = sampleRate;
		}
/*===================================================================================*/
/*
	[Function] Implement your DSP Logic here
*/
		void run(Frame<sample>& frame)
		{	
			/*===================================================================================*/
			/*
				Making a copy of the input Frame (Dont Mess with it)
			*/
			/*===================================================================================*/

			sample leftChannel = frame.left;
			sample rightChannel = frame.right;
			/*===================================================================================*/
			/*===================================================================================*/
			/*
				Write DSP Algorithm here
			*/
			/*===================================================================================*/
			/*
				Start
			*/

			// knob assignment
			knob dryWet = getParameter(controlID::dryWet);
			knob lowsGain = getParameter(controlID::lowsGain);
			knob lowMidsGain = getParameter(controlID::lowMidsGain);
			knob highMidsGain = getParameter(controlID::highMidsGain);
			knob highsGain = getParameter(controlID::highsGain);
			knob upMult = getParameter(controlID::upMult);
			knob downMult = getParameter(controlID::downMult);
			knob leftMult = getParameter(controlID::leftMult);
			knob rightMult = getParameter(controlID::rightMult);
			knob bypass = getParameter(controlID::bypass);
			knob inputGain = getParameter(controlID::m_inputGain);

			// filter into 4 bands
			sample filteredLowsL = lpf1[0].process(leftChannel);
			sample filteredLowsR = lpf1[1].process(rightChannel);

			sample LML = hpf1[0].process(leftChannel);
			sample LMR = hpf1[1].process(rightChannel);
			sample filteredLowMidsL = -lpf2[0].process(LML);
			sample filteredLowMidsR = -lpf2[1].process(LMR);

			sample HML = hpf2[0].process(leftChannel);
			sample HMR = hpf2[1].process(rightChannel);
			sample filteredHighMidsL = lpf3[0].process(HML);
			sample filteredHighMidsR = lpf3[1].process(HMR);

			sample filteredHighsL = -hpf3[0].process(leftChannel);
			sample filteredHighsR = -hpf3[1].process(rightChannel);

			//apply input gain
			inputGain = pow(10.0, inputGain / 20.0);
			filteredLowsL *= inputGain;
			filteredLowsR *= inputGain;
			filteredLowMidsL *= inputGain;
			filteredLowMidsR *= inputGain;
			filteredHighMidsL *= inputGain;
			filteredHighMidsR *= inputGain;
			filteredHighsL *= inputGain;
			filteredHighsR *= inputGain;

			// upward compression
			sample ynLowsExpandL = compressorUpLows[0].processAudioSample(filteredLowsL);
			sample ynLowsExpandR = compressorUpLows[1].processAudioSample(filteredLowsR);

			sample ynLowMidsExpandL = compressorUpLowMids[0].processAudioSample(filteredLowMidsL);
			sample ynLowMidsExpandR = compressorUpLowMids[1].processAudioSample(filteredLowMidsR);

			sample ynHighMidsExpandL = compressorUpHighMids[0].processAudioSample(filteredHighMidsL);
			sample ynHighMidsExpandR = compressorUpHighMids[1].processAudioSample(filteredHighMidsR);

			sample ynHighsExpandL = compressorUpHighs[0].processAudioSample(filteredHighsL);
			sample ynHighsExpandR = compressorUpHighs[1].processAudioSample(filteredHighsR);

			// downward compression
			sample ynLowsCompressedL = compressorDownLows[0].processAudioSample(ynLowsExpandL);
			sample ynLowsCompressedR = compressorDownLows[1].processAudioSample(ynLowsExpandR);

			sample ynLowMidsCompressedL = compressorDownLowMids[0].processAudioSample(ynLowMidsExpandL);
			sample ynLowMidsCompressedR = compressorDownLowMids[1].processAudioSample(ynLowMidsExpandR);

			sample ynHighMidsCompressedL = compressorDownHighMids[0].processAudioSample(ynHighMidsExpandL);
			sample ynHighMidsCompressedR = compressorDownHighMids[1].processAudioSample(ynHighMidsExpandR);

			sample ynHighsCompressedL = compressorDownHighs[0].processAudioSample(ynHighsExpandL);
			sample ynHighsCompressedR = compressorDownHighs[1].processAudioSample(ynHighsExpandR);

			// saturate lows and highs
			sample ynLowsSaturatedL = tubePre[0].processAudioSample(ynLowsCompressedL);
			sample ynLowsSaturatedR = tubePre[1].processAudioSample(ynLowsCompressedR);

			sample ynHighsFuzzL = exciter[0].processAudioSample(ynHighsCompressedL);
			sample ynHighsFuzzR = exciter[1].processAudioSample(ynHighsCompressedR);

			//sum bands back together
			sample ynLowsL = ynLowsSaturatedL * (leftMult / 100.0) + ynLowsCompressedL * (1.0 - (leftMult / 100.0));
			sample ynLowsR = ynLowsSaturatedR * (leftMult / 100.0) + ynLowsCompressedR * (1.0 - (leftMult / 100.0));
			sample ynHighsL = ynHighsFuzzL * (rightMult / 100.0) + ynHighsCompressedL * (1.0 - (rightMult / 100.0));
			sample ynHighsR = ynHighsFuzzR * (rightMult / 100.0) + ynHighsCompressedR * (1.0 - (rightMult / 100.0));

			sample ynL = ynLowsL + ynLowMidsCompressedL + ynHighMidsCompressedL + ynHighsL;
			sample ynR = ynLowsR + ynLowMidsCompressedR + ynHighMidsCompressedR + ynHighsR;

			//limiter
			ynL = finalLimiter[0].processAudioSample(ynL);
			ynR = finalLimiter[1].processAudioSample(ynR);

			//output
			ynL = ynL * (dryWet / 100.0) + leftChannel * (1.0 - (dryWet / 100.0));
			ynR = ynR * (dryWet / 100.0) + rightChannel * (1.0 - (dryWet / 100.0));

			if (!bypass) {
				ynL = leftChannel;
				ynR = rightChannel;
			}

			// send to meters
			float grLowsL = 0.0;
			float grLowsR = 0.0;
			float upLowsL = 0.0;
			float upLowsR = 0.0;
			float grLowMidsL = 0.0;
			float grLowMidsR = 0.0;
			float upLowMidsL = 0.0;
			float upLowMidsR = 0.0;
			float grHighMidsL = 0.0;
			float grHighMidsR = 0.0;
			float upHighMidsL = 0.0;
			float upHighMidsR = 0.0;
			float grHighsL = 0.0;
			float grHighsR = 0.0;
			float upHighsL = 0.0;
			float upHighsR = 0.0;

			DynamicsProcessorParameters params = compressorDownLows[0].getParameters();
			grLowsL = params.gainReduction;
			params = compressorDownLows[1].getParameters();
			grLowsR = params.gainReduction;
			setParameter(1.0 - (0.5 * grLowsL + 0.5 * grLowsR), controlID::mLowsDown);
			
			upLowsL = compressorUpLows[0].getGR();
			upLowsR = compressorUpLows[1].getGR();
			if (upLowsL > 10) upLowsL = 10;
			if (upLowsR > 10) upLowsR = 10;
			setParameter(1.0 - (0.5 * 1.0 / upLowsL + 0.5 * 1.0 / upLowsR), controlID::mLowsUp);
			
			params = compressorDownLowMids[0].getParameters();
			grLowMidsL = params.gainReduction;
			params = compressorDownLowMids[1].getParameters();
			grLowMidsR = params.gainReduction;
			setParameter(1.0 - (0.5 * grLowMidsL + 0.5 * grLowMidsR), controlID::mLowMidsDown);
			
			upLowMidsL = compressorUpLowMids[0].getGR();
			upLowMidsR = compressorUpLowMids[1].getGR();
			if (upLowMidsL > 10) upLowMidsL = 10;
			if (upLowMidsR > 10) upLowMidsR = 10;
			setParameter(1.0 - (0.5 * 1.0 / upLowMidsL + 0.5 * 1.0 / upLowMidsR), controlID::mLowMidsUp);

			params = compressorDownHighMids[0].getParameters();
			grHighMidsL = params.gainReduction;
			params = compressorDownHighMids[1].getParameters();
			grHighMidsR = params.gainReduction;
			setParameter(1.0 - (0.5 * grHighMidsL + 0.5 * grHighMidsR), controlID::mHighMidsDown);
			
			upHighMidsL = compressorUpHighMids[0].getGR();
			upHighMidsR = compressorUpHighMids[1].getGR();
			if (upHighMidsL > 10) upHighMidsL = 10;
			if (upHighMidsR > 10) upHighMidsR = 10;
			setParameter(1.0 - (0.5 * 1.0 / upHighMidsL + 0.5 * 1.0 / upHighMidsR), controlID::mHighMidsUp);

			params = compressorDownHighs[0].getParameters();
			grHighsL = params.gainReduction;
			params = compressorDownHighs[1].getParameters();
			grHighsR = params.gainReduction;
			setParameter(1.0 - (0.5 * grHighsL + 0.5 * grHighsR), controlID::mHighsDown);
			
			upHighsL = compressorUpHighs[0].getGR();
			upHighsR = compressorUpHighs[1].getGR();
			if (upHighsL > 10) upHighsL = 10;
			if (upHighsR > 10) upHighsR = 10;
			setParameter(1.0 - (0.5 * 1.0 / upHighsL + 0.5 * 1.0 / upHighsR), controlID::mHighsUp);

			/*
				End
			*/
			/*===================================================================================*/
			/*===================================================================================*/
			/*
				Save your processed Samples back to the Frame (Dont Mess with it)
			*/
			/*===================================================================================*/

			frame.left = ynL;
			frame.right = ynR;
		}

		

/*===================================================================================*/
	private:
		Filter<float, float> lpf1[2];
		Filter<float, float> lpf2[2];
		Filter<float, float> lpf3[2];
		Filter<float, float> hpf1[2];
		Filter<float, float> hpf2[2];
		Filter<float, float> hpf3[2];

		DynamicsProcessor compressorDownLows[2];
		DynamicsProcessor compressorDownLowMids[2];
		DynamicsProcessor compressorDownHighMids[2];
		DynamicsProcessor compressorDownHighs[2];

		DynamicsProcessor finalLimiter[2];

		ClassATubePre tubePre[2];
		TriodeClassA exciter[2];

		//std::vector<Colin::UpwardCompressor<float, float>> compressorUpLows[2];
		//std::vector<Colin::UpwardCompressor<float, float>> compressorUpLowMids[2];
		//std::vector<Colin::UpwardCompressor<float, float>> compressorUpHighMids[2];
		//std::vector<Colin::UpwardCompressor<float, float>> compressorUpHighs[2];

		Colin::UpwardCompressor<float, float> compressorUpLows[2];
		Colin::UpwardCompressor<float, float> compressorUpLowMids[2];
		Colin::UpwardCompressor<float, float> compressorUpHighMids[2];
		Colin::UpwardCompressor<float, float> compressorUpHighs[2];

/*===================================================================================*/
/*
	[Function] Gets the Control from our nice dandy vector of pointers (DONT MESS WITH IT)
*/
		knob getParameter(const int& i)
		{
			Parameters* para;
			for (size_t j = 0; j < _controls.size(); j++)
			{
				para = &_controls[j];
				if (para->controlNumber == i)
				{
					if (para->_dataType == boundVariableType::kFloat)
						return *static_cast<float*>(para->_parameterAddress);
					if (para->_dataType == boundVariableType::kDouble)
						return *static_cast<double*>(para->_parameterAddress);
					if (para->_dataType == boundVariableType::kInt)
						return *static_cast<int*>(para->_parameterAddress);
					if (para->_dataType == boundVariableType::kUInt)
						return *static_cast<uint32_t*>(para->_parameterAddress);
				}
			}
		}
/*===================================================================================*/
/*
	[Function] Use this function to Update your meters
*/
		void setParameter(const double& newValue,const int& i)
		{
			Parameters* para;
			for (size_t j = 0; j < _controls.size(); j++)
			{
				para = &_controls[j];
				if (para->controlNumber == i)
				{
					if (para->_dataType == boundVariableType::kFloat)
						*static_cast<float*>(para->_parameterAddress) = static_cast<float>(newValue);
					else if (para->_dataType == boundVariableType::kDouble)
						*static_cast<double*>(para->_parameterAddress) = newValue;
					else if (para->_dataType == boundVariableType::kInt)
						*static_cast<int*>(para->_parameterAddress) = static_cast<int>(newValue);
					else if (para->_dataType == boundVariableType::kUInt)
						*static_cast<uint32_t*>(para->_parameterAddress) = static_cast<uint32_t>(newValue);
					break;
				}

			}		
		}

		struct Parameters
		{
			void* _parameterAddress;
			boundVariableType _dataType;
			int controlNumber;
		};
		std::vector<Parameters> _controls;
		uint32_t sampleRate;
		
	};
}
#endif