#pragma once

#ifndef __StereoExpander__
#define __StereoExpander__

#include "fxobjects.h"

/**
\struct StereoExpanderParameters
\ingroup FX-Objects
\brief
Custom parameter structure for the StereoExpander object.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
struct StereoExpanderParameters
{
	StereoExpanderParameters() {}

	/** all FXObjects parameter objects require overloaded= operator so remember to add new entries if you add new variables. */
	StereoExpanderParameters& operator=(const StereoExpanderParameters& params)	// need this override for collections to work
	{
		// --- it is possible to try to make the object equal to itself
		//     e.g. thisObject = thisObject; so this code catches that
		//     trivial case and just returns this object
		if (this == &params)
			return *this;

		// --- copy from params (argument) INTO our variables
		right_dB = params.right_dB;
		left_dB = params.left_dB;
		hf_gain = params.hf_gain;
		out_dB = params.out_dB;


		// --- MUST be last
		return *this;
	}

	// --- individual parameters
	double right_dB = -10;
	double left_dB = -10;
	double hf_gain = 15;
	double out_dB = 0;
};


/**
\class StereoExpander
\ingroup FX-Objects
\brief
The StereoExpander object implements ....

Audio I/O:
- Processes mono input to mono output.
- *** Optionally, process frame *** Modify this according to your object functionality

Control I/F:
- Use StereoExpanderParameters structure to get/set object params.

\author <Your Name> <http://www.yourwebsite.com>
\remark <Put any remarks or notes here>
\version Revision : 1.0
\date Date : 2019 / 01 / 31
*/
class StereoExpander : public IAudioSignalProcessor
{
public:
	StereoExpander(void) {}	/* C-TOR */
	~StereoExpander(void) {}	/* D-TOR */

public:
	/** reset members to initialized state */
	virtual bool reset(double _sampleRate)
	{
		// --- store the sample rate
		sampleRate = _sampleRate;

		// --- do any other per-audio-run inits here

		AudioFilterParameters params = highPassRL.getParameters();
		params.algorithm = filterAlgorithm::kHPF1;
		params.fc = 40;
		highPassRL.setParameters(params);
		highPassRL.reset(_sampleRate);

		AudioFilterParameters APFparams = allPassFilter.getParameters();
		params.algorithm = filterAlgorithm::kAPF2;
		params.fc = 500;
		allPassFilter.setParameters(APFparams);
		allPassFilter.reset(_sampleRate);

		AudioFilterParameters LFparams = paraLF.getParameters();
		LFparams.algorithm = filterAlgorithm::kCQParaEQ;
		LFparams.fc = 125;
		LFparams.boostCut_dB = 8.0;
		LFparams.Q = 1.0;
		paraLF.setParameters(LFparams);
		paraLF.reset(_sampleRate);

		AudioFilterParameters HFparams = paraHF.getParameters();
		HFparams.algorithm = filterAlgorithm::kCQParaEQ;
		HFparams.fc = 10000;
		HFparams.Q = 0.5;
		paraHF.setParameters(HFparams);
		paraHF.reset(_sampleRate);

		return true;
	}

	/** process MONO input */
	/**
	\param xn input
	\return the processed sample
	*/
	virtual double processAudioSample(double xn)
	{
		// --- the output variable
		double yn = 0.0;

		// --- do your DSP magic here to create yn

		// --- done
		return yn;
	}

	/** query to see if this object can process frames */
	virtual bool canProcessAudioFrame() { return true; } // <-- change this!

	/** process audio frame: implement this function if you answer "true" to above query */
	virtual bool processAudioFrame(const float* inputFrame,	/* ptr to one frame of data: pInputFrame[0] = left, pInputFrame[1] = right, etc...*/
					     float* outputFrame,
					     uint32_t inputChannels,
					     uint32_t outputChannels)
	{
		// --- do nothing
		if (inputChannels != 2 && outputChannels != 2)
		{
			// --- copy input to output (pass through, no FX)
			// --- mono
			outputFrame[0] = inputFrame[0];
			// --- mono to stereo
			if (outputChannels == 2)
				outputFrame[1] = inputFrame[0];
			return true; // handled
		}
		double xL = inputFrame[0];
		double xR = inputFrame[1];
		double xLhp = 1.0;
		double xRhp = 1.0;
		xLhp = highPassRL.processAudioSample(xL);
		xRhp = highPassRL.processAudioSample(xR);
		double sumRL = (xLhp + xRhp) * right_cooked;
		double subLR = (xLhp - xRhp) * left_cooked;
		double subAP = allPassFilter.processAudioSample(subLR);
		double subLF = paraLF.processAudioSample(subLR);
		double subHF = paraHF.processAudioSample(subLR);
		double sumAP = allPassFilter.processAudioSample(sumRL);
		double sumLF = paraLF.processAudioSample(sumRL);
		double sumHF = paraHF.processAudioSample(sumRL);
		double filterSum = 0;
		filterSum = subLF + subAP + subHF;

		double outL = xL + sumRL + subAP + filterSum;
		double outR = xR + sumRL - filterSum;

		outputFrame[0] = outL * out_Cooked; // processFrameInfo.audioInputFrame[0];
		outputFrame[1] = outR * out_Cooked; // processFrameInfo.audioInputFrame[1];


		return false; // NOT handled
	}


	/** get parameters: note use of custom structure for passing param data */
	/**
	\return StereoExpanderParameters custom data structure
	*/
	StereoExpanderParameters getParameters()
	{
		return parameters;
	}

	/** set parameters: note use of custom structure for passing param data */
	/**
	\param StereoExpanderParameters custom data structure
	*/
	void setParameters(const StereoExpanderParameters& _params)
	{
		// --- copy them; note you may choose to ignore certain items
		//     and copy the variables one at a time, or you may test
		//     to see if cook-able variables have changed; if not, then
		//     do not re-cook them as it just wastes CPU
		parameters = _params;

		// --- cook parameters here
		right_cooked = pow(10.0, parameters.right_dB / 20);
		left_cooked = pow(10.0, parameters.left_dB / 20);
		out_Cooked = pow(10.0, parameters.out_dB / 20);
	}

private:
	StereoExpanderParameters parameters; ///< object parameters

	// --- local variables used by this object
	double sampleRate = 0.0;	///< sample rate
	double right_cooked = 1.0;
	double left_cooked = 1.0;
	double out_Cooked = 1.0;
	AudioFilter highPassRL;
	AudioFilter allPassFilter;
	AudioFilter paraLF;
	AudioFilter paraHF;

};

#endif