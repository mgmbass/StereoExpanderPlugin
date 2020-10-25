// -----------------------------------------------------------------------------
//    ASPiK Plugin Kernel File:  plugincore.cpp
//
/**
    \file   plugincore.cpp
    \author Will Pirkle
    \date   17-September-2018
    \brief  Implementation file for PluginCore object
    		- http://www.aspikplugins.com
    		- http://www.willpirkle.com
*/
// -----------------------------------------------------------------------------
#include "plugincore.h"
#include "plugindescription.h"


/**
\brief PluginCore constructor is launching pad for object initialization

Operations:
- initialize the plugin description (strings, codes, numbers, see initPluginDescriptors())
- setup the plugin's audio I/O channel support
- create the PluginParameter objects that represent the plugin parameters (see FX book if needed)
- create the presets
*/
PluginCore::PluginCore()
{
    // --- describe the plugin; call the helper to init the static parts you setup in plugindescription.h
    initPluginDescriptors();

    // --- default I/O combinations
	// --- for FX plugins
	if (getPluginType() == kFXPlugin)
	{
		addSupportedIOCombination({ kCFMono, kCFMono });
		addSupportedIOCombination({ kCFMono, kCFStereo });
		addSupportedIOCombination({ kCFStereo, kCFStereo });
	}
	else // --- synth plugins have no input, only output
	{
		addSupportedIOCombination({ kCFNone, kCFMono });
		addSupportedIOCombination({ kCFNone, kCFStereo });
	}

	// --- for sidechaining, we support mono and stereo inputs; auxOutputs reserved for future use
	addSupportedAuxIOCombination({ kCFMono, kCFNone });
	addSupportedAuxIOCombination({ kCFStereo, kCFNone });

	// --- create the parameters
    initPluginParameters();

    // --- create the presets
    initPluginPresets();
}

/**
\brief create all of your plugin parameters here

\return true if parameters were created, false if they already existed
*/
bool PluginCore::initPluginParameters()
{
	if (pluginParameterMap.size() > 0)
		return false;

    // --- Add your plugin parameter instantiation code bewtween these hex codes
	// **--0xDEA7--**


	// --- Declaration of Plugin Parameter Objects 
	PluginParameter* piParam = nullptr;

	// --- continuous control: Right Att
	piParam = new PluginParameter(controlID::right_dB, "Right Att", "dB", controlVariableType::kDouble, -60.000000, 12.000000, -20.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&right_dB, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: Left Att
	piParam = new PluginParameter(controlID::left_dB, "Left Att", "dB", controlVariableType::kDouble, -60.000000, 12.000000, -20.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&left_dB, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: APF Level
	piParam = new PluginParameter(controlID::att3_dB, "APF Level", "dB", controlVariableType::kDouble, -80.000000, 20.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&att3_dB, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: HPF Fc
	piParam = new PluginParameter(controlID::LFF_Fc, "HPF Fc", "Hz", controlVariableType::kFloat, 20.000000, 250.000000, 40.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&LFF_Fc, boundVariableType::kFloat);
	addPluginParameter(piParam);

	// --- continuous control: APF Fc
	piParam = new PluginParameter(controlID::APF_Fc, "APF Fc", "Hz", controlVariableType::kDouble, 20.000000, 20040.000000, 500.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&APF_Fc, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LF Fc
	piParam = new PluginParameter(controlID::LF_Fc, "LF Fc", "Hz", controlVariableType::kDouble, 50.000000, 300.000000, 125.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&LF_Fc, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LF Gain
	piParam = new PluginParameter(controlID::LF_dB, "LF Gain", "dB", controlVariableType::kDouble, 0.000000, 16.000000, 8.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&LF_dB, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: HF Gain
	piParam = new PluginParameter(controlID::hf_gain, "HF Gain", "dB", controlVariableType::kDouble, 0.000000, 20.000000, 15.400000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&hf_gain, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: HF Fc
	piParam = new PluginParameter(controlID::hf_Fc, "HF Fc", "Hz", controlVariableType::kDouble, 5000.000000, 15000.000000, 10000.000000, taper::kVoltOctaveTaper);
	piParam->setParameterSmoothing(false);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&hf_Fc, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: LF Q
	piParam = new PluginParameter(controlID::LF_Q, "LF Q", "Units", controlVariableType::kDouble, 0.500000, 5.000000, 1.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&LF_Q, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- continuous control: HF Q
	piParam = new PluginParameter(controlID::HF_Q, "HF Q", "Units", controlVariableType::kDouble, 0.500000, 5.000000, 0.500000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&HF_Q, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- discrete control: APF On
	piParam = new PluginParameter(controlID::APF_On, "APF On", "SWITCH OFF,SWITCH ON", "SWITCH ON");
	piParam->setBoundVariable(&APF_On, boundVariableType::kInt);
	piParam->setIsDiscreteSwitch(true);
	addPluginParameter(piParam);

	// --- continuous control: OutPut
	piParam = new PluginParameter(controlID::out_dB, "OutPut", "dB", controlVariableType::kDouble, -60.000000, 20.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&out_dB, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- discrete control: EQ Listen
	piParam = new PluginParameter(controlID::eq_Listen, "EQ Listen", "SWITCH OFF,SWITCH ON", "SWITCH OFF");
	piParam->setBoundVariable(&eq_Listen, boundVariableType::kInt);
	piParam->setIsDiscreteSwitch(true);
	addPluginParameter(piParam);

	// --- continuous control: Pitch Shift
	piParam = new PluginParameter(controlID::pShift, "Pitch Shift", "Units", controlVariableType::kDouble, 0.000000, 1.000000, 0.000000, taper::kLinearTaper);
	piParam->setParameterSmoothing(true);
	piParam->setSmoothingTimeMsec(100.00);
	piParam->setBoundVariable(&pShift, boundVariableType::kDouble);
	addPluginParameter(piParam);

	// --- Aux Attributes
	AuxParameterAttribute auxAttribute;

	// --- RAFX GUI attributes
	// --- controlID::right_dB
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483660);
	setParamAuxAttribute(controlID::right_dB, auxAttribute);

	// --- controlID::left_dB
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483660);
	setParamAuxAttribute(controlID::left_dB, auxAttribute);

	// --- controlID::att3_dB
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483654);
	setParamAuxAttribute(controlID::att3_dB, auxAttribute);

	// --- controlID::LFF_Fc
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483656);
	setParamAuxAttribute(controlID::LFF_Fc, auxAttribute);

	// --- controlID::APF_Fc
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483654);
	setParamAuxAttribute(controlID::APF_Fc, auxAttribute);

	// --- controlID::LF_Fc
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483652);
	setParamAuxAttribute(controlID::LF_Fc, auxAttribute);

	// --- controlID::LF_dB
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483652);
	setParamAuxAttribute(controlID::LF_dB, auxAttribute);

	// --- controlID::hf_gain
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483662);
	setParamAuxAttribute(controlID::hf_gain, auxAttribute);

	// --- controlID::hf_Fc
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483662);
	setParamAuxAttribute(controlID::hf_Fc, auxAttribute);

	// --- controlID::LF_Q
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483652);
	setParamAuxAttribute(controlID::LF_Q, auxAttribute);

	// --- controlID::HF_Q
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483662);
	setParamAuxAttribute(controlID::HF_Q, auxAttribute);

	// --- controlID::APF_On
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(1073741829);
	setParamAuxAttribute(controlID::APF_On, auxAttribute);

	// --- controlID::out_dB
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::out_dB, auxAttribute);

	// --- controlID::eq_Listen
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(1073741829);
	setParamAuxAttribute(controlID::eq_Listen, auxAttribute);

	// --- controlID::pShift
	auxAttribute.reset(auxGUIIdentifier::guiControlData);
	auxAttribute.setUintAttribute(2147483648);
	setParamAuxAttribute(controlID::pShift, auxAttribute);


	// **--0xEDA5--**
   
    // --- BONUS Parameter
    // --- SCALE_GUI_SIZE
    PluginParameter* piParamBonus = new PluginParameter(SCALE_GUI_SIZE, "Scale GUI", "tiny,small,medium,normal,large,giant", "normal");
    addPluginParameter(piParamBonus);

	// --- create the super fast access array
	initPluginParameterArray();

    return true;
}

/**
\brief initialize object for a new run of audio; called just before audio streams

Operation:
- store sample rate and bit depth on audioProcDescriptor - this information is globally available to all core functions
- reset your member objects here

\param resetInfo structure of information about current audio format

\return true if operation succeeds, false otherwise
*/
bool PluginCore::reset(ResetInfo& resetInfo)
{
    // --- save for audio processing
    audioProcDescriptor.sampleRate = resetInfo.sampleRate;
    audioProcDescriptor.bitDepth = resetInfo.bitDepth;

	AudioFilterParameters params = highPassRL.getParameters();
	params.algorithm = filterAlgorithm::kHPF1;
	highPassRL.setParameters(params);
	highPassRL.reset(resetInfo.sampleRate);

	AudioFilterParameters APFparams = allPassFilter.getParameters();
	params.algorithm = filterAlgorithm::kAPF2;
	allPassFilter.setParameters(APFparams);
	allPassFilter.reset(resetInfo.sampleRate);

	AudioFilterParameters LFparams = paraLF.getParameters();
	LFparams.algorithm = filterAlgorithm::kCQParaEQ;
	paraLF.setParameters(LFparams);
	paraLF.reset(resetInfo.sampleRate);

	AudioFilterParameters HFparams = paraHF.getParameters();
	HFparams.algorithm = filterAlgorithm::kCQParaEQ;
	paraHF.setParameters(HFparams);
	paraHF.reset(resetInfo.sampleRate);

	//for se object usage
	stereoExpander.reset(resetInfo.sampleRate);
	

    // --- other reset inits
    return PluginBase::reset(resetInfo);
}

/**
\brief one-time initialize function called after object creation and before the first reset( ) call

Operation:
- saves structure for the plugin to use; you can also load WAV files or state information here
*/
bool PluginCore::initialize(PluginInfo& pluginInfo)
{
	// --- add one-time init stuff here

	return true;
}

/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- syncInBoundVariables when preProcessAudioBuffers is called, it is *guaranteed* that all GUI control change information
  has been applied to plugin parameters; this binds parameter changes to your underlying variables
- NOTE: postUpdatePluginParameter( ) will be called for all bound variables that are acutally updated; if you need to process
  them individually, do so in that function
- use this function to bulk-transfer the bound variable data into your plugin's member object variables

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::preProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
    // --- sync internal variables to GUI parameters; you can also do this manually if you don't
    //     want to use the auto-variable-binding
    syncInBoundVariables();

    return true;
}

/**
\brief frame-processing method

Operation:
- decode the plugin type - for synth plugins, fill in the rendering code; for FX plugins, delete the if(synth) portion and add your processing code
- note that MIDI events are fired for each sample interval so that MIDI is tightly sunk with audio
- doSampleAccurateParameterUpdates will perform per-sample interval smoothing

\param processFrameInfo structure of information about *frame* processing

\return true if operation succeeds, false otherwise
*/
void PluginCore::updateParameters() {

	right_cooked = pow(10.0, right_dB / 20);
	left_cooked = pow(10.0, left_dB / 20);
	att3_cooked = pow(10.0, att3_dB / 20);
	out_Cooked = pow(10.0, out_dB / 20);
	//LF_cooked = pow(10.0, LF_dB / 20);

	AudioFilterParameters paramshp = highPassRL.getParameters();
	paramshp.fc = LFF_Fc;
	highPassRL.setParameters(paramshp);

	AudioFilterParameters APFparamsaf = allPassFilter.getParameters();
	APFparamsaf.fc = APF_Fc;
	//APFparamsaf.boostCut_dB = att3_dB;
	allPassFilter.setParameters(APFparamsaf);

	AudioFilterParameters LFparamsaf = paraLF.getParameters();
	LFparamsaf.fc = LF_Fc;
	LFparamsaf.boostCut_dB = LF_dB;
	LFparamsaf.Q = LF_Q;
	paraLF.setParameters(LFparamsaf);

	AudioFilterParameters HFparamsaf = paraHF.getParameters();
	HFparamsaf.fc = hf_Fc;
	HFparamsaf.boostCut_dB = hf_gain;
	HFparamsaf.Q = HF_Q;
	paraHF.setParameters(HFparamsaf);


	//for usage of se object
	StereoExpanderParameters params = stereoExpander.getParameters();
	params.hf_gain = hf_gain;
	params.left_dB = left_dB;
	params.right_dB = right_dB;
	params.out_dB = out_dB;
	stereoExpander.setParameters(params);


}

bool PluginCore::processAudioFrame(ProcessFrameInfo& processFrameInfo)
{
    // --- fire any MIDI events for this sample interval
    processFrameInfo.midiEventQueue->fireMidiEvents(processFrameInfo.currentFrame);

	// --- do per-frame updates; VST automation and parameter smoothing
	doSampleAccurateParameterUpdates();
	updateParameters();
    // --- decode the channelIOConfiguration and process accordingly
    //
	// --- Synth Plugin:
	// --- Synth Plugin --- remove for FX plugins
	
	

    // --- FX Plugin:
    if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFMono)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];

        return true; /// processed
    }

    // --- Mono-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFMono &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing
        processFrameInfo.audioOutputFrame[0] = processFrameInfo.audioInputFrame[0];
        processFrameInfo.audioOutputFrame[1] = processFrameInfo.audioInputFrame[0];

        return true; /// processed
    }

    // --- Stereo-In/Stereo-Out
    else if(processFrameInfo.channelIOConfig.inputChannelFormat == kCFStereo &&
       processFrameInfo.channelIOConfig.outputChannelFormat == kCFStereo)
    {
		// --- pass through code: change this with your signal processing

       /* double xL = processFrameInfo.audioInputFrame[0];
        double xR = processFrameInfo.audioInputFrame[1];
		double xLhp = 1.0;
		double xRhp = 1.0;	
		xLhp = highPassRL.processAudioSample(xL);
		xRhp = highPassRL.processAudioSample(xR);
        double sumRL = (xLhp + xRhp)*right_cooked;
        double subLR = (xLhp - xRhp)*left_cooked;
		double subAP = allPassFilter.processAudioSample(subLR);
		double subLF = paraLF.processAudioSample(subLR);
		double subHF = paraHF.processAudioSample(subLR);
		double sumAP = allPassFilter.processAudioSample(sumRL);
		double sumLF = paraLF.processAudioSample(sumRL);
		double sumHF = paraHF.processAudioSample(sumRL);

		double filterSum = 0.0;
		double filterListenSum = sumAP + sumLF + sumHF;
		subAP = subAP * att3_cooked;
		if (compareIntToEnum(APF_On, APF_OnEnum::SWITCH_ON)) {
			filterSum = subLF + subAP + subHF;
			std::cout << "APF ON" <<std::endl; 
		}
		else {
			filterSum = subLF + subHF;
			std::cout << "APF OFF" << std::endl;
		}
		//subAP = subAP * att3_cooked;
        double outL =  xL + sumRL + subAP + filterSum;
        double outR =  xR + sumRL - filterSum;

		if (compareIntToEnum(eq_Listen, eq_ListenEnum::SWITCH_ON)) {
			processFrameInfo.audioOutputFrame[0] = filterListenSum; //*out_Cooked; // processFrameInfo.audioInputFrame[0];
			processFrameInfo.audioOutputFrame[1] = filterListenSum; //*out_Cooked; // processFrameInfo.audioInputFrame[1];
		}
		else {
			processFrameInfo.audioOutputFrame[0] = outL * out_Cooked; // processFrameInfo.audioInputFrame[0];
			processFrameInfo.audioOutputFrame[1] = outR * out_Cooked; // processFrameInfo.audioInputFrame[1];
		}
		return true;*/

       return stereoExpander.processAudioFrame(processFrameInfo.audioInputFrame,
			processFrameInfo.audioOutputFrame,
			processFrameInfo.numAudioInChannels,
			processFrameInfo.numAudioOutChannels); /// processed
    }

    return false; /// NOT processed
}


/**
\brief do anything needed prior to arrival of audio buffers

Operation:
- updateOutBoundVariables sends metering data to the GUI meters

\param processInfo structure of information about *buffer* processing

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postProcessAudioBuffers(ProcessBufferInfo& processInfo)
{
	// --- update outbound variables; currently this is meter data only, but could be extended
	//     in the future
	updateOutBoundVariables();

    return true;
}

/**
\brief update the PluginParameter's value based on GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- use base class helper
    setPIParamValue(controlID, controlValue);

    // --- do any post-processing
    postUpdatePluginParameter(controlID, controlValue, paramInfo);

    return true; /// handled
}

/**
\brief update the PluginParameter's value based on *normlaized* GUI control, preset, or data smoothing (thread-safe)

Operation:
- update the parameter's value (with smoothing this initiates another smoothing process)
- call postUpdatePluginParameter to do any further processing

\param controlID the control ID value of the parameter being updated
\param normalizedValue the new control value in normalized form
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::updatePluginParameterNormalized(int32_t controlID, double normalizedValue, ParameterUpdateInfo& paramInfo)
{
	// --- use base class helper, returns actual value
	double controlValue = setPIParamValueNormalized(controlID, normalizedValue, paramInfo.applyTaper);

	// --- do any post-processing
	postUpdatePluginParameter(controlID, controlValue, paramInfo);

	return true; /// handled
}

/**
\brief perform any operations after the plugin parameter has been updated; this is one paradigm for
	   transferring control information into vital plugin variables or member objects. If you use this
	   method you can decode the control ID and then do any cooking that is needed. NOTE: do not
	   overwrite bound variables here - this is ONLY for any extra cooking that is required to convert
	   the GUI data to meaninful coefficients or other specific modifiers.

\param controlID the control ID value of the parameter being updated
\param controlValue the new control value
\param paramInfo structure of information about why this value is being udpated (e.g as a result of a preset being loaded vs. the top of a buffer process cycle)

\return true if operation succeeds, false otherwise
*/
bool PluginCore::postUpdatePluginParameter(int32_t controlID, double controlValue, ParameterUpdateInfo& paramInfo)
{
    // --- now do any post update cooking; be careful with VST Sample Accurate automation
    //     If enabled, then make sure the cooking functions are short and efficient otherwise disable it
    //     for the Parameter involved
    /*switch(controlID)
    {
        case 0:
        {
            return true;    /// handled
        }

        default:
            return false;   /// not handled
    }*/

    return false;
}

/**
\brief has nothing to do with actual variable or updated variable (binding)

CAUTION:
- DO NOT update underlying variables here - this is only for sending GUI updates or letting you
  know that a parameter was changed; it should not change the state of your plugin.

WARNING:
- THIS IS NOT THE PREFERRED WAY TO LINK OR COMBINE CONTROLS TOGETHER. THE PROPER METHOD IS
  TO USE A CUSTOM SUB-CONTROLLER THAT IS PART OF THE GUI OBJECT AND CODE.
  SEE http://www.willpirkle.com for more information

\param controlID the control ID value of the parameter being updated
\param actualValue the new control value

\return true if operation succeeds, false otherwise
*/
bool PluginCore::guiParameterChanged(int32_t controlID, double actualValue)
{
	/*
	switch (controlID)
	{
		case controlID::<your control here>
		{

			return true; // handled
		}

		default:
			break;
	}*/

	return false; /// not handled
}

/**
\brief For Custom View and Custom Sub-Controller Operations

NOTES:
- this is for advanced users only to implement custom view and custom sub-controllers
- see the SDK for examples of use

\param messageInfo a structure containing information about the incoming message

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMessage(MessageInfo& messageInfo)
{
	// --- decode message
	switch (messageInfo.message)
	{
		// --- add customization appearance here
	case PLUGINGUI_DIDOPEN:
	{
		return false;
	}

	// --- NULL pointers so that we don't accidentally use them
	case PLUGINGUI_WILLCLOSE:
	{
		return false;
	}

	// --- update view; this will only be called if the GUI is actually open
	case PLUGINGUI_TIMERPING:
	{
		return false;
	}

	// --- register the custom view, grab the ICustomView interface
	case PLUGINGUI_REGISTER_CUSTOMVIEW:
	{

		return false;
	}

	case PLUGINGUI_REGISTER_SUBCONTROLLER:
	case PLUGINGUI_QUERY_HASUSERCUSTOM:
	case PLUGINGUI_USER_CUSTOMOPEN:
	case PLUGINGUI_USER_CUSTOMCLOSE:
	case PLUGINGUI_EXTERNAL_SET_NORMVALUE:
	case PLUGINGUI_EXTERNAL_SET_ACTUALVALUE:
	{

		return false;
	}

	default:
		break;
	}

	return false; /// not handled
}


/**
\brief process a MIDI event

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param event a structure containing the MIDI event data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::processMIDIEvent(midiEvent& event)
{
	return true;
}

/**
\brief (for future use)

NOTES:
- MIDI events are 100% sample accurate; this function will be called repeatedly for every MIDI message
- see the SDK for examples of use

\param vectorJoysickData a structure containing joystick data

\return true if operation succeeds, false otherwise
*/
bool PluginCore::setVectorJoystickParameters(const VectorJoystickData& vectorJoysickData)
{
	return true;
}

/**
\brief use this method to add new presets to the list

NOTES:
- see the SDK for examples of use
- for non RackAFX users that have large paramter counts, there is a secret GUI control you
  can enable to write C++ code into text files, one per preset. See the SDK or http://www.willpirkle.com for details

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginPresets()
{
	// **--0xFF7A--**

	// --- Plugin Presets 
	int index = 0;
	PresetInfo* preset = nullptr;

	// --- Preset: Factory Preset
	preset = new PresetInfo(index++, "Factory Preset");
	initPresetParameters(preset->presetParameters);
	setPresetParameter(preset->presetParameters, controlID::right_dB, -20.000000);
	setPresetParameter(preset->presetParameters, controlID::left_dB, -20.000000);
	setPresetParameter(preset->presetParameters, controlID::att3_dB, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::LFF_Fc, 40.000000);
	setPresetParameter(preset->presetParameters, controlID::APF_Fc, 500.000000);
	setPresetParameter(preset->presetParameters, controlID::LF_Fc, 125.000000);
	setPresetParameter(preset->presetParameters, controlID::LF_dB, 8.000000);
	setPresetParameter(preset->presetParameters, controlID::hf_gain, 15.400000);
	setPresetParameter(preset->presetParameters, controlID::hf_Fc, 10000.000000);
	setPresetParameter(preset->presetParameters, controlID::LF_Q, 1.000000);
	setPresetParameter(preset->presetParameters, controlID::HF_Q, 0.500000);
	setPresetParameter(preset->presetParameters, controlID::APF_On, 1.000000);
	setPresetParameter(preset->presetParameters, controlID::out_dB, 0.000000);
	setPresetParameter(preset->presetParameters, controlID::eq_Listen, -0.000000);
	setPresetParameter(preset->presetParameters, controlID::pShift, 0.000000);
	addPreset(preset);


	// **--0xA7FF--**

    return true;
}

/**
\brief setup the plugin description strings, flags and codes; this is ordinarily done through the ASPiKreator or CMake

\return true if operation succeeds, false otherwise
*/
bool PluginCore::initPluginDescriptors()
{
    pluginDescriptor.pluginName = PluginCore::getPluginName();
    pluginDescriptor.shortPluginName = PluginCore::getShortPluginName();
    pluginDescriptor.vendorName = PluginCore::getVendorName();
    pluginDescriptor.pluginTypeCode = PluginCore::getPluginType();

	// --- describe the plugin attributes; set according to your needs
	pluginDescriptor.hasSidechain = kWantSidechain;
	pluginDescriptor.latencyInSamples = kLatencyInSamples;
	pluginDescriptor.tailTimeInMSec = kTailTimeMsec;
	pluginDescriptor.infiniteTailVST3 = kVSTInfiniteTail;

    // --- AAX
    apiSpecificInfo.aaxManufacturerID = kManufacturerID;
    apiSpecificInfo.aaxProductID = kAAXProductID;
    apiSpecificInfo.aaxBundleID = kAAXBundleID;  /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.aaxEffectID = "aaxDeveloper.";
    apiSpecificInfo.aaxEffectID.append(PluginCore::getPluginName());
    apiSpecificInfo.aaxPluginCategoryCode = kAAXCategory;

    // --- AU
    apiSpecificInfo.auBundleID = kAUBundleID;
	apiSpecificInfo.auBundleName = kAUBundleName;   /* MacOS only: this MUST match the bundle identifier in your info.plist file */
    apiSpecificInfo.auBundleName = kAUBundleName;

    // --- VST3
    apiSpecificInfo.vst3FUID = PluginCore::getVSTFUID(); // OLE string format
    apiSpecificInfo.vst3BundleID = kVST3BundleID;/* MacOS only: this MUST match the bundle identifier in your info.plist file */
	apiSpecificInfo.enableVST3SampleAccurateAutomation = kVSTSAA;
	apiSpecificInfo.vst3SampleAccurateGranularity = kVST3SAAGranularity;

    // --- AU and AAX
    apiSpecificInfo.fourCharCode = PluginCore::getFourCharCode();

    return true;
}

// --- static functions required for VST3/AU only --------------------------------------------- //
const char* PluginCore::getPluginBundleName() { return kAUBundleName; }
const char* PluginCore::getPluginName(){ return kPluginName; }
const char* PluginCore::getShortPluginName(){ return kShortPluginName; }
const char* PluginCore::getVendorName(){ return kVendorName; }
const char* PluginCore::getVendorURL(){ return kVendorURL; }
const char* PluginCore::getVendorEmail(){ return kVendorEmail; }
const char* PluginCore::getAUCocoaViewFactoryName(){ return AU_COCOA_VIEWFACTORY_STRING; }
pluginType PluginCore::getPluginType(){ return kPluginType; }
const char* PluginCore::getVSTFUID(){ return kVSTFUID; }
int32_t PluginCore::getFourCharCode(){ return kFourCharCode; }
