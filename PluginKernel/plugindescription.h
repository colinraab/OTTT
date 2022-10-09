// --- CMAKE generated variables for your plugin

#include "pluginstructures.h"

#ifndef _plugindescription_h
#define _plugindescription_h

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#define AU_COCOA_VIEWFACTORY_STRING STR(AU_COCOA_VIEWFACTORY_NAME)
#define AU_COCOA_VIEW_STRING STR(AU_COCOA_VIEW_NAME)

// --- AU Plugin Cocoa View Names (flat namespace)
#define AU_COCOA_VIEWFACTORY_NAME AUCocoaViewFactory_8C67599821F84CF080F58650C464959B
#define AU_COCOA_VIEW_NAME AUCocoaView_8C67599821F84CF080F58650C464959B

// --- BUNDLE IDs (MacOS Only)
const char* kAAXBundleID = "colinraab.aax.OTTT3.bundleID";
const char* kAUBundleID = "colinraab.au.OTTT3.bundleID";
const char* kVST3BundleID = "colinraab.vst3.OTTT3.bundleID";

// --- Plugin Names
const char* kPluginName = "OTTT3";
const char* kShortPluginName = "OTTT3";
const char* kAUBundleName = "OTTT3";
const char* kAAXBundleName = "OTTT3";
const char* kVSTBundleName = "OTTT3";

// --- bundle name helper
inline static const char* getPluginDescBundleName()
{
#ifdef AUPLUGIN
	return kAUBundleName;
#endif

#ifdef AAXPLUGIN
	return kAAXBundleName;
#endif

#ifdef VSTPLUGIN
	return kVSTBundleName;
#endif

	// --- should never get here
	return kPluginName;
}

// --- Plugin Type
const pluginType kPluginType = pluginType::kFXPlugin;

// --- VST3 UUID
const char* kVSTFUID = "{8C675998-21F8-4CF0-80F5-8650C464959B}";

// --- 4-char codes
const int32_t kFourCharCode = '5GDZ';
const int32_t kAAXProductID = '5GDZ';
const int32_t kManufacturerID = 'VNDR';

// --- Vendor information
const char* kVendorName = "Colin Raab";
const char* kVendorURL = "www.colinraab.com";
const char* kVendorEmail = "csr@raabfamily.net";

// --- Plugin Options
const bool kProcessFrames = true;
const uint32_t kBlockSize = DEFAULT_AUDIO_BLOCK_SIZE;
const bool kWantSidechain = true;
const uint32_t kLatencyInSamples = 0;
const double kTailTimeMsec = 0.000;
const bool kVSTInfiniteTail = false;
const bool kVSTSAA = false;
const uint32_t kVST3SAAGranularity = 1;
const uint32_t kAAXCategory = 0;

#endif


