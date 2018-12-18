
include(FeatureSummary)
include(CMakeDependentOption)
cmake_dependent_option(ENABLE_SOAPY_RSJET "Enable RsJet SoapySDR bindings" ON "ENABLE_LIBRARY;SoapySDR_FOUND" OFF)
add_feature_info(SoapySDRRsJet ENABLE_SOAPY_RSJET "SoapySDR bindings for RsJet")
if (NOT ENABLE_SOAPY_RSJET)
    return()
endif()


########################################################################
# build the module
########################################################################
if (NOT SoapySDR_FOUND)
    message(WARNING "SoapySDR development files not found - skipping support")
    return()
endif ()

include_directories(${USB_INCLUDE_DIRS}
${EXT_INCLUDE_DIRS}
)
set(SOAPY_SDR_SOURCES
RsJet_Registation.cpp
	RsJet_Settings.cpp
	RsJet_Streaming.cpp
	${PROJECT_SOURCE_DIR}/Src/RsJetUsb/Rs_FX3_USB.cpp
)
SOAPY_SDR_MODULE_UTIL(
    TARGET RsJetSupport
    SOURCES  ${SOAPY_SDR_SOURCES}
	LIBRARIES ${USB_LIBRARIES} ${LIQUID_LIBRARIES}
)