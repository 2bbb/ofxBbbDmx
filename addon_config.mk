meta:
	ADDON_NAME = ofxBbbDmx
	ADDON_DESCRIPTION = openFrameworks integration for bbb.dmx / bbb-artnet: Art-Net receive, fixture show loading, DMX parameter evaluation, GDTF / MVR import
	ADDON_AUTHOR = 2bit
	ADDON_TAGS = "dmx" "artnet" "gdtf" "mvr" "lighting"
	ADDON_URL = https://github.com/2bbb/ofxBbbDmx

common:
	ADDON_DEFINES = ASIO_STANDALONE=1 MINIZ_NO_ZLIB_COMPATIBLE_NAMES=1
	ADDON_INCLUDES = src libs/bbb-artnet/include libs/asio/asio/include libs/bbb.dmx/source libs/miniz
