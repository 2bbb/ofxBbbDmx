#pragma once

// ofxBbbDmx — openFrameworks integration layer for bbb.dmx / bbb-artnet:
// Art-Net receive (bbb::artnet::managed_node), bbb.dmx show loading
// (fixture.profile.v1 / patch.v2), DMX parameter evaluation, semantic
// fixture state (GDTF device-orientation convention), and GDTF / MVR
// import emitting bbb.dmx JSON byte-identical to bbb-dmx-convert.
//
// Header-only. Define OFX_BBB_DMX_IMPLEMENTATION in exactly one
// translation unit before including this header to embed miniz.

#define MINIZ_NO_ZLIB_COMPATIBLE_NAMES 1
#if defined(OFX_BBB_DMX_IMPLEMENTATION)
	#include "miniz.c"
#else
	#include "miniz.h"
#endif

// bbb-artnet: Art-Net node manager + protocol parsing (header-only, standalone asio)
#define ASIO_STANDALONE 1
#include <bbb/artnet/artnet_node_manager.hpp>

// bbb.dmx: fixture profile / patch data model + JSON loader (Max-independent layer)
#include "bbb/dmx/fixture_json.hpp"
#include "bbb/dmx/aim.hpp"
#include "bbb/dmx/color_mapping.hpp"

#include "ofMain.h"
#include "ofJson.h"

#include <array>
#include <chrono>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <set>

namespace bbb { namespace dmx { namespace ofx {
	using ordered_json = nlohmann::ordered_json;
	using steady_clock = std::chrono::steady_clock;

	// ------------------------------------------------------------------
	// small helpers
	// ------------------------------------------------------------------

	// same slug rules as tools/bbb-dmx-convert (lowercase, "&" -> "and",
	// non-alphanumeric runs -> ".", trimmed, capped at 96 characters)
	inline std::string sanitize_key(const std::string &text, const std::string &fallback = "unnamed") {
		std::string result{};
		bool last_was_separator{true};
		for(const char raw : text) {
			const unsigned char c{(unsigned char)raw};
			if(std::isalnum(c)) {
				result.push_back((char)std::tolower(c));
				last_was_separator = false;
			} else if(c == '&') {
				if(!last_was_separator) {
					result.push_back('.');
				}
				result += "and.";
				last_was_separator = true;
			} else if(!last_was_separator) {
				result.push_back('.');
				last_was_separator = true;
			}
		}
		while(!result.empty() && result.back() == '.') {
			result.pop_back();
		}
		if(result.empty()) {
			return fallback;
		}
		if(96 < result.size()) {
			result.resize(96);
		}
		return result;
	}

	// patch fixture ids use "_" instead of "." (same as tools/bbb-dmx-convert)
	inline std::string sanitize_identifier(const std::string &text, const std::string &fallback = "unnamed") {
		std::string result{sanitize_key(text, fallback)};
		std::replace(result.begin(), result.end(), '.', '_');
		return result;
	}

	inline void write_text_file(const std::string &path, const std::string &text) {
		ofBufferToFile(path, ofBuffer(text.c_str(), text.size()));
	}

	inline std::string to_lower(const std::string &text) {
		std::string result{text};
		std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
			return (char)std::tolower(c);
		});
		return result;
	}

	// bbb.dmx world (right-handed, +X right / +Y forward / +Z up, meters)
	// -> openFrameworks GL world (right-handed, +X right / +Y up / +Z toward viewer)
	inline glm::mat4 bbb_to_gl_matrix() {
		return glm::mat4{
			1.f, 0.f, 0.f, 0.f,   // image of bbb +X
			0.f, 0.f, -1.f, 0.f,  // image of bbb +Y
			0.f, 1.f, 0.f, 0.f,   // image of bbb +Z
			0.f, 0.f, 0.f, 1.f
		};
	}

	inline glm::vec3 bbb_to_gl_point(const glm::vec3 &p) {
		return glm::vec3{p.x, p.z, -p.y};
	}

	// ------------------------------------------------------------------
	// Art-Net receiver (bbb::artnet::managed_node, asio based)
	// ------------------------------------------------------------------

	struct universe_state {
		std::array<std::uint8_t, 512> data{};
		std::uint64_t packet_count{0};
		int last_length{0};  // channels in the most recent ArtDmx (senders may truncate < 512)
		steady_clock::time_point last_packet{};
	};

	using universe_map = std::map<int, universe_state>;

	class artnet_receiver {
	public:
		~artnet_receiver() {
			stop();
		}

		bool start(const std::string &bind_ip) {
			stop();
			node_ = bbb::artnet::managed_node::get_or_create(bind_ip.empty() ? nullptr : bind_ip.c_str());
			if(!node_ || !node_->valid()) {
				error_ = "failed to bind Art-Net node on UDP 6454"
					+ (bind_ip.empty() ? std::string{} : " (" + bind_ip + ")");
				node_.reset();
				return false;
			}
			error_.clear();
			node_->retain();
			bbb::artnet::callback_entry entry{};
			entry.type = bbb::artnet::callback_type::dmx;
			entry.owner = this;
			entry.dmx_fn = [this](const std::uint8_t *data, int length, int universe_address) {
				if(data == nullptr || length < 1) {
					return;
				}
				std::lock_guard<std::mutex> lock{mutex_};
				auto &universe = universes_[universe_address];
				const std::size_t copy_length{std::min<std::size_t>((std::size_t)length, universe.data.size())};
				std::memcpy(universe.data.data(), data, copy_length);
				universe.packet_count++;
				universe.last_length = length;
				universe.last_packet = steady_clock::now();
				total_packets_++;
			};
			node_->add_callback(entry);
			return true;
		}

		void stop() {
			if(node_) {
				node_->remove_callbacks(this);
				node_->release();
				node_.reset();
			}
		}

		std::string effective_ip() const {
			return node_ ? node_->effective_ip() : std::string{};
		}

		std::string error() const {
			std::lock_guard<std::mutex> lock{mutex_};
			return error_;
		}

		std::uint64_t total_packets() const {
			std::lock_guard<std::mutex> lock{mutex_};
			return total_packets_;
		}

		universe_map snapshot() const {
			std::lock_guard<std::mutex> lock{mutex_};
			return universes_;
		}

	private:
		std::shared_ptr<bbb::artnet::managed_node> node_{};
		mutable std::mutex mutex_;
		universe_map universes_{};
		std::uint64_t total_packets_{0};
		std::string error_{};
	};

	// ------------------------------------------------------------------
	// show data (bbb.dmx patch + profiles)
	// ------------------------------------------------------------------

	struct fixture_view {
		const bbb::dmx::fixture_instance *instance{nullptr};
		const bbb::dmx::fixture_profile *profile{nullptr};
		const bbb::dmx::fixture_mode *mode{nullptr};
		std::string error{};

		bool valid() const {
			return error.empty();
		}
	};

	struct show_data {
		std::string patch_path{};
		bbb::dmx::fixture_patch patch{};
		std::deque<bbb::dmx::fixture_profile> profiles{};
		std::vector<fixture_view> views{};
		std::vector<std::string> errors{};
	};

	inline show_data load_show(const std::string &patch_path) {
		show_data show{};
		show.patch_path = patch_path;

		const auto patch_result = bbb::dmx::read_fixture_patch_file(patch_path, show.patch);
		if(!patch_result.ok) {
			show.errors.push_back("patch: " + patch_result.message);
			return show;
		}

		const std::string base_directory{bbb::dmx::parent_directory(patch_path)};
		for(const auto &profile_path : show.patch.profile_paths) {
			const std::string full_path{
				bbb::dmx::path_is_absolute(profile_path)
					? profile_path
					: bbb::dmx::join_relative_path(base_directory, profile_path)
			};
			bbb::dmx::fixture_profile profile{};
			const auto profile_result = bbb::dmx::read_fixture_profile_file(full_path, profile);
			if(!profile_result.ok) {
				show.errors.push_back("profile " + profile_path + ": " + profile_result.message);
				continue;
			}
			show.profiles.push_back(std::move(profile));
		}

		for(const auto &instance : show.patch.fixtures) {
			fixture_view view{};
			view.instance = &instance;
			for(const auto &profile : show.profiles) {
				if(profile.key == instance.profile) {
					view.profile = &profile;
					break;
				}
			}
			if(view.profile == nullptr) {
				view.error = "profile not found: " + instance.profile;
			} else {
				view.mode = view.profile->find_mode(instance.mode);
				if(view.mode == nullptr) {
					view.error = "mode not found: " + instance.mode;
				} else if(instance.address < 1
				          || 512 < instance.address + view.mode->footprint - 1) {
					view.error = "address out of range: " + std::to_string(instance.address);
				}
			}
			if(!view.valid()) {
				show.errors.push_back(instance.id + ": " + view.error);
			}
			show.views.push_back(view);
		}
		return show;
	}

	// ------------------------------------------------------------------
	// DMX parameter read / write (read mirrors bbb.dmx fixture_mapper layout:
	// multi-byte parameters are contiguous from the first listed channel)
	// ------------------------------------------------------------------

	struct parameter_sample {
		bool found{false};
		bool live{false};
		int raw{0};
		double normalized{0.0};
	};

	inline int parameter_byte_count(bbb::dmx::fixture_parameter_type type) {
		switch(type) {
			case bbb::dmx::fixture_parameter_type::u16: return 2;
			case bbb::dmx::fixture_parameter_type::u24: return 3;
			default: return 1;
		}
	}

	inline int parameter_max_value(int byte_count) {
		switch(byte_count) {
			case 2: return 65535;
			case 3: return 16777215;
			default: return 255;
		}
	}

	inline int compose_bytes(const std::uint8_t *bytes, int byte_count, bbb::dmx::byte_order order) {
		if(byte_count == 1) {
			return bytes[0];
		}
		if(byte_count == 2) {
			if(order == bbb::dmx::byte_order::fine_coarse) {
				return ((int)bytes[1] << 8) | bytes[0];
			}
			return ((int)bytes[0] << 8) | bytes[1];
		}
		if(order == bbb::dmx::byte_order::fine_mid_coarse) {
			return ((int)bytes[2] << 16) | ((int)bytes[1] << 8) | bytes[0];
		}
		return ((int)bytes[0] << 16) | ((int)bytes[1] << 8) | bytes[2];
	}

	inline parameter_sample read_parameter(const fixture_view &view,
	                                       const std::string &key,
	                                       const universe_map &universes,
	                                       int universe_base)
	{
		parameter_sample sample{};
		if(!view.valid()) {
			return sample;
		}
		const auto *parameter = view.mode->find_parameter(key);
		if(parameter == nullptr || parameter->channels.empty()) {
			return sample;
		}
		const auto *first_channel = view.mode->find_channel(parameter->channels.front());
		if(first_channel == nullptr) {
			return sample;
		}
		const int byte_count{parameter_byte_count(parameter->type)};
		const int max_value{parameter_max_value(byte_count)};
		const int base_index{view.instance->address - 1 + first_channel->offset - 1};
		if(base_index < 0 || 512 < base_index + byte_count) {
			return sample;
		}
		sample.found = true;

		const int port_address{universe_base + view.instance->universe - 1};
		const auto found = universes.find(port_address);
		if(found != universes.end()) {
			sample.live = true;
			sample.raw = compose_bytes(found->second.data.data() + base_index, byte_count, parameter->order);
		} else {
			sample.raw = std::max(0, std::min(max_value, parameter->default_value));
		}
		sample.normalized = (double)sample.raw / (double)max_value;
		return sample;
	}

	inline void write_parameter(universe_map &universes,
	                            const fixture_view &view,
	                            const std::string &key,
	                            double normalized,
	                            int universe_base)
	{
		if(!view.valid()) {
			return;
		}
		const auto *parameter = view.mode->find_parameter(key);
		if(parameter == nullptr || parameter->channels.empty()) {
			return;
		}
		const auto *first_channel = view.mode->find_channel(parameter->channels.front());
		if(first_channel == nullptr) {
			return;
		}
		const int byte_count{parameter_byte_count(parameter->type)};
		const int base_index{view.instance->address - 1 + first_channel->offset - 1};
		if(base_index < 0 || 512 < base_index + byte_count) {
			return;
		}
		const int max_value{parameter_max_value(byte_count)};
		const int raw{(int)std::round(bbb::dmx::clamp_normalized_value(normalized) * max_value)};
		auto &universe = universes[universe_base + view.instance->universe - 1];
		std::array<std::uint8_t, 3> bytes{};
		for(int index = 0; index < byte_count; index++) {
			bytes[(std::size_t)index] = (std::uint8_t)((raw >> (8 * (byte_count - 1 - index))) & 255);
		}
		const bool reversed{
			parameter->order == bbb::dmx::byte_order::fine_coarse
			|| parameter->order == bbb::dmx::byte_order::fine_mid_coarse
		};
		for(int index = 0; index < byte_count; index++) {
			const int source{reversed ? byte_count - 1 - index : index};
			universe.data[(std::size_t)(base_index + index)] = bytes[(std::size_t)source];
		}
	}

	// ------------------------------------------------------------------
	// per-fixture render state
	// ------------------------------------------------------------------

	struct fixture_render_state {
		bool valid{false};
		bool mover{false};
		bool emitting{true};  // false: positional marker etc., no photometric parameters
		glm::vec3 position{};        // bbb coords (meters)
		glm::vec3 beam_direction{};  // bbb coords, unit vector
		ofFloatColor color{1.f, 1.f, 1.f};
		float intensity{1.f};
		float half_angle{8.f};
		float beam_start_radius{0.03f};
	};

	// shutter handling. with a "ranges" table (bbb.dmx issue #7) the DMX value is
	// looked up exactly: closed / open / strobe / pulse / random, rate from the
	// physical Hz pair when present, otherwise pseudo (position within the range).
	// without a table: pseudo model (0 = closed, mid = strobe, top = open).
	//
	// both paths are gated on live DMX: a fixture is never blacked out purely by
	// a profile shutter default (commonly 0 = closed in GDTF) when no console is
	// driving its universe. otherwise a patch mixing fixtures whose shutter
	// default maps to "open" with ones that map to "closed" would render an
	// inconsistent half-dark idle scene.
	inline float shutter_factor(const fixture_view &view, const parameter_sample &shutter, double time) {
		if(!shutter.found || !shutter.live) {
			return 1.f;
		}
		const auto *parameter = view.valid() ? view.mode->find_parameter("shutter") : nullptr;
		if(parameter != nullptr && 2 <= parameter->ranges.size()) {
			const bbb::dmx::fixture_parameter_range *match{nullptr};
			for(const auto &range : parameter->ranges) {
				if(range.from <= shutter.raw && shutter.raw <= range.to) {
					match = &range;
					break;
				}
			}
			if(match == nullptr) {
				return 1.f;
			}
			const double span{(double)std::max(1, match->to - match->from)};
			const double position{(shutter.raw - match->from) / span};
			const bool has_physical{match->has_physical_from && match->has_physical_to
			                        && match->physical_from != match->physical_to};
			const double rate{std::max(0.1, has_physical
				? match->physical_from + (match->physical_to - match->physical_from) * position
				: 1.0 + position * 11.0)};
			if(match->function == "closed") {
				return 0.f;
			}
			if(match->function == "strobe") {
				return std::fmod(time * rate, 1.0) < 0.5 ? 1.f : 0.f;
			}
			if(match->function == "pulse") {
				return (float)(0.5 + 0.5 * std::sin(time * rate * glm::two_pi<double>()));
			}
			if(match->function == "random") {
				const double cycle{std::floor(time * rate * 2.0)};
				const double noise{std::fmod(std::abs(std::sin(cycle * 12.9898)) * 43758.5453, 1.0)};
				return noise < 0.5 ? 1.f : 0.f;
			}
			return 1.f;  // open / unknown slugs pass through
		}
		const double value{shutter.normalized};
		if(value < 0.02) {
			return 0.f;
		}
		if(value < 0.94) {
			const double frequency{1.0 + value * 11.0};
			return std::fmod(time * frequency, 1.0) < 0.5 ? 1.f : 0.f;
		}
		return 1.f;
	}

	// treat a parameter as a color-wheel source if it carries wheel metadata, or
	// if it is simply named "color"/"colour" with a range table — older imports
	// store the wheel as a "color" parameter whose ranges only have descriptive
	// function/label slugs (e.g. "deep.red.positioning"), which still resolve via
	// name matching. Color temperature / macro params are excluded.
	inline bool is_color_wheel_parameter(const bbb::dmx::fixture_parameter &parameter) {
		if(bbb::dmx::parameter_is_likely_color_wheel(parameter)) {
			return true;
		}
		const std::string key{bbb::dmx::normalized_color_key(parameter.key)};
		if(key.find("color") == std::string::npos && key.find("colour") == std::string::npos) {
			return false;
		}
		if(key.find("temperature") != std::string::npos
		   || key.find("macro") != std::string::npos
		   || key.find("correction") != std::string::npos
		   || key == "ctc" || key == "cto" || key == "ctb") {
			return false;
		}
		return 2 <= parameter.ranges.size();
	}

	// resolve a color-wheel parameter's current DMX value to a display color
	// using the profile's wheel-slot metadata (bbb-dmx color wheel fallback
	// data, GDTF-derived) or range function/label name matching. Reverse of the
	// sender-side RGB->nearest-slot mapping.
	inline bool color_wheel_color(const fixture_view &view,
	                              const universe_map &universes,
	                              int universe_base,
	                              ofFloatColor &out)
	{
		if(!view.valid()) {
			return false;
		}
		for(const auto &parameter : view.mode->parameters) {
			if(!is_color_wheel_parameter(parameter)) {
				continue;
			}
			const auto sample = read_parameter(view, parameter.key, universes, universe_base);
			if(!sample.found) {
				continue;
			}
			for(const auto &range : parameter.ranges) {
				if(range.from <= sample.raw && sample.raw <= range.to) {
					bbb::dmx::semantic_color_request slot_color{};
					if(bbb::dmx::color_from_range_metadata(view.profile, parameter, range, slot_color)) {
						out = ofFloatColor{(float)slot_color.red, (float)slot_color.green, (float)slot_color.blue};
						return true;
					}
					break;  // active range carries no color; try another color param
				}
			}
		}
		return false;
	}

	// ------------------------------------------------------------------
	// semantic overrides (bbb.dmx.semantic_overrides.v1)
	//
	// Per profile+mode hints that remap awkward GDTF-derived parameter keys onto
	// the renderer's color / intensity model. Reuses the very file the bbb.dmx
	// tooling consumes (bin/data/semantic-overrides.json). The importer keeps
	// emitting keys byte-identical to bbb-dmx-utils (e.g. ColorSub_C -> colorsub.c);
	// the override layer is where those map back to cyan/magenta/yellow etc.
	// ------------------------------------------------------------------

	struct semantic_color_group {
		bool subtractive{false};                          // true: cmy, false: rgb
		std::string red{}, green{}, blue{}, white{};      // rgb keys (white optional)
		std::string cyan{}, magenta{}, yellow{};          // cmy keys
		std::string dimmer{};                             // optional per-emitter intensity key
	};

	struct semantic_mode_override {
		std::vector<std::string> intensity_parameters{};  // contributors; brightest wins
		std::vector<semantic_color_group> color_groups{};
	};

	// profile.key -> mode.key -> override
	using semantic_overrides = std::map<std::string, std::map<std::string, semantic_mode_override>>;

	inline const semantic_mode_override *find_semantic_override(
		const semantic_overrides *table,
		const std::string &profile_key,
		const std::string &mode_key)
	{
		if(table == nullptr) {
			return nullptr;
		}
		const auto profile = table->find(profile_key);
		if(profile == table->end()) {
			return nullptr;
		}
		const auto mode = profile->second.find(mode_key);
		if(mode == profile->second.end()) {
			return nullptr;
		}
		return &mode->second;
	}

	inline semantic_overrides load_semantic_overrides(const std::string &path) {
		semantic_overrides table{};
		if(!ofFile::doesFileExist(path)) {
			return table;
		}
		ofJson root{};
		try {
			root = ofLoadJson(path);
		} catch(const std::exception &e) {
			ofLogError("lsim::load_semantic_overrides") << "failed to parse " << path << ": " << e.what();
			return table;
		}
		if(!root.contains("profiles")) {
			return table;
		}
		for(auto profile_it = root["profiles"].begin(); profile_it != root["profiles"].end(); ++profile_it) {
			const ofJson &profile_node = profile_it.value();
			if(!profile_node.contains("modes")) {
				continue;
			}
			const ofJson &modes = profile_node["modes"];
			for(auto mode_it = modes.begin(); mode_it != modes.end(); ++mode_it) {
				const ofJson &mode_node = mode_it.value();
				semantic_mode_override entry{};
				if(mode_node.contains("intensity")) {
					const ofJson &intensity = mode_node["intensity"];
					if(intensity.contains("primary")) {
						entry.intensity_parameters.push_back(intensity["primary"].get<std::string>());
					}
					if(intensity.contains("parameters")) {
						for(const auto &key : intensity["parameters"]) {
							const std::string value{key.get<std::string>()};
							if(std::find(entry.intensity_parameters.begin(),
							             entry.intensity_parameters.end(), value)
							   == entry.intensity_parameters.end()) {
								entry.intensity_parameters.push_back(value);
							}
						}
					}
				}
				if(mode_node.contains("color")) {
					const ofJson &color = mode_node["color"];
					const auto load_groups = [&](const char *field, bool subtractive) {
						if(!color.contains(field)) {
							return;
						}
						for(const auto &group_node : color[field]) {
							semantic_color_group group{};
							group.subtractive = subtractive;
							group.red = group_node.value("red", std::string{});
							group.green = group_node.value("green", std::string{});
							group.blue = group_node.value("blue", std::string{});
							group.white = group_node.value("white", std::string{});
							group.cyan = group_node.value("cyan", std::string{});
							group.magenta = group_node.value("magenta", std::string{});
							group.yellow = group_node.value("yellow", std::string{});
							group.dimmer = group_node.value("dimmer", std::string{});
							entry.color_groups.push_back(group);
						}
					};
					load_groups("rgb", false);
					load_groups("cmy", true);
				}
				table[profile_it.key()][mode_it.key()] = entry;
			}
		}
		return table;
	}

	struct render_options {
	public:
		int universe_base{0};
		double default_beam_half_angle{8.0};
		const semantic_overrides *overrides{nullptr};
	};

	inline fixture_render_state compute_render_state(const fixture_view &view,
	                                                 const universe_map &universes,
	                                                 const render_options &options,
	                                                 double time)
	{
		fixture_render_state state{};
		if(!view.valid()) {
			return state;
		}
		state.valid = true;
		const auto &instance = *view.instance;
		state.position = glm::vec3{(float)instance.position.x,
		                           (float)instance.position.y,
		                           (float)instance.position.z};

		const int base{options.universe_base};

		bool color_found{false};
		bool intensity_found{false};
		float intensity_value{1.f};

		const semantic_mode_override *override_entry{
			find_semantic_override(options.overrides, view.profile->key, view.mode->key)};
		if(override_entry != nullptr) {
			// explicit color / intensity mapping. multiple color groups (plate +
			// beam, pixel segments...) collapse to a single emission color by
			// per-channel max; intensity is the brightest of its contributors.
			ofFloatColor mixed{0.f, 0.f, 0.f};
			for(const auto &group : override_entry->color_groups) {
				ofFloatColor group_color{0.f, 0.f, 0.f};
				bool group_found{false};
				if(group.subtractive) {
					const auto c = read_parameter(view, group.cyan, universes, base);
					const auto m = read_parameter(view, group.magenta, universes, base);
					const auto y = read_parameter(view, group.yellow, universes, base);
					if(c.found || m.found || y.found) {
						group_color = ofFloatColor{1.f - (float)c.normalized,
						                           1.f - (float)m.normalized,
						                           1.f - (float)y.normalized};
						group_found = true;
					}
				} else {
					const auto r = read_parameter(view, group.red, universes, base);
					const auto g = read_parameter(view, group.green, universes, base);
					const auto b = read_parameter(view, group.blue, universes, base);
					if(r.found || g.found || b.found) {
						group_color = ofFloatColor{(float)r.normalized, (float)g.normalized, (float)b.normalized};
						group_found = true;
					}
					if(!group.white.empty()) {
						const auto w = read_parameter(view, group.white, universes, base);
						if(w.found) {
							group_color.r = std::min(1.f, group_color.r + (float)w.normalized);
							group_color.g = std::min(1.f, group_color.g + (float)w.normalized);
							group_color.b = std::min(1.f, group_color.b + (float)w.normalized);
							group_found = true;
						}
					}
				}
				if(group_found) {
					mixed.r = std::max(mixed.r, group_color.r);
					mixed.g = std::max(mixed.g, group_color.g);
					mixed.b = std::max(mixed.b, group_color.b);
					color_found = true;
				}
			}
			state.color = color_found ? mixed : ofFloatColor{1.f, 1.f, 1.f};

			float max_intensity{0.f};
			for(const auto &key : override_entry->intensity_parameters) {
				const auto sample = read_parameter(view, key, universes, base);
				if(sample.found) {
					max_intensity = std::max(max_intensity, (float)sample.normalized);
					intensity_found = true;
				}
			}
			intensity_value = intensity_found ? max_intensity : 1.f;
		} else {
			// default heuristic: fixed key names (red/green/blue, cyan/magenta/yellow,
			// color wheel, white/amber/uv), intensity from "dimmer".
			const auto red = read_parameter(view, "red", universes, base);
			const auto green = read_parameter(view, "green", universes, base);
			const auto blue = read_parameter(view, "blue", universes, base);
			const auto white = read_parameter(view, "white", universes, base);
			const auto cyan = read_parameter(view, "cyan", universes, base);
			const auto magenta = read_parameter(view, "magenta", universes, base);
			const auto yellow = read_parameter(view, "yellow", universes, base);

			ofFloatColor wheel_color{};
			const bool has_wheel{color_wheel_color(view, universes, base, wheel_color)};
			if(red.found || green.found || blue.found) {
				state.color = ofFloatColor{(float)red.normalized, (float)green.normalized, (float)blue.normalized};
			} else if(cyan.found || magenta.found || yellow.found) {
				state.color = ofFloatColor{1.f - (float)cyan.normalized,
				                           1.f - (float)magenta.normalized,
				                           1.f - (float)yellow.normalized};
			} else if(has_wheel) {
				// no additive/subtractive mixing channels: a color wheel is the
				// fixture's only hue source, so it sets the color outright.
				state.color = wheel_color;
			} else {
				state.color = ofFloatColor{1.f, 1.f, 1.f};
			}
			if(white.found) {
				state.color.r = std::min(1.f, state.color.r + (float)white.normalized);
				state.color.g = std::min(1.f, state.color.g + (float)white.normalized);
				state.color.b = std::min(1.f, state.color.b + (float)white.normalized);
			}
			const auto amber = read_parameter(view, "amber", universes, base);
			if(amber.found) {
				state.color.r = std::min(1.f, state.color.r + (float)amber.normalized);
				state.color.g = std::min(1.f, state.color.g + 0.55f * (float)amber.normalized);
			}
			const auto uv = read_parameter(view, "uv", universes, base);
			if(uv.found) {
				state.color.r = std::min(1.f, state.color.r + 0.25f * (float)uv.normalized);
				state.color.b = std::min(1.f, state.color.b + 0.6f * (float)uv.normalized);
			}
			const auto dimmer = read_parameter(view, "dimmer", universes, base);
			intensity_found = dimmer.found;
			intensity_value = dimmer.found ? (float)dimmer.normalized : 1.f;
			color_found = red.found || green.found || blue.found
				|| white.found || amber.found || uv.found
				|| cyan.found || magenta.found || yellow.found;
		}

		state.intensity = intensity_value;
		state.intensity *= shutter_factor(view, read_parameter(view, "shutter", universes, base), time);

		const auto pan = read_parameter(view, "pan", universes, base);
		const auto tilt = read_parameter(view, "tilt", universes, base);
		state.emitting = intensity_found || color_found || (pan.found && tilt.found);
		if(pan.found && tilt.found) {
			state.mover = true;
			const auto *pan_parameter = view.mode->find_parameter("pan");
			const auto *tilt_parameter = view.mode->find_parameter("tilt");
			const double pan_range{bbb::dmx::sanitize_positive_range(pan_parameter->range_degrees, 540.0)};
			const double tilt_range{bbb::dmx::sanitize_positive_range(tilt_parameter->range_degrees, 270.0)};
			const std::uint16_t pan16{(std::uint16_t)(pan_parameter->type == bbb::dmx::fixture_parameter_type::u8 ? pan.raw << 8 : pan.raw)};
			const std::uint16_t tilt16{(std::uint16_t)(tilt_parameter->type == bbb::dmx::fixture_parameter_type::u8 ? tilt.raw << 8 : tilt.raw)};
			double pan_degrees{bbb::dmx::u16_to_angle(pan16, pan_range)};
			double tilt_degrees{bbb::dmx::u16_to_angle(tilt16, tilt_range)};

			// invert bbb.dmx.movertrack calibration: dmx = invert ? -(raw + offset) : (raw + offset)
			const auto &calibration = instance.calibration;
			pan_degrees = (calibration.pan_invert ? -pan_degrees : pan_degrees) - calibration.pan_offset;
			tilt_degrees = (calibration.tilt_invert ? -tilt_degrees : tilt_degrees) - calibration.tilt_offset;

			const double pan_radians{bbb::dmx::degrees_to_radians(pan_degrees)};
			const double tilt_radians{bbb::dmx::degrees_to_radians(tilt_degrees)};
			// GDTF convention (bbb.dmx.patch.v2): hanging rest pose, beam rest = -Z,
			// pan about device +Z, tilt about device +X:
			// v = Rz(pan) * Rx(tilt) * (0, 0, -1)
			const bbb::dmx::vec3 local_direction{
				-std::sin(pan_radians) * std::sin(tilt_radians),
				std::cos(pan_radians) * std::sin(tilt_radians),
				-std::cos(tilt_radians)
			};
			const auto rotation = bbb::dmx::fixture_rotation_xyz_degrees(
				instance.rotation.x, instance.rotation.y, instance.rotation.z);
			const auto world_direction = rotation * local_direction;
			state.beam_direction = glm::normalize(glm::vec3{
				(float)world_direction.x, (float)world_direction.y, (float)world_direction.z});

			// beam angle priority: live zoom > profile photometry > global default
			const auto &photometry = view.profile->photometry;
			const auto zoom = read_parameter(view, "zoom", universes, base);
			if(zoom.found) {
				state.half_angle = 2.f + (float)zoom.normalized * 23.f;
			} else if(photometry.has_beam_angle_degrees && 0.0 < photometry.beam_angle_degrees) {
				state.half_angle = (float)(photometry.beam_angle_degrees * 0.5);
			} else {
				state.half_angle = (float)options.default_beam_half_angle;
			}
			if(photometry.has_beam_radius && 0.0 < photometry.beam_radius) {
				state.beam_start_radius = (float)photometry.beam_radius;
			}
		}
		return state;
	}

	// ------------------------------------------------------------------
	// zip helpers (miniz)
	// ------------------------------------------------------------------

	struct zip_entry {
		std::string name{};
		std::vector<std::uint8_t> data{};
	};

	inline bool zip_extract_entry(mz_zip_archive &zip, int index, std::vector<std::uint8_t> &out) {
		std::size_t size{0};
		void *pointer{mz_zip_reader_extract_to_heap(&zip, (mz_uint)index, &size, 0)};
		if(pointer == nullptr) {
			return false;
		}
		out.assign((std::uint8_t *)pointer, (std::uint8_t *)pointer + size);
		mz_free(pointer);
		return true;
	}

	inline int zip_find_entry(mz_zip_archive &zip, const std::string &name_lower) {
		const int count{(int)mz_zip_reader_get_num_files(&zip)};
		for(int index = 0; index < count; index++) {
			mz_zip_archive_file_stat stat{};
			if(!mz_zip_reader_file_stat(&zip, (mz_uint)index, &stat)) {
				continue;
			}
			if(to_lower(stat.m_filename) == name_lower) {
				return index;
			}
		}
		return -1;
	}

	// ------------------------------------------------------------------
	// GDTF importer: description.xml -> bbb.dmx.fixture.profile.v1 JSON
	// ------------------------------------------------------------------

	struct gdtf_mode_info {
		std::string label{};  // original GDTF mode name
		std::string key{};    // sanitized mode key
	};

	struct gdtf_import_result {
		bool ok{false};
		std::string error{};
		std::string profile_key{};
		std::string output_path{};
		std::vector<gdtf_mode_info> modes{};
		std::vector<std::string> warnings{};
	};

	// GDTF attribute name -> bbb.dmx semantic parameter key. Mirrors the
	// regex-pair logic of bbb-dmx-utils normalizeAttributeName (substring match
	// on the lowercased attribute, first hit wins; unknown -> slug) so importer
	// output stays byte-identical to the CLI.
	inline std::string map_gdtf_attribute(const std::string &attribute) {
		const std::string normalized{to_lower(attribute)};
		static const std::vector<std::pair<std::regex, std::string>> pairs{
			{std::regex(R"(^pan$|panrotate|pan rotate|^p\b)"), "pan"},
			{std::regex(R"(^tilt$|tiltrotate|tilt rotate|^t\b)"), "tilt"},
			{std::regex(R"(dimmer|intensity|master)"), "dimmer"},
			{std::regex(R"(shutter|strobe)"), "shutter"},
			{std::regex(R"(coloradd[_\s-]*r|additive.*red|\bred\b|^r$)"), "red"},
			{std::regex(R"(coloradd[_\s-]*g|additive.*green|\bgreen\b|^g$)"), "green"},
			{std::regex(R"(coloradd[_\s-]*b|additive.*blue|\bblue\b|^b$)"), "blue"},
			{std::regex(R"(white|coloradd[_\s-]*w|^w$)"), "white"},
			{std::regex(R"(amber|coloradd[_\s-]*a|^a$)"), "amber"},
			{std::regex(R"(uv|ultraviolet)"), "uv"},
			{std::regex(R"(cyan)"), "cyan"},
			{std::regex(R"(magenta)"), "magenta"},
			{std::regex(R"(yellow)"), "yellow"},
			{std::regex(R"(zoom)"), "zoom"},
			{std::regex(R"(focus)"), "focus"},
			{std::regex(R"(iris)"), "iris"},
			{std::regex(R"(gobo)"), "gobo"},
			{std::regex(R"(colorwheel|color wheel|^color$)"), "color"},
			{std::regex(R"(cto|ctb|ctc|color temperature)"), "ctc"},
			{std::regex(R"(prism)"), "prism"},
			{std::regex(R"(frost)"), "frost"},
			{std::regex(R"(speed)"), "speed"},
		};
		for(const auto &pair : pairs) {
			if(std::regex_search(normalized, pair.first)) {
				return pair.second;
			}
		}
		return sanitize_key(attribute);
	}

	// parse GDTF DMXValue: "value/n" or "value/ns" (byte-shifted), or plain int
	inline bool parse_gdtf_dmx_value(const std::string &text, long long &value, int &bytes) {
		if(text.empty()) {
			return false;
		}
		char *end{nullptr};
		value = std::strtoll(text.c_str(), &end, 10);
		bytes = 1;
		if(end != nullptr && *end == '/') {
			bytes = (int)std::strtol(end + 1, nullptr, 10);
			if(bytes < 1) {
				bytes = 1;
			}
		}
		return true;
	}

	// one entry of a parameter "ranges" table (bbb.dmx issue #7)
	struct gdtf_range_entry {
		long long from{0};
		long long to{0};
		std::string function{};
		std::string label{};
		std::string wheel{};        // referenced wheels[].id (bbb-dmx color wheel data)
		bool has_wheel_slot{false};
		int wheel_slot{0};
		bool has_physical{false};
		double physical_from{0.0};
		double physical_to{0.0};
	};

	// integral numbers as int, fractional as double (matches bbb-dmx-utils output)
	inline ordered_json json_number(double value) {
		if(value == std::floor(value)) {
			return (long long)value;
		}
		return value;
	}

	// GDTF ChannelFunction @Wheel -> wheels[].id reference
	inline std::string wheel_name_for_function(const ofXml &function_node) {
		std::string raw{function_node.getAttribute("Wheel").getValue()};
		if(raw.empty()) {
			raw = function_node.getAttribute("WheelName").getValue();
		}
		return raw.empty() ? std::string{} : sanitize_key(raw, raw);
	}

	// GDTF ChannelSet/ChannelFunction @WheelSlotIndex (1-based)
	inline bool wheel_slot_index_for_node(const ofXml &node, int &out_slot) {
		for(const char *attribute : {"WheelSlotIndex", "WheelSlot", "Slot"}) {
			const std::string text{node.getAttribute(attribute).getValue()};
			if(!text.empty()) {
				const int value{(int)std::lround(ofToDouble(text))};
				if(0 < value) {
					out_slot = value;
					return true;
				}
			}
		}
		return false;
	}

	inline std::string wheel_type_from_name(const std::string &name) {
		const std::string normalized{to_lower(name)};
		if(normalized.find("color") != std::string::npos || normalized.find("colour") != std::string::npos
		   || normalized.find("cto") != std::string::npos || normalized.find("ctb") != std::string::npos
		   || normalized.find("ctc") != std::string::npos) {
			return "color";
		}
		if(normalized.find("gobo") != std::string::npos) {
			return "gobo";
		}
		if(normalized.find("anim") != std::string::npos) {
			return "animation";
		}
		if(normalized.find("prism") != std::string::npos) {
			return "prism";
		}
		return "generic";
	}

	// parse a GDTF color string ("{x,y,Y}" or "x,y,Y") into up to 3 numbers
	inline bool parse_color_numbers(const std::string &text, std::array<double, 3> &out) {
		std::string cleaned{text};
		for(char &c : cleaned) {
			if(c == '{' || c == '}') {
				c = ' ';
			}
		}
		std::vector<double> values{};
		for(const auto &part : ofSplitString(cleaned, ",", true, true)) {
			for(const auto &token : ofSplitString(part, " ", true, true)) {
				const std::string trimmed{ofTrim(token)};
				if(!trimmed.empty()) {
					values.push_back(ofToDouble(trimmed));
				}
			}
		}
		if(values.size() < 3) {
			return false;
		}
		out = {values[0], values[1], values[2]};
		return true;
	}

	inline bool color_name_to_rgb(const std::string &name, std::array<int, 3> &out) {
		const std::string n{to_lower(name)};
		const auto has = [&](const char *needle) { return n.find(needle) != std::string::npos; };
		if(has("open") || has("white")) { out = {255, 255, 255}; return true; }
		if(has("red"))                  { out = {255, 0, 0};     return true; }
		if(has("green"))                { out = {0, 255, 0};     return true; }
		if(has("blue"))                 { out = {0, 0, 255};     return true; }
		if(has("cyan"))                 { out = {0, 255, 255};   return true; }
		if(has("magenta") || has("pink")) { out = {255, 0, 255}; return true; }
		if(has("yellow"))               { out = {255, 255, 0};   return true; }
		if(has("amber") || has("orange")) { out = {255, 115, 0}; return true; }
		if(has("lime"))                 { out = {115, 255, 0};   return true; }
		if(has("purple") || has("violet") || n == "uv") { out = {115, 0, 255}; return true; }
		return false;
	}

	// emit profile-level wheels[] from GDTF <Wheels>/<Wheel> (bbb-dmx color wheel
	// metadata; mirrors bbb-dmx-utils fixtureWheelsFromGdtf for byte parity)
	inline ordered_json gdtf_wheels(const ofXml &fixture_type) {
		ordered_json wheels_json(ordered_json::value_t::array);
		auto wheel_nodes = fixture_type.find("Wheels/Wheel");
		if(wheel_nodes.empty()) {
			wheel_nodes = fixture_type.find("//Wheel");
		}
		int wheel_index{0};
		for(auto &wheel_node : wheel_nodes) {
			wheel_index++;
			std::string label{wheel_node.getAttribute("Name").getValue()};
			if(label.empty()) {
				label = "Wheel " + std::to_string(wheel_index);
			}
			ordered_json wheel_json{};
			wheel_json["id"] = sanitize_key(label, "wheel" + std::to_string(wheel_index));
			wheel_json["label"] = label;
			wheel_json["type"] = wheel_type_from_name(label);

			std::vector<ofXml> slot_nodes{};
			for(auto &slot : wheel_node.getChildren("Slot")) {
				slot_nodes.push_back(slot);
			}
			for(auto &nested : wheel_node.find("Slots/Slot")) {
				slot_nodes.push_back(nested);
			}
			ordered_json slots_json(ordered_json::value_t::array);
			int slot_index{0};
			for(auto &slot_node : slot_nodes) {
				slot_index++;
				std::string slot_label{slot_node.getAttribute("Name").getValue()};
				if(slot_label.empty()) {
					slot_label = "Slot " + std::to_string(slot_index);
				}
				ordered_json slot_json{};
				slot_json["index"] = slot_index;
				slot_json["id"] = sanitize_key(slot_label, "slot" + std::to_string(slot_index));
				slot_json["label"] = slot_label;
				slot_json["kind"] = (to_lower(slot_label).find("open") != std::string::npos) ? "open" : "color";
				std::array<double, 3> color_values{};
				std::array<double, 3> rgb_values{};
				std::array<int, 3> named_rgb{};
				std::string rgb_text{slot_node.getAttribute("RGB").getValue()};
				if(rgb_text.empty()) {
					rgb_text = slot_node.getAttribute("sRGB").getValue();
				}
				std::string cie_text{slot_node.getAttribute("Color").getValue()};
				if(cie_text.empty()) {
					cie_text = slot_node.getAttribute("ColorCIE").getValue();
				}
				if(!rgb_text.empty() && parse_color_numbers(rgb_text, rgb_values)) {
					slot_json["rgb"] = {
						std::max(0, std::min(255, (int)std::lround(rgb_values[0]))),
						std::max(0, std::min(255, (int)std::lround(rgb_values[1]))),
						std::max(0, std::min(255, (int)std::lround(rgb_values[2])))
					};
				} else if(!cie_text.empty() && parse_color_numbers(cie_text, color_values)) {
					slot_json["cie_xyY"] = {json_number(color_values[0]), json_number(color_values[1]), json_number(color_values[2])};
				} else if(color_name_to_rgb(slot_label, named_rgb)) {
					slot_json["rgb"] = {named_rgb[0], named_rgb[1], named_rgb[2]};
				}
				const std::string filter{slot_node.getAttribute("Filter").getValue()};
				const std::string media{slot_node.getAttribute("MediaFileName").getValue()};
				if(!filter.empty()) {
					slot_json["filter"] = filter;
				}
				if(!media.empty()) {
					slot_json["media"] = media;
				}
				slots_json.push_back(slot_json);
			}
			if(!slots_json.empty()) {
				wheel_json["slots"] = slots_json;
			}
			wheels_json.push_back(wheel_json);
		}
		return wheels_json;
	}

	// ChannelFunction attribute / ChannelSet name -> range function key
	inline std::string range_function_key(const std::string &attribute, const std::string &set_name) {
		const std::string set_lower{to_lower(set_name)};
		if(!set_lower.empty()) {
			if(set_lower.find("closed") != std::string::npos
			   || set_lower.find("blackout") != std::string::npos
			   || set_lower == "close") {
				return "closed";
			}
			if(set_lower.find("open") != std::string::npos) {
				return "open";
			}
		}
		const std::string attribute_lower{to_lower(attribute)};
		if(attribute_lower.find("stroberandom") != std::string::npos) {
			return "random";
		}
		if(attribute_lower.find("strobepulse") != std::string::npos) {
			return "pulse";
		}
		if(attribute_lower.find("strobe") != std::string::npos) {
			return "strobe";
		}
		if(attribute_lower.rfind("shutter", 0) == 0) {
			return set_lower.empty() ? "open" : sanitize_key(set_name);
		}
		if(!set_lower.empty()) {
			return sanitize_key(set_name);
		}
		return sanitize_key(attribute);
	}

	inline long long scale_dmx_value(long long value, int from_bytes, int to_bytes) {
		while(from_bytes < to_bytes) {
			value <<= 8;
			from_bytes++;
		}
		while(to_bytes < from_bytes) {
			value >>= 8;
			from_bytes--;
		}
		return value;
	}

	inline gdtf_import_result import_gdtf_bytes(const std::vector<std::uint8_t> &bytes,
	                                            const std::string &fixtures_directory)
	{
		gdtf_import_result result{};
		mz_zip_archive zip{};
		if(!mz_zip_reader_init_mem(&zip, bytes.data(), bytes.size(), 0)) {
			result.error = "not a zip archive";
			return result;
		}
		const int description_index{zip_find_entry(zip, "description.xml")};
		std::vector<std::uint8_t> xml_bytes{};
		if(description_index < 0 || !zip_extract_entry(zip, description_index, xml_bytes)) {
			mz_zip_reader_end(&zip);
			result.error = "description.xml not found in GDTF archive";
			return result;
		}
		mz_zip_reader_end(&zip);

		ofXml xml{};
		if(!xml.parse(std::string{xml_bytes.begin(), xml_bytes.end()})) {
			result.error = "failed to parse description.xml";
			return result;
		}
		const auto fixture_type = xml.findFirst("//FixtureType");
		if(!fixture_type) {
			result.error = "FixtureType element not found";
			return result;
		}
		// attribute precedence mirrors bbb-dmx-utils so the profile key matches
		// the CLI byte-for-byte. GLP-style fixtures repeat the manufacturer in
		// LongName ("GLP JDC 1"), so LongName-first yields glp.glp.jdc.1 rather
		// than the shorter glp.jdc.1 -- the longer key is preferred (less likely
		// to collide than a name shared across vendors).
		const auto first_attribute = [&](std::initializer_list<const char *> names) -> std::string {
			for(const char *attribute : names) {
				const std::string value{fixture_type.getAttribute(attribute).getValue()};
				if(!value.empty()) {
					return value;
				}
			}
			return "";
		};
		std::string name{first_attribute({"LongName", "Name", "ShortName", "Model", "model"})};
		std::string manufacturer{first_attribute({"Manufacturer", "manufacturer", "Company", "Vendor"})};
		if(manufacturer.empty()) {
			manufacturer = "Unknown";
		}
		result.profile_key = sanitize_key(manufacturer) + "." + sanitize_key(name);

		ordered_json profile_json{};
		profile_json["schema"] = "bbb.dmx.fixture.profile.v1";
		profile_json["key"] = result.profile_key;
		profile_json["manufacturer"] = manufacturer;
		profile_json["model"] = name;
		ordered_json modes_json(ordered_json::value_t::object);

		auto mode_nodes = xml.find("//DMXModes/DMXMode");
		for(auto &mode_node : mode_nodes) {
			const std::string mode_name{mode_node.getAttribute("Name").getValue()};
			const std::string mode_key{sanitize_key(mode_name)};

			ordered_json channels_json(ordered_json::value_t::array);
			ordered_json parameters_json(ordered_json::value_t::object);
			std::set<std::string> used_keys{};
			std::set<int> used_offsets{};
			int footprint{0};
			int first_break{-1};
			int channel_index{0};

			auto channel_nodes = mode_node.find("DMXChannels/DMXChannel");
			for(auto &channel_node : channel_nodes) {
				channel_index++;
				const std::string offset_text{channel_node.getAttribute("Offset").getValue()};
				if(offset_text.empty() || offset_text == "None") {
					continue;  // virtual channel without DMX footprint
				}
				int dmx_break{1};
				const std::string break_text{channel_node.getAttribute("DMXBreak").getValue()};
				if(!break_text.empty()) {
					dmx_break = ofToInt(break_text);
				}
				if(first_break < 0) {
					first_break = dmx_break;
				}
				if(dmx_break != first_break) {
					result.warnings.push_back(mode_name + ": multi-break channel skipped (break "
					                          + std::to_string(dmx_break) + ")");
					continue;
				}

				std::vector<int> offsets{};
				for(const auto &part : ofSplitString(offset_text, ",", true, true)) {
					offsets.push_back(ofToInt(part));
				}
				if(offsets.empty()) {
					continue;
				}
				int byte_count{(int)std::min<std::size_t>(offsets.size(), 3)};
				bool contiguous{true};
				for(int index = 1; index < byte_count; index++) {
					if(offsets[(std::size_t)index] != offsets[(std::size_t)(index - 1)] + 1) {
						contiguous = false;
						break;
					}
				}
				if(!contiguous) {
					result.warnings.push_back(mode_name + ": non-contiguous offsets, using coarse byte only");
					byte_count = 1;
				}

				// SPEC.json.md 2.4: no duplicate offsets inside one mode
				bool offset_collision{false};
				for(int index = 0; index < byte_count; index++) {
					if(used_offsets.count(offsets[(std::size_t)index]) != 0) {
						offset_collision = true;
						break;
					}
				}
				if(offset_collision) {
					result.warnings.push_back(mode_name + ": duplicate channel offset "
					                          + std::to_string(offsets.front()) + ", channel skipped");
					continue;
				}

				// attribute lookup order matches bbb-dmx-convert: ChannelFunction
				// Attribute/Name, then LogicalChannel, then DMXChannel, then index
				const auto logical = channel_node.getChild("LogicalChannel");
				const auto function = logical ? logical.getChild("ChannelFunction") : ofXml{};
				const auto pick_attribute = [](const ofXml &node) -> std::string {
					if(!node) {
						return "";
					}
					std::string value{node.getAttribute("Attribute").getValue()};
					if(value.empty()) {
						value = node.getAttribute("Name").getValue();
					}
					return value;
				};
				std::string attribute{pick_attribute(function)};
				if(attribute.empty()) {
					attribute = pick_attribute(logical);
				}
				if(attribute.empty()) {
					attribute = pick_attribute(channel_node);
				}
				if(attribute.empty()) {
					attribute = "channel" + std::to_string(channel_index);
				}
				std::string parameter_key{map_gdtf_attribute(attribute)};
				int suffix{2};
				while(used_keys.count(parameter_key) != 0) {
					parameter_key = map_gdtf_attribute(attribute) + "_" + std::to_string(suffix++);
				}
				used_keys.insert(parameter_key);

				// default value: ChannelFunction Default, fallback to DMXChannel Default
				long long default_value{0};
				int default_bytes{1};
				bool has_default{false};
				double physical_from{0.0};
				double physical_to{0.0};
				if(logical) {
					const auto function = logical.getChild("ChannelFunction");
					if(function) {
						has_default = parse_gdtf_dmx_value(
							function.getAttribute("Default").getValue(), default_value, default_bytes);
						physical_from = ofToDouble(function.getAttribute("PhysicalFrom").getValue());
						physical_to = ofToDouble(function.getAttribute("PhysicalTo").getValue());
					}
				}
				if(!has_default) {
					has_default = parse_gdtf_dmx_value(
						channel_node.getAttribute("Default").getValue(), default_value, default_bytes);
				}
				// clamp to the parameter type range required by the JSON schema
				const long long type_max{(1LL << (8 * byte_count)) - 1};
				const long long parameter_default{
					has_default
						? std::max(0LL, std::min(type_max, scale_dmx_value(default_value, default_bytes, byte_count)))
						: 0
				};

				// DMX range table from all ChannelFunctions / named ChannelSets
				// (bbb.dmx issue #7); emitted only when the channel really has modes
				std::vector<gdtf_range_entry> range_entries{};
				int function_count{0};
				int named_set_count{0};
				std::string parameter_wheel{};
				if(logical) {
					for(auto &function_node : logical.getChildren("ChannelFunction")) {
						function_count++;
						const std::string function_attribute{function_node.getAttribute("Attribute").getValue()};
						const std::string function_label{function_node.getAttribute("Name").getValue()};
						const std::string function_wheel{wheel_name_for_function(function_node)};
						if(parameter_wheel.empty() && !function_wheel.empty()) {
							parameter_wheel = function_wheel;
						}
						int function_slot{0};
						const bool function_has_slot{wheel_slot_index_for_node(function_node, function_slot)};
						long long function_from{0};
						int from_bytes{1};
						if(parse_gdtf_dmx_value(function_node.getAttribute("DMXFrom").getValue(),
						                        function_from, from_bytes)) {
							function_from = scale_dmx_value(function_from, from_bytes, byte_count);
						} else {
							function_from = 0;
						}
						function_from = std::max(0LL, std::min(type_max, function_from));

						const std::string physical_from_text{function_node.getAttribute("PhysicalFrom").getValue()};
						const std::string physical_to_text{function_node.getAttribute("PhysicalTo").getValue()};
						const double physical_from{ofToDouble(physical_from_text)};
						const double physical_to{ofToDouble(physical_to_text)};
						const bool function_has_physical{
							!physical_from_text.empty() && !physical_to_text.empty()
							&& physical_from != physical_to
						};

						struct named_set_info {
							long long from{0};
							std::string name{};
							bool has_slot{false};
							int slot{0};
						};
						std::vector<named_set_info> named_sets{};
						for(auto &set_node : function_node.getChildren("ChannelSet")) {
							const std::string set_name{set_node.getAttribute("Name").getValue()};
							if(set_name.empty()) {
								continue;
							}
							long long set_from{0};
							int set_bytes{1};
							if(!parse_gdtf_dmx_value(set_node.getAttribute("DMXFrom").getValue(),
							                         set_from, set_bytes)) {
								continue;
							}
							set_from = std::max(0LL, std::min(type_max,
								scale_dmx_value(set_from, set_bytes, byte_count)));
							named_set_info set_info{};
							set_info.from = set_from;
							set_info.name = set_name;
							set_info.has_slot = wheel_slot_index_for_node(set_node, set_info.slot);
							named_sets.push_back(set_info);
							named_set_count++;
						}

						if(named_sets.size() < 2) {
							gdtf_range_entry entry{};
							entry.from = function_from;
							entry.function = range_function_key(function_attribute,
								named_sets.empty() ? "" : named_sets.front().name);
							entry.label = named_sets.empty() ? function_label : named_sets.front().name;
							entry.wheel = function_wheel;
							entry.has_wheel_slot = function_has_slot;
							entry.wheel_slot = function_slot;
							const std::string &key = entry.function;
							entry.has_physical = function_has_physical
								&& (key == "strobe" || key == "pulse" || key == "random");
							entry.physical_from = physical_from;
							entry.physical_to = physical_to;
							range_entries.push_back(entry);
						} else {
							if(function_from < named_sets.front().from) {
								gdtf_range_entry lead{};
								lead.from = function_from;
								lead.function = range_function_key(function_attribute, "");
								lead.label = function_label;
								lead.wheel = function_wheel;
								range_entries.push_back(lead);
							}
							for(const auto &named_set : named_sets) {
								gdtf_range_entry entry{};
								entry.from = named_set.from;
								entry.function = range_function_key(function_attribute, named_set.name);
								entry.label = named_set.name;
								entry.wheel = function_wheel;
								entry.has_wheel_slot = named_set.has_slot;
								entry.wheel_slot = named_set.slot;
								range_entries.push_back(entry);
							}
						}
					}
				}
				const bool emit_ranges{(1 < function_count || 2 <= named_set_count)
				                       && 2 <= range_entries.size()};

				// channel entries (coarse first; GDTF Offset lists MSB first)
				static const std::array<const char *, 3> suffix_two{".coarse", ".fine", ""};
				static const std::array<const char *, 3> suffix_three{".coarse", ".middle", ".fine"};
				std::vector<std::string> channel_keys{};
				for(int index = 0; index < byte_count; index++) {
					std::string channel_key{parameter_key};
					if(byte_count == 2) {
						channel_key += suffix_two[(std::size_t)index];
					} else if(byte_count == 3) {
						channel_key += suffix_three[(std::size_t)index];
					}
					channel_keys.push_back(channel_key);
					const int byte_default{
						(int)((parameter_default >> (8 * (byte_count - 1 - index))) & 255)
					};
					channels_json.push_back({
						{"offset", offsets[(std::size_t)index]},
						{"key", channel_key},
						{"default", byte_default},
						{"label", attribute}
					});
					used_offsets.insert(offsets[(std::size_t)index]);
					footprint = std::max(footprint, offsets[(std::size_t)index]);
				}

				// field order matches bbb-dmx-convert: type, default, channel(s),
				// byte_order, range_degrees
				ordered_json parameter_json{};
				parameter_json["type"] = byte_count == 3 ? "u24" : (byte_count == 2 ? "u16" : "u8");
				parameter_json["default"] = parameter_default;
				if(byte_count == 1) {
					parameter_json["channel"] = channel_keys.front();
				} else {
					parameter_json["channels"] = channel_keys;
					parameter_json["byte_order"] = byte_count == 3 ? "coarsemidfine" : "coarsefine";
				}
				if(parameter_key == "pan" || parameter_key == "tilt") {
					double range{std::abs(physical_to - physical_from)};
					if(range <= 0.0) {
						range = parameter_key == "pan" ? 540.0 : 270.0;
					}
					// integral ranges as int, matching bbb-dmx-convert output
					if(range == std::floor(range)) {
						parameter_json["range_degrees"] = (long long)range;
					} else {
						parameter_json["range_degrees"] = range;
					}
				}
				if(!parameter_wheel.empty()) {
					parameter_json["wheel"] = parameter_wheel;
				}
				if(emit_ranges) {
					std::stable_sort(range_entries.begin(), range_entries.end(),
					                 [](const gdtf_range_entry &a, const gdtf_range_entry &b) {
						return a.from < b.from;
					});
					ordered_json ranges_json(ordered_json::value_t::array);
					for(std::size_t index = 0; index < range_entries.size(); index++) {
						const auto &entry = range_entries[index];
						const long long entry_to{
							index + 1 < range_entries.size()
								? range_entries[index + 1].from - 1
								: type_max
						};
						if(entry_to < entry.from) {
							continue;  // duplicate boundary, zero-width
						}
						ordered_json range_json{};
						range_json["from"] = entry.from;
						range_json["to"] = entry_to;
						range_json["function"] = entry.function;
						if(!entry.label.empty()) {
							range_json["label"] = entry.label;
						}
						if(!entry.wheel.empty()) {
							range_json["wheel"] = entry.wheel;
						}
						if(entry.has_wheel_slot) {
							range_json["wheel_slot"] = entry.wheel_slot;
						}
						if(entry.has_physical) {
							// integral values as int, matching bbb-dmx-convert output
							const auto physical_number = [](double value) -> ordered_json {
								if(value == std::floor(value)) {
									return (long long)value;
								}
								return value;
							};
							range_json["physical_from"] = physical_number(entry.physical_from);
							range_json["physical_to"] = physical_number(entry.physical_to);
						}
						ranges_json.push_back(range_json);
					}
					if(2 <= ranges_json.size()) {
						parameter_json["ranges"] = ranges_json;
					}
				}
				parameters_json[parameter_key] = parameter_json;
			}

			if(channels_json.empty()) {
				result.warnings.push_back("mode without DMX channels skipped: " + mode_name);
				continue;
			}

			// keep channels sorted by offset for readability
			std::sort(channels_json.begin(), channels_json.end(),
			          [](const ordered_json &a, const ordered_json &b) {
				return a["offset"].get<int>() < b["offset"].get<int>();
			});

			ordered_json mode_json{};
			mode_json["label"] = mode_name;
			mode_json["footprint"] = footprint;
			mode_json["channels"] = channels_json;
			mode_json["parameters"] = parameters_json;
			modes_json[mode_key] = mode_json;
			result.modes.push_back(gdtf_mode_info{mode_name, mode_key});
		}

		if(modes_json.empty()) {
			result.error = "no usable DMX mode found";
			return result;
		}
		profile_json["modes"] = modes_json;

		// optional photometry from the first <Beam> geometry node, emitted after
		// modes to match bbb-dmx-convert output (bbb.dmx issue #5)
		const auto beam_node = xml.findFirst("//Beam");
		if(beam_node) {
			ordered_json photometry_json(ordered_json::value_t::object);
			const auto emit_positive = [&](const char *gdtf_attribute, const char *json_key) {
				const std::string text{beam_node.getAttribute(gdtf_attribute).getValue()};
				if(!text.empty()) {
					const double value{ofToDouble(text)};
					if(0.0 < value) {
						// integral values as int, matching bbb-dmx-convert output
						if(value == std::floor(value)) {
							photometry_json[json_key] = (long long)value;
						} else {
							photometry_json[json_key] = value;
						}
					}
				}
			};
			emit_positive("BeamAngle", "beam_angle_degrees");
			emit_positive("FieldAngle", "field_angle_degrees");
			emit_positive("BeamRadius", "beam_radius");
			emit_positive("LuminousFlux", "luminous_flux");
			emit_positive("ColorTemperature", "color_temperature");
			if(!photometry_json.empty()) {
				profile_json["photometry"] = photometry_json;
			}
		}

		// optional wheels[] from GDTF <Wheels> (bbb-dmx color wheel metadata),
		// emitted after photometry to match bbb-dmx-utils output. NB: copy-init,
		// not brace-init -- ordered_json{array} would double-wrap the array.
		const ordered_json wheels_json = gdtf_wheels(fixture_type);
		if(!wheels_json.empty()) {
			profile_json["wheels"] = wheels_json;
		}

		ofDirectory::createDirectory(fixtures_directory, false, true);
		const std::string output_path{
			ofFilePath::join(fixtures_directory, result.profile_key + ".json")
		};
		write_text_file(output_path, profile_json.dump(2) + "\n");

		// sanity: the written profile must load back through bbb.dmx
		bbb::dmx::fixture_profile written{};
		const auto reload = bbb::dmx::read_fixture_profile_file(output_path, written);
		if(!reload.ok) {
			result.error = "written profile failed bbb.dmx validation: " + reload.message;
			return result;
		}

		result.output_path = output_path;
		result.ok = true;
		return result;
	}

	inline gdtf_import_result import_gdtf_file(const std::string &path,
	                                           const std::string &fixtures_directory)
	{
		const ofBuffer buffer{ofBufferFromFile(path, true)};
		if(buffer.size() == 0) {
			gdtf_import_result result{};
			result.error = "cannot read file: " + path;
			return result;
		}
		const std::vector<std::uint8_t> bytes{buffer.getData(), buffer.getData() + buffer.size()};
		return import_gdtf_bytes(bytes, fixtures_directory);
	}

	// ------------------------------------------------------------------
	// MVR importer: GeneralSceneDescription.xml (+ embedded GDTF files)
	//   -> bbb.dmx.patch.v1 JSON
	// ------------------------------------------------------------------

	struct mvr_import_result {
		bool ok{false};
		std::string error{};
		std::string patch_path{};      // absolute path of the written patch
		std::string patch_data_path{}; // path relative to data/ (for settings)
		int fixture_count{0};
		std::vector<std::string> warnings{};
	};

	// MVR matrix: "{ux,uy,uz}{vx,vy,vz}{wx,wy,wz}{ox,oy,oz}", offset in millimeters
	inline bool parse_mvr_matrix(const std::string &text, bbb::dmx::mat3 &rotation, bbb::dmx::vec3 &position) {
		std::vector<double> values{};
		const char *cursor{text.c_str()};
		while(*cursor != '\0') {
			if(std::isdigit((unsigned char)*cursor) || *cursor == '-' || *cursor == '+' || *cursor == '.') {
				char *end{nullptr};
				values.push_back(std::strtod(cursor, &end));
				cursor = end;
			} else {
				cursor++;
			}
		}
		if(values.size() != 12) {
			return false;
		}
		// columns u, v, w = images of the x, y, z axes
		for(int column = 0; column < 3; column++) {
			const double x{values[(std::size_t)(column * 3 + 0)]};
			const double y{values[(std::size_t)(column * 3 + 1)]};
			const double z{values[(std::size_t)(column * 3 + 2)]};
			const double length{std::sqrt(x * x + y * y + z * z)};
			const double scale{length < 1.0e-9 ? 1.0 : 1.0 / length};
			rotation.m[0][column] = x * scale;
			rotation.m[1][column] = y * scale;
			rotation.m[2][column] = z * scale;
		}
		position = bbb::dmx::vec3{values[9] / 1000.0, values[10] / 1000.0, values[11] / 1000.0};
		return true;
	}

	// decompose rotation matrix as R = Rz(rz) * Ry(ry) * Rx(rx), angles in degrees
	inline bbb::dmx::vec3 rotation_to_euler_zyx(const bbb::dmx::mat3 &rotation) {
		const double m20{rotation.m[2][0]};
		double rx{0.0};
		double ry{0.0};
		double rz{0.0};
		if(std::abs(m20) < 0.999999) {
			ry = std::asin(-m20);
			rx = std::atan2(rotation.m[2][1], rotation.m[2][2]);
			rz = std::atan2(rotation.m[1][0], rotation.m[0][0]);
		} else {
			// gimbal lock: pin rz to 0
			ry = m20 < 0.0 ? bbb::dmx::pi / 2.0 : -bbb::dmx::pi / 2.0;
			rz = 0.0;
			rx = std::atan2(-rotation.m[1][2], rotation.m[1][1]);
		}
		return bbb::dmx::vec3{
			bbb::dmx::radians_to_degrees(rx),
			bbb::dmx::radians_to_degrees(ry),
			bbb::dmx::radians_to_degrees(rz)
		};
	}

	inline mvr_import_result import_mvr_file(const std::string &path,
	                                         const std::string &fixtures_directory,
	                                         const std::string &patches_directory)
	{
		mvr_import_result result{};
		const ofBuffer buffer{ofBufferFromFile(path, true)};
		if(buffer.size() == 0) {
			result.error = "cannot read file: " + path;
			return result;
		}
		const std::vector<std::uint8_t> bytes{buffer.getData(), buffer.getData() + buffer.size()};

		mz_zip_archive zip{};
		if(!mz_zip_reader_init_mem(&zip, bytes.data(), bytes.size(), 0)) {
			result.error = "not a zip archive";
			return result;
		}
		const int scene_index{zip_find_entry(zip, "generalscenedescription.xml")};
		std::vector<std::uint8_t> xml_bytes{};
		if(scene_index < 0 || !zip_extract_entry(zip, scene_index, xml_bytes)) {
			mz_zip_reader_end(&zip);
			result.error = "GeneralSceneDescription.xml not found in MVR archive";
			return result;
		}

		ofXml xml{};
		if(!xml.parse(std::string{xml_bytes.begin(), xml_bytes.end()})) {
			mz_zip_reader_end(&zip);
			result.error = "failed to parse GeneralSceneDescription.xml";
			return result;
		}

		// import every referenced GDTF embedded in the archive
		std::map<std::string, gdtf_import_result> profiles_by_spec{};
		auto fixture_nodes = xml.find("//Fixture");
		for(auto &fixture_node : fixture_nodes) {
			const std::string spec{fixture_node.getChild("GDTFSpec").getValue()};
			if(spec.empty() || profiles_by_spec.count(spec) != 0) {
				continue;
			}
			std::string entry_lower{to_lower(spec)};
			int entry_index{zip_find_entry(zip, entry_lower)};
			if(entry_index < 0 && entry_lower.rfind(".gdtf") == std::string::npos) {
				entry_index = zip_find_entry(zip, entry_lower + ".gdtf");
			}
			gdtf_import_result profile_result{};
			if(entry_index < 0) {
				profile_result.error = "GDTF not embedded in MVR: " + spec;
			} else {
				std::vector<std::uint8_t> gdtf_bytes{};
				if(zip_extract_entry(zip, entry_index, gdtf_bytes)) {
					profile_result = import_gdtf_bytes(gdtf_bytes, fixtures_directory);
				} else {
					profile_result.error = "failed to extract: " + spec;
				}
			}
			if(!profile_result.ok) {
				result.warnings.push_back(spec + ": " + profile_result.error);
			}
			for(const auto &warning : profile_result.warnings) {
				result.warnings.push_back(spec + ": " + warning);
			}
			profiles_by_spec[spec] = profile_result;
		}
		mz_zip_reader_end(&zip);

		ordered_json patch_json{};
		patch_json["schema"] = "bbb.dmx.patch.v2";
		patch_json["coordinates"] = "gdtf";
		ordered_json profiles_json(ordered_json::value_t::array);
		std::set<std::string> profile_paths{};
		ordered_json fixtures_json(ordered_json::value_t::array);
		std::set<std::string> used_ids{};

		int fixture_index{0};
		for(auto &fixture_node : fixture_nodes) {
			fixture_index++;
			const std::string fixture_name{fixture_node.getAttribute("name").getValue()};
			const std::string spec{fixture_node.getChild("GDTFSpec").getValue()};
			const auto profile_found = profiles_by_spec.find(spec);
			if(profile_found == profiles_by_spec.end() || !profile_found->second.ok) {
				result.warnings.push_back("fixture skipped (no profile): " + fixture_name);
				continue;
			}
			const auto &profile = profile_found->second;

			const std::string mode_name{fixture_node.getChild("GDTFMode").getValue()};
			std::string mode_key{};
			for(const auto &mode : profile.modes) {
				if(mode.label == mode_name) {
					mode_key = mode.key;
					break;
				}
			}
			if(mode_key.empty()) {
				mode_key = sanitize_key(mode_name);
				bool known{false};
				for(const auto &mode : profile.modes) {
					if(mode.key == mode_key) {
						known = true;
						break;
					}
				}
				if(!known) {
					if(profile.modes.empty()) {
						result.warnings.push_back("fixture skipped (no mode): " + fixture_name);
						continue;
					}
					result.warnings.push_back(fixture_name + ": mode '" + mode_name
					                          + "' not found, using '" + profile.modes.front().label + "'");
					mode_key = profile.modes.front().key;
				}
			}

			const auto address_node = fixture_node.findFirst("Addresses/Address");
			if(!address_node) {
				result.warnings.push_back("fixture skipped (no address): " + fixture_name);
				continue;
			}
			const long long absolute_address{(long long)ofToDouble(address_node.getValue())};
			if(absolute_address < 1) {
				result.warnings.push_back("fixture skipped (bad address): " + fixture_name);
				continue;
			}
			const int universe{(int)((absolute_address - 1) / 512) + 1};
			const int address{(int)((absolute_address - 1) % 512) + 1};

			bbb::dmx::mat3 rotation_matrix{};
			bbb::dmx::vec3 position{};
			bbb::dmx::vec3 rotation_degrees{};
			bool has_transform{false};
			const std::string matrix_text{fixture_node.getChild("Matrix").getValue()};
			if(!matrix_text.empty() && parse_mvr_matrix(matrix_text, rotation_matrix, position)) {
				rotation_degrees = rotation_to_euler_zyx(rotation_matrix);
				has_transform = true;
			}

			// id source priority matches bbb-dmx-convert: FixtureID, UnitNumber,
			// Name, UUID (XML attribute or text-only child element), then index
			const auto fixture_field = [&fixture_node](std::initializer_list<const char *> names) -> std::string {
				for(const char *field_name : names) {
					std::string value{fixture_node.getAttribute(field_name).getValue()};
					if(!value.empty()) {
						return value;
					}
					const auto child = fixture_node.getChild(field_name);
					if(child) {
						value = child.getValue();
						if(!value.empty()) {
							return value;
						}
					}
				}
				return "";
			};
			std::string id_source{fixture_field({"FixtureID", "fixtureID", "UnitNumber", "Name", "name", "UUID", "uuid"})};
			if(id_source.empty()) {
				id_source = "fixture_" + std::to_string(fixture_index);
			}
			std::string id{sanitize_identifier(id_source)};
			int suffix{2};
			while(used_ids.count(id) != 0) {
				id = sanitize_identifier(id_source) + "_" + std::to_string(suffix++);
			}
			used_ids.insert(id);

			const std::string profile_relative{"../fixtures/" + profile.profile_key + ".json"};
			if(profile_paths.insert(profile_relative).second) {
				profiles_json.push_back(profile_relative);
			}

			ordered_json fixture_json{};
			fixture_json["id"] = id;
			fixture_json["profile"] = profile.profile_key;
			fixture_json["mode"] = mode_key;
			fixture_json["universe"] = universe;
			fixture_json["address"] = address;
			// only when a Matrix exists, integral coordinates as int (also folds
			// -0.0 to 0) — both matching the CLI output
			if(has_transform) {
				const auto json_number = [](double value) -> ordered_json {
					if(value == std::floor(value)) {
						return (long long)value;
					}
					return value;
				};
				fixture_json["position"] = {json_number(position.x), json_number(position.y), json_number(position.z)};
				fixture_json["rotation"] = {json_number(rotation_degrees.x), json_number(rotation_degrees.y), json_number(rotation_degrees.z)};
			}
			fixtures_json.push_back(fixture_json);
			result.fixture_count++;
		}

		if(fixtures_json.empty()) {
			result.error = "no usable fixture found in MVR";
			return result;
		}
		patch_json["profiles"] = profiles_json;
		patch_json["fixtures"] = fixtures_json;

		ofDirectory::createDirectory(patches_directory, false, true);
		const std::string stem{sanitize_identifier(ofFilePath::getBaseName(path))};
		const std::string output_path{ofFilePath::join(patches_directory, stem + ".json")};
		write_text_file(output_path, patch_json.dump(2) + "\n");

		result.patch_path = output_path;
		result.patch_data_path = "patches/" + stem + ".json";
		result.ok = true;
		return result;
	}

}}} // namespace bbb::dmx::ofx
