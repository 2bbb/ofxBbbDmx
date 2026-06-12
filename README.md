# ofxBbbDmx

openFrameworks (v0.12.1+) integration layer for
[bbb-dmx](https://github.com/2bbb/bbb-dmx) /
[bbb-artnet](https://github.com/2bbb/bbb-artnet), header-only:

- **Art-Net receive**: `bbb::dmx::ofx::artnet_receiver` wrapping
  `bbb::artnet::managed_node` (standalone asio), per-universe buffers + stats.
- **Show loading**: `bbb.dmx.fixture.profile.v1` / `bbb.dmx.patch.v2` JSON via
  the bbb-dmx C++ layer, resolved fixture views.
- **Parameter evaluation**: u8/u16/u24 sampling mirroring `fixture_mapper`
  layout, shutter `ranges` tables (closed/open/strobe/pulse/random),
  semantic fixture state (color mixing RGB/CMY/W/A/UV, GDTF
  device-orientation beam direction, photometry beam angles).
- **GDTF / MVR import**: emits bbb.dmx JSON byte-identical to
  `bbb-dmx-utils` (profiles with photometry/ranges, patch.v2 with
  `coordinates: "gdtf"`, MVR Matrix -> position/rotation).

## Usage

Header-only. In exactly one translation unit:

```cpp
#define OFX_BBB_DMX_IMPLEMENTATION  // embeds miniz
#include "ofxBbbDmx.h"

namespace lsim = bbb::dmx::ofx;
```

Clone with submodules (`git clone --recursive`): dependencies live under
`libs/` (bbb-artnet, standalone asio, bbb-dmx, vendored miniz with a one-line
C++ compilation patch).

The coordinate model is the GDTF (DIN SPEC 15800) device-orientation
convention adopted by bbb.dmx patch.v2: identity rotation = hanging device,
beam rest = device -Z, pan about +Z, tilt about +X.
