// Compositing (COP) helpers shared by the nodes, the command line tool and the tests.
#pragma once
#include "Geometry.h"
#include <string>

namespace hd {

// Writes an image layer: .exr keeps linear RGBA (+ Z when present); .png is converted to sRGB with
// straight (unpremultiplied) alpha. "$F" / "$F4" in the path are replaced by the frame number.
bool writeImageLayer(const std::string& path, const ImageLayer& layer, float frame, std::string* error = nullptr);

// Replaces $F, $F2 .. $F6 with the (zero padded) frame number.
std::string expandFrame(const std::string& path, float frame);

} // namespace hd
