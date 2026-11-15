// Python scripting for HD.
//
// HD has no build-time dependency on Python. At runtime it finds the user's Python 3 (3.8 or newer),
// loads its shared library and talks to it only through functions of Python's stable ABI, resolved
// by name. So one HD build works with whatever Python 3 is installed, on every platform.
//
// Which Python: the HD_PYTHON environment variable (path to a python executable), otherwise
// `python3` / `python` (Windows: also `py -3`) on PATH. The interpreter is asked for its shared
// library, home and sys.path. A Python built without a shared library (static only) can't be
// embedded; the error message says so.
//
// Scripts see the `hd` module (nodes, parameters, wiring, cooking, geometry, frames, files);
// see docs in the module itself: `import hd; help(hd)`.
#pragma once
#include <string>
#include <vector>

namespace hd {
struct NodeType;
class Graph;
class Geometry;
class Node;

namespace python {

struct Info {
    bool ok = false;
    std::string error;      // why Python is unavailable
    std::string version;    // e.g. "3.12.3"
    std::string executable; // the interpreter that was queried
    std::string library;    // the shared library that was loaded
};

// Finds and starts Python once (later calls return the cached result). Main thread only.
const Info& ensure();
// True once ensure() has succeeded.
bool available();

// Runs code with `hd` bound to graph g. `shell` keeps variables between calls (one shared
// namespace) and prints the value of a lone expression, like the interactive prompt. Output
// (stdout and stderr, including tracebacks) is appended to `output`. Returns false if Python is
// unavailable or the code raised.
bool run(Graph& g, const std::string& code, std::string& output, const std::string& filename = "<script>", bool shell = false);

// Python SOP: runs code with `geo` (the writable output, starting as a copy of input 1),
// `inputs` (read-only geometry of every input) and `node`. Returns false and sets `error`
// (the last traceback line) on failure; `output` gets everything printed.
bool runGeometry(Graph& g, Node& node, const std::string& code, const std::vector<const Geometry*>& inputs, Geometry& out,
                 std::string& output, std::string& error);

// The Python source of the `hd` module (for documentation and tests).
const char* moduleSource();

// Text printed by Python SOPs since the last call ("[node] ..." lines).
std::string takeLog();

// Registers the Python SOP node type.
void registerNodes(std::vector<NodeType>& types);

} // namespace python
} // namespace hd
