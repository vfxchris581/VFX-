// Python scripting: runtime loading of the user's Python (stable ABI only), the `_hd` C module and
// the `hd` Python module. See Python.h.
#include "core/Python.h"
#include "core/Json.h"
#include "core/Node.h"
#include <algorithm>
#include <cstdarg>
#include <functional>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

#if defined(_WIN32)
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#else
#  include <dlfcn.h>
#endif

namespace fs = std::filesystem;

namespace hd {
namespace python {
namespace {

// ------------------------------------------------------------------ the stable-ABI surface we use
struct PyObject;
using Py_ssize_t = std::ptrdiff_t;
using PyCFunction = PyObject* (*)(PyObject*, PyObject*);
struct PyMethodDef {
    const char* ml_name;
    PyCFunction ml_meth;
    int ml_flags;
    const char* ml_doc;
};
constexpr int METH_VARARGS = 0x0001;
constexpr int Py_file_input = 257;

struct Api {
    void (*Py_InitializeEx)(int);
    int (*Py_IsInitialized)();
    void* (*PyEval_SaveThread)();
    int (*PyGILState_Ensure)();
    void (*PyGILState_Release)(int);
    void (*Py_IncRef)(PyObject*);
    void (*Py_DecRef)(PyObject*);
    PyObject* (*PyImport_ImportModule)(const char*);
    PyObject* (*PyImport_AddModule)(const char*);
    PyObject* (*PyImport_GetModuleDict)();
    PyObject* (*PyModule_New)(const char*);
    PyObject* (*PyModule_GetDict)(PyObject*);
    PyObject* (*PyCFunction_NewEx)(PyMethodDef*, PyObject*, PyObject*);
    PyObject* (*PyDict_New)();
    int (*PyDict_SetItemString)(PyObject*, const char*, PyObject*);
    PyObject* (*PyDict_GetItemString)(PyObject*, const char*);
    PyObject* (*Py_CompileString)(const char*, const char*, int);
    PyObject* (*PyEval_EvalCode)(PyObject*, PyObject*, PyObject*);
    PyObject* (*PyObject_CallObject)(PyObject*, PyObject*);
    PyObject* (*PyErr_Occurred)();
    void (*PyErr_Clear)();
    void (*PyErr_SetString)(PyObject*, const char*);
    void (*PyErr_Print)();
    int (*PyArg_ParseTuple)(PyObject*, const char*, ...);
    PyObject* (*Py_BuildValue)(const char*, ...);
    PyObject* (*PyLong_FromLong)(long);
    long (*PyLong_AsLong)(PyObject*);
    PyObject* (*PyFloat_FromDouble)(double);
    double (*PyFloat_AsDouble)(PyObject*);
    PyObject* (*PyBool_FromLong)(long);
    PyObject* (*PyUnicode_FromString)(const char*);
    PyObject* (*PyUnicode_AsUTF8String)(PyObject*);
    char* (*PyBytes_AsString)(PyObject*);
    PyObject* (*PyTuple_New)(Py_ssize_t);
    int (*PyTuple_SetItem)(PyObject*, Py_ssize_t, PyObject*);
    PyObject* (*PyList_New)(Py_ssize_t);
    int (*PyList_SetItem)(PyObject*, Py_ssize_t, PyObject*);
    int (*PyList_Append)(PyObject*, PyObject*);
    Py_ssize_t (*PySequence_Size)(PyObject*);
    PyObject* (*PySequence_GetItem)(PyObject*, Py_ssize_t);
    int (*PyObject_IsTrue)(PyObject*);
    PyObject* None = nullptr;          // &_Py_NoneStruct
    PyObject* ExcRuntimeError = nullptr; // *PyExc_RuntimeError
    PyObject* ExcValueError = nullptr;
    PyObject* ExcIndexError = nullptr;
};

Api A;
Info g_info;
std::string g_log; // output of Python SOPs (shown by the app's Python Shell, printed by hdc)
bool g_tried = false;
PyObject* g_hdModule = nullptr;  // the `hd` module
PyObject* g_shellGlobals = nullptr;

// ------------------------------------------------------------------ dynamic loading
#if defined(_WIN32)
using LibHandle = HMODULE;
LibHandle openLib(const std::string& path, std::string& err) {
    std::wstring w = fs::path(path).wstring();
    HMODULE h = LoadLibraryExW(w.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
    if (!h) err = "cannot load " + path + " (error " + std::to_string(GetLastError()) + ")";
    return h;
}
void* sym(LibHandle h, const char* name) { return reinterpret_cast<void*>(GetProcAddress(h, name)); }
#else
using LibHandle = void*;
LibHandle openLib(const std::string& path, std::string& err) {
    void* h = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (!h) { const char* e = dlerror(); err = e ? e : ("cannot load " + path); }
    return h;
}
void* sym(LibHandle h, const char* name) { return dlsym(h, name); }
#endif

// Runs a command and returns its standard output.
bool capture(const std::string& cmd, std::string& out) {
#if defined(_WIN32)
    FILE* p = _popen(("\"" + cmd + "\"").c_str(), "r"); // cmd.exe strips one pair of outer quotes
#else
    FILE* p = popen((cmd + " 2>/dev/null").c_str(), "r");
#endif
    if (!p) return false;
    char buf[4096];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, p)) > 0) out.append(buf, n);
#if defined(_WIN32)
    int rc = _pclose(p);
#else
    int rc = pclose(p);
#endif
    return rc == 0 && !out.empty();
}

const char* kQuery = R"PY(
import sys, os, json, sysconfig
v = sysconfig.get_config_var
libs = []
def add(d, n):
    if d and n and not n.endswith('.a'):
        libs.append(os.path.join(d, n))
if sys.platform == 'win32':
    name = 'python%d%d.dll' % sys.version_info[:2]
    for d in (sys.base_prefix, os.path.dirname(sys.executable), sys.prefix):
        libs.append(os.path.join(d, name))
elif sys.platform == 'darwin':
    fw, fname = v('PYTHONFRAMEWORKPREFIX'), v('PYTHONFRAMEWORK')
    if fw and fname:
        libs.append(os.path.join(fw, fname + '.framework', 'Versions', v('VERSION'), fname))
    for k in ('LDLIBRARY', 'INSTSONAME'):
        add(v('LIBDIR'), v(k))
else:
    for d in (v('LIBDIR'), os.path.join(v('LIBDIR') or '', v('MULTIARCH') or ''), v('LIBPL')):
        for k in ('INSTSONAME', 'LDLIBRARY'):
            add(d, v(k))
print(json.dumps({'version': '%d.%d.%d' % sys.version_info[:3], 'major': sys.version_info[0], 'minor': sys.version_info[1],
                  'base_prefix': sys.base_prefix, 'path': [p for p in sys.path if p], 'libs': libs, 'executable': sys.executable}))
)PY";

void setEnv(const char* k, const std::string& v) {
#if defined(_WIN32)
    _putenv_s(k, v.c_str());
#else
    setenv(k, v.c_str(), 1);
#endif
}
void unsetEnv(const char* k) {
#if defined(_WIN32)
    _putenv_s(k, "");
#else
    unsetenv(k);
#endif
}

template <class F> bool resolve(LibHandle h, F& fp, const char* name, std::string& missing) {
    fp = reinterpret_cast<F>(sym(h, name));
    if (!fp) missing += std::string(missing.empty() ? "" : ", ") + name;
    return fp != nullptr;
}

// ------------------------------------------------------------------ host context
struct GeoHandle {
    Geometry* geo = nullptr; // owned elsewhere (Python SOP output / inputs)
    Graph* graph = nullptr;  // node geometry: looked up by id on every use (the node may be gone)
    int nodeId = -1;
    bool writable = false;
};
struct Ctx {
    Graph* graph = nullptr;
    std::string* out = nullptr;
    std::vector<GeoHandle> geos;
    std::string pwd; // node running a Python SOP
};
std::vector<Ctx> g_ctx;

Ctx& ctx() { return g_ctx.back(); }
Py_ssize_t PyTupleSize(PyObject* t) { return A.PySequence_Size(t); } // argument count

PyObject* none() { A.Py_IncRef(A.None); return A.None; }
PyObject* fail(PyObject* type, const std::string& msg) { A.PyErr_SetString(type ? type : A.ExcRuntimeError, msg.c_str()); return nullptr; }
PyObject* str(const std::string& s) { return A.PyUnicode_FromString(s.c_str()); }
bool toFloat(PyObject* o, float& f) {
    double d = A.PyFloat_AsDouble(o);
    if (d == -1.0 && A.PyErr_Occurred()) return false;
    f = float(d);
    return true;
}
PyObject* tuple3(const Vec3& v) { return A.Py_BuildValue("(ddd)", double(v.x), double(v.y), double(v.z)); }
bool toVec3(PyObject* o, Vec3& v) {
    Py_ssize_t n = A.PySequence_Size(o);
    if (n < 0) { A.PyErr_Clear(); float f; if (!toFloat(o, f)) return false; v = Vec3(f); return true; }
    if (n != 3) { fail(A.ExcValueError, "expected 3 numbers"); return false; }
    for (int k = 0; k < 3; ++k) {
        PyObject* it = A.PySequence_GetItem(o, k);
        if (!it) return false;
        float f;
        bool ok = toFloat(it, f);
        A.Py_DecRef(it);
        if (!ok) return false;
        v[k] = f;
    }
    return true;
}

// "a/b/c": nodes inside subnets
Node* findPath(Graph& g, const std::string& path) {
    Graph* cur = &g;
    Node* n = nullptr;
    std::stringstream ss(path);
    std::string part;
    while (std::getline(ss, part, '/')) {
        if (part.empty()) continue;
        if (!cur) return nullptr;
        n = cur->findNode(part);
        if (!n) return nullptr;
        cur = n->subnet();
    }
    return n;
}
Graph* graphOf(Graph& root, const std::string& path, std::string& leaf) {
    size_t s = path.find_last_of('/');
    leaf = s == std::string::npos ? path : path.substr(s + 1);
    if (s == std::string::npos || s == 0) return &root;
    Node* parent = findPath(root, path.substr(0, s));
    return parent ? parent->subnet() : nullptr;
}
std::string pathOf(Graph& root, const Graph* g, const Node* n) {
    std::string p = n->name();
    while (g && g != &root && g->owner()) {
        p = g->owner()->name() + "/" + p;
        g = g->parent();
    }
    return p;
}
Node* nodeArg(const char* path) {
    Node* n = findPath(*ctx().graph, path);
    if (!n) fail(A.ExcValueError, std::string("no node '") + path + "'");
    return n;
}
Parm* parmArg(Node* n, const char* name) {
    Parm* p = n->parm(name);
    if (!p) fail(A.ExcValueError, "node '" + n->name() + "' has no parameter '" + name + "'");
    return p;
}

// ------------------------------------------------------------------ _hd functions
#define HD_FN(name) PyObject* name(PyObject*, PyObject* args)

HD_FN(f_write) {
    const char* s;
    if (!A.PyArg_ParseTuple(args, "s", &s)) return nullptr;
    if (!g_ctx.empty() && ctx().out) *ctx().out += s;
    return none();
}
HD_FN(f_nodes) {
    const char* parent = "";
    if (!A.PyArg_ParseTuple(args, "|s", &parent)) return nullptr;
    Graph* g = ctx().graph;
    if (parent && *parent) {
        Node* p = nodeArg(parent);
        if (!p) return nullptr;
        if (!p->subnet()) return fail(A.ExcValueError, std::string("'") + parent + "' is not a subnet");
        g = p->subnet();
    }
    PyObject* l = A.PyList_New(0);
    for (auto& n : g->nodes()) { PyObject* s = str(pathOf(*ctx().graph, g, n.get())); A.PyList_Append(l, s); A.Py_DecRef(s); }
    return l;
}
HD_FN(f_exists) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    return A.PyBool_FromLong(findPath(*ctx().graph, p) != nullptr);
}
HD_FN(f_node_type) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    return A.Py_BuildValue("(sss)", n->type().name.c_str(), n->type().label.c_str(), n->type().category.c_str());
}
HD_FN(f_node_types) {
    PyObject* l = A.PyList_New(0);
    for (auto& t : nodeTypes()) {
        PyObject* e = A.Py_BuildValue("(ssss)", t.name.c_str(), t.label.c_str(), t.category.c_str(), t.description.c_str());
        A.PyList_Append(l, e);
        A.Py_DecRef(e);
    }
    return l;
}
HD_FN(f_create) {
    const char* type; const char* name = ""; const char* parent = "";
    if (!A.PyArg_ParseTuple(args, "s|ss", &type, &name, &parent)) return nullptr;
    if (!findNodeType(type)) return fail(A.ExcValueError, std::string("unknown node type '") + type + "' (see hd.nodeTypes())");
    Graph* g = ctx().graph;
    if (parent && *parent) {
        Node* p = nodeArg(parent);
        if (!p) return nullptr;
        if (!p->subnet()) return fail(A.ExcValueError, std::string("'") + parent + "' is not a subnet");
        g = p->subnet();
    }
    // place new nodes below the existing ones
    float y = 0;
    for (auto& n : g->nodes()) y = std::max(y, n->y + 80);
    Node* n = g->createNode(type, 0, y, name ? name : "");
    if (!n) return fail(nullptr, std::string("could not create '") + type + "'");
    return str(pathOf(*ctx().graph, g, n));
}
HD_FN(f_delete) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, p, leaf);
    if (!g) return fail(nullptr, "bad path");
    g->removeNode(n);
    return none();
}
HD_FN(f_rename) {
    const char* p; const char* nn;
    if (!A.PyArg_ParseTuple(args, "ss", &p, &nn)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, p, leaf);
    if (!g || !g->rename(n, nn)) return fail(A.ExcValueError, std::string("cannot rename to '") + nn + "' (name taken or invalid)");
    return str(pathOf(*ctx().graph, g, n));
}
HD_FN(f_connect) {
    const char* src; const char* dst; int idx = 0;
    if (!A.PyArg_ParseTuple(args, "ssi", &src, &dst, &idx)) return nullptr;
    Node* a = nodeArg(src);
    if (!a) return nullptr;
    Node* b = nodeArg(dst);
    if (!b) return nullptr;
    std::string la, lb;
    Graph* ga = graphOf(*ctx().graph, src, la);
    Graph* gb = graphOf(*ctx().graph, dst, lb);
    if (ga != gb) return fail(A.ExcValueError, "nodes are in different networks");
    if (idx < 0 || idx >= b->type().maxInputs) return fail(A.ExcIndexError, "input " + std::to_string(idx) + " out of range for " + b->type().label);
    std::string err;
    if (!ga->connect(a, b, idx, &err)) return fail(A.ExcValueError, err.empty() ? "cannot connect" : err);
    return none();
}
HD_FN(f_disconnect) {
    const char* dst; int idx;
    if (!A.PyArg_ParseTuple(args, "si", &dst, &idx)) return nullptr;
    Node* b = nodeArg(dst);
    if (!b) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, dst, leaf);
    if (idx < 0 || idx >= int(b->inputs.size())) return fail(A.ExcIndexError, "input out of range");
    g->disconnect(b, idx);
    return none();
}
HD_FN(f_inputs) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, p, leaf);
    PyObject* l = A.PyList_New(0);
    for (Node* in : n->inputs) {
        PyObject* e = in ? str(pathOf(*ctx().graph, g, in)) : none();
        A.PyList_Append(l, e);
        A.Py_DecRef(e);
    }
    return l;
}
HD_FN(f_outputs) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, p, leaf);
    PyObject* l = A.PyList_New(0);
    for (auto& [c, i] : g->outputsOf(n)) {
        PyObject* e = A.Py_BuildValue("(Ni)", str(pathOf(*ctx().graph, g, c)), i);
        A.PyList_Append(l, e);
        A.Py_DecRef(e);
    }
    return l;
}
HD_FN(f_display) {
    const char* p = nullptr;
    if (!A.PyArg_ParseTuple(args, "|s", &p)) return nullptr;
    Graph& g = *ctx().graph;
    if (p) {
        Node* n = nodeArg(p);
        if (!n) return nullptr;
        std::string leaf;
        Graph* gg = graphOf(g, p, leaf);
        gg->setDisplayNode(n);
        return none();
    }
    return g.displayNode() ? str(g.displayNode()->name()) : none();
}
HD_FN(f_position) {
    const char* p; PyObject* xy = nullptr;
    if (!A.PyArg_ParseTuple(args, "s|O", &p, &xy)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    if (xy) {
        PyObject* a = A.PySequence_GetItem(xy, 0);
        PyObject* b = A.PySequence_GetItem(xy, 1);
        float x = 0, y = 0;
        bool ok = a && b && toFloat(a, x) && toFloat(b, y);
        if (a) A.Py_DecRef(a);
        if (b) A.Py_DecRef(b);
        if (!ok) return nullptr;
        n->x = x; n->y = y;
        return none();
    }
    return A.Py_BuildValue("(dd)", double(n->x), double(n->y));
}

const char* typeName(ParmType t) {
    switch (t) {
        case ParmType::Float: return "float";
        case ParmType::Int: return "int";
        case ParmType::Toggle: return "toggle";
        case ParmType::Vec3: return "vector";
        case ParmType::String: return "string";
        case ParmType::Menu: return "menu";
        case ParmType::Code: return "code";
        case ParmType::File: return "file";
    }
    return "?";
}
bool isText(ParmType t) { return t == ParmType::String || t == ParmType::Code || t == ParmType::File; }

HD_FN(f_parms) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    PyObject* l = A.PyList_New(0);
    for (auto& pm : n->parms()) {
        PyObject* menu = A.PyList_New(0);
        for (auto& m : pm.def->menu) { PyObject* s = str(m); A.PyList_Append(menu, s); A.Py_DecRef(s); }
        PyObject* e = A.Py_BuildValue("(sssiN)", pm.def->name.c_str(), pm.def->label.c_str(), typeName(pm.def->type), pm.def->components(), menu);
        A.PyList_Append(l, e);
        A.Py_DecRef(e);
    }
    return l;
}
HD_FN(f_parm_get) {
    const char* p; const char* name;
    if (!A.PyArg_ParseTuple(args, "ss", &p, &name)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (isText(pm->def->type)) return str(pm->str);
    Graph& g = *ctx().graph;
    int nc = pm->def->components();
    PyObject* t = A.PyTuple_New(nc);
    for (int c = 0; c < nc; ++c) A.PyTuple_SetItem(t, c, A.PyFloat_FromDouble(n->evalFloat(*pm, c, g.frame, g.fps)));
    return t;
}
HD_FN(f_parm_set) {
    const char* p; const char* name; int comp; double v;
    if (!A.PyArg_ParseTuple(args, "ssid", &p, &name, &comp, &v)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (isText(pm->def->type)) return fail(A.ExcValueError, std::string("'") + name + "' is a text parameter: set it to a string");
    if (comp < 0 || comp >= pm->def->components()) return fail(A.ExcIndexError, "component out of range");
    pm->setValue(comp, float(v), ctx().graph->frame);
    pm->expr[comp].clear();
    n->touch();
    return none();
}
HD_FN(f_parm_set_str) {
    const char* p; const char* name; const char* v;
    if (!A.PyArg_ParseTuple(args, "sss", &p, &name, &v)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (!isText(pm->def->type)) {
        if (pm->def->type == ParmType::Menu) { // menus also take the item label
            const auto& items = pm->def->menu;
            for (size_t i = 0; i < items.size(); ++i)
                if (items[i] == v) { pm->val[0] = float(i); n->touch(); return none(); }
            return fail(A.ExcValueError, std::string("'") + v + "' is not an item of '" + name + "'");
        }
        return fail(A.ExcValueError, std::string("'") + name + "' is numeric: set a number, or use setExpression()");
    }
    pm->str = v;
    n->touch();
    return none();
}
HD_FN(f_parm_expr) {
    const char* p; const char* name; int comp; const char* e = nullptr;
    if (!A.PyArg_ParseTuple(args, "ssi|z", &p, &name, &comp, &e)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (isText(pm->def->type) || comp < 0 || comp >= pm->def->components()) return fail(A.ExcIndexError, "not a numeric component");
    if (PyTupleSize(args) >= 4) { pm->expr[comp] = e ? e : ""; n->touch(); return none(); }
    return str(pm->expr[comp]);
}
HD_FN(f_parm_key) {
    const char* p; const char* name; int comp; double frame, value;
    if (!A.PyArg_ParseTuple(args, "ssidd", &p, &name, &comp, &frame, &value)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (isText(pm->def->type) || comp < 0 || comp >= pm->def->components()) return fail(A.ExcIndexError, "not a numeric component");
    pm->expr[comp].clear();
    pm->setKey(comp, float(frame), float(value));
    n->touch();
    return none();
}
HD_FN(f_parm_clear_keys) {
    const char* p; const char* name; int comp;
    if (!A.PyArg_ParseTuple(args, "ssi", &p, &name, &comp)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (comp < 0 || comp >= 3) return fail(A.ExcIndexError, "component out of range");
    pm->keys[comp].clear();
    n->touch();
    return none();
}
HD_FN(f_parm_keys) {
    const char* p; const char* name; int comp;
    if (!A.PyArg_ParseTuple(args, "ssi", &p, &name, &comp)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    Parm* pm = parmArg(n, name);
    if (!pm) return nullptr;
    if (comp < 0 || comp >= 3) return fail(A.ExcIndexError, "component out of range");
    PyObject* l = A.PyList_New(0);
    for (auto& k : pm->keys[comp]) { PyObject* e = A.Py_BuildValue("(dd)", double(k.frame), double(k.value)); A.PyList_Append(l, e); A.Py_DecRef(e); }
    return l;
}
HD_FN(f_cook) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, p, leaf);
    g->frame = ctx().graph->frame;
    g->fps = ctx().graph->fps;
    g->cook(n);
    return A.Py_BuildValue("(ssd)", n->error().c_str(), n->warning().c_str(), n->cookTimeMs());
}
HD_FN(f_frame) {
    PyObject* f = nullptr;
    if (!A.PyArg_ParseTuple(args, "|O", &f)) return nullptr;
    Graph& g = *ctx().graph;
    if (f) { float v; if (!toFloat(f, v)) return nullptr; g.frame = v; return none(); }
    return A.PyFloat_FromDouble(g.frame);
}
HD_FN(f_fps) {
    PyObject* f = nullptr;
    if (!A.PyArg_ParseTuple(args, "|O", &f)) return nullptr;
    Graph& g = *ctx().graph;
    if (f) { float v; if (!toFloat(f, v)) return nullptr; if (v <= 0) return fail(A.ExcValueError, "fps must be positive"); g.fps = v; return none(); }
    return A.PyFloat_FromDouble(g.fps);
}
HD_FN(f_range) {
    int a = 0, b = 0, set = 0;
    if (!A.PyArg_ParseTuple(args, "|ii", &a, &b)) return nullptr;
    set = int(PyTupleSize(args));
    Graph& g = *ctx().graph;
    if (set == 2) { if (b < a) return fail(A.ExcValueError, "end before start"); g.frameStart = a; g.frameEnd = b; return none(); }
    return A.Py_BuildValue("(ii)", g.frameStart, g.frameEnd);
}
HD_FN(f_save) {
    const char* path;
    if (!A.PyArg_ParseTuple(args, "s", &path)) return nullptr;
    std::string err;
    if (!ctx().graph->save(path, &err)) return fail(nullptr, err);
    return none();
}
HD_FN(f_load) {
    const char* path;
    if (!A.PyArg_ParseTuple(args, "s", &path)) return nullptr;
    std::string err;
    if (!ctx().graph->load(path, &err)) return fail(nullptr, err.empty() ? std::string("cannot load ") + path : err);
    return none();
}
HD_FN(f_clear) {
    if (!A.PyArg_ParseTuple(args, "")) return nullptr;
    ctx().graph->clear();
    return none();
}
HD_FN(f_pwd) {
    if (!A.PyArg_ParseTuple(args, "")) return nullptr;
    return ctx().pwd.empty() ? none() : str(ctx().pwd);
}

// ---- geometry
Geometry* geoArg(long h, bool write) {
    auto& gs = ctx().geos;
    if (h < 0 || size_t(h) >= gs.size()) { fail(A.ExcValueError, "invalid geometry handle"); return nullptr; }
    GeoHandle& e = gs[size_t(h)];
    if (write && !e.writable) { fail(nullptr, "this geometry is read-only (only a Python SOP's own geo can be changed)"); return nullptr; }
    if (e.geo) return e.geo;
    Node* n = e.graph ? e.graph->findNode(e.nodeId) : nullptr;
    if (!n) { fail(nullptr, "the node of this geometry no longer exists"); return nullptr; }
    return const_cast<Geometry*>(&n->geometry());
}
AttribClass classArg(const char* c) { return std::string(c) == "prim" ? AttribClass::Prim : AttribClass::Point; }

HD_FN(f_node_geo) {
    const char* p;
    if (!A.PyArg_ParseTuple(args, "s", &p)) return nullptr;
    Node* n = nodeArg(p);
    if (!n) return nullptr;
    std::string leaf;
    Graph* g = graphOf(*ctx().graph, p, leaf);
    g->frame = ctx().graph->frame;
    g->cook(n);
    GeoHandle h;
    h.graph = g; h.nodeId = n->id();
    ctx().geos.push_back(h);
    return A.PyLong_FromLong(long(ctx().geos.size() - 1));
}
HD_FN(f_g_info) {
    long h;
    if (!A.PyArg_ParseTuple(args, "l", &h)) return nullptr;
    Geometry* g = geoArg(h, false);
    if (!g) return nullptr;
    return A.Py_BuildValue("(lllllli)", long(g->numPoints()), long(g->numPrims()), long(g->numVertices()), long(g->numPacked()),
                           long(g->volumes.size()), long(g->images.size()), int(ctx().geos[size_t(h)].writable));
}
HD_FN(f_g_points) {
    long h;
    if (!A.PyArg_ParseTuple(args, "l", &h)) return nullptr;
    Geometry* g = geoArg(h, false);
    if (!g) return nullptr;
    PyObject* l = A.PyList_New(Py_ssize_t(g->numPoints()));
    for (size_t i = 0; i < g->numPoints(); ++i) A.PyList_SetItem(l, Py_ssize_t(i), tuple3(g->P[i]));
    return l;
}
HD_FN(f_g_set_points) {
    long h; PyObject* seq;
    if (!A.PyArg_ParseTuple(args, "lO", &h, &seq)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    Py_ssize_t n = A.PySequence_Size(seq);
    if (n < 0) return nullptr;
    if (size_t(n) != g->numPoints()) return fail(A.ExcValueError, "expected " + std::to_string(g->numPoints()) + " positions, got " + std::to_string(n));
    for (Py_ssize_t i = 0; i < n; ++i) {
        PyObject* it = A.PySequence_GetItem(seq, i);
        if (!it) return nullptr;
        bool ok = toVec3(it, g->P[size_t(i)]);
        A.Py_DecRef(it);
        if (!ok) return nullptr;
    }
    return none();
}
HD_FN(f_g_add_points) {
    long h; PyObject* seq;
    if (!A.PyArg_ParseTuple(args, "lO", &h, &seq)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    Py_ssize_t n = A.PySequence_Size(seq);
    if (n < 0) return nullptr;
    long first = long(g->numPoints());
    for (Py_ssize_t i = 0; i < n; ++i) {
        PyObject* it = A.PySequence_GetItem(seq, i);
        if (!it) return nullptr;
        Vec3 v;
        bool ok = toVec3(it, v);
        A.Py_DecRef(it);
        if (!ok) return nullptr;
        g->addPoint(v);
    }
    return A.PyLong_FromLong(first);
}
HD_FN(f_g_prims) {
    long h;
    if (!A.PyArg_ParseTuple(args, "l", &h)) return nullptr;
    Geometry* g = geoArg(h, false);
    if (!g) return nullptr;
    PyObject* l = A.PyList_New(Py_ssize_t(g->numPrims()));
    for (size_t i = 0; i < g->numPrims(); ++i) {
        const Prim& pr = g->prims[i];
        PyObject* pts = A.PyList_New(pr.count);
        for (int k = 0; k < pr.count; ++k) A.PyList_SetItem(pts, k, A.PyLong_FromLong(g->vertices[size_t(pr.start + k)]));
        A.PyList_SetItem(l, Py_ssize_t(i), A.Py_BuildValue("(NN)", pts, A.PyBool_FromLong(pr.closed)));
    }
    return l;
}
HD_FN(f_g_add_prim) {
    long h; PyObject* seq; int closed = 1;
    if (!A.PyArg_ParseTuple(args, "lO|i", &h, &seq, &closed)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    Py_ssize_t n = A.PySequence_Size(seq);
    if (n < 0) return nullptr;
    if (n < 2) return fail(A.ExcValueError, "a primitive needs at least 2 points");
    std::vector<int> pts;
    for (Py_ssize_t i = 0; i < n; ++i) {
        PyObject* it = A.PySequence_GetItem(seq, i);
        if (!it) return nullptr;
        long v = A.PyLong_AsLong(it);
        A.Py_DecRef(it);
        if (v == -1 && A.PyErr_Occurred()) return nullptr;
        if (v < 0 || size_t(v) >= g->numPoints()) return fail(A.ExcIndexError, "point " + std::to_string(v) + " does not exist");
        pts.push_back(int(v));
    }
    return A.PyLong_FromLong(g->addPrim(pts, closed != 0));
}
HD_FN(f_g_attribs) {
    long h; const char* cls;
    if (!A.PyArg_ParseTuple(args, "ls", &h, &cls)) return nullptr;
    Geometry* g = geoArg(h, false);
    if (!g) return nullptr;
    PyObject* l = A.PyList_New(0);
    for (auto& [name, a] : g->attribs(classArg(cls))) { PyObject* e = A.Py_BuildValue("(si)", name.c_str(), a.tuple); A.PyList_Append(l, e); A.Py_DecRef(e); }
    return l;
}
HD_FN(f_g_attrib_get) {
    long h; const char* cls; const char* name;
    if (!A.PyArg_ParseTuple(args, "lss", &h, &cls, &name)) return nullptr;
    Geometry* g = geoArg(h, false);
    if (!g) return nullptr;
    const Attrib* a = g->findAttrib(classArg(cls), name);
    if (!a) return fail(A.ExcValueError, std::string("no ") + cls + " attribute '" + name + "'");
    size_t n = a->count();
    PyObject* l = A.PyList_New(Py_ssize_t(n));
    for (size_t i = 0; i < n; ++i) A.PyList_SetItem(l, Py_ssize_t(i), a->tuple == 3 ? tuple3(a->getV(i)) : A.PyFloat_FromDouble(a->getF(i)));
    return l;
}
HD_FN(f_g_attrib_set) {
    long h; const char* cls; const char* name; int size; PyObject* seq;
    if (!A.PyArg_ParseTuple(args, "lssiO", &h, &cls, &name, &size, &seq)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    if (size != 1 && size != 3) return fail(A.ExcValueError, "attribute size must be 1 or 3");
    if (std::string(name) == "P") return fail(A.ExcValueError, "use setPointPositions() for P");
    AttribClass c = classArg(cls);
    const size_t count = c == AttribClass::Point ? g->numPoints() : g->numPrims();
    Py_ssize_t n = A.PySequence_Size(seq);
    if (n < 0) return nullptr;
    if (size_t(n) != count) return fail(A.ExcValueError, "expected " + std::to_string(count) + " values, got " + std::to_string(n));
    Attrib* existing = g->findAttrib(c, name);
    if (existing && existing->tuple != size) g->removeAttrib(c, name);
    Attrib& a = g->addAttrib(c, name, size);
    a.resize(count);
    for (Py_ssize_t i = 0; i < n; ++i) {
        PyObject* it = A.PySequence_GetItem(seq, i);
        if (!it) return nullptr;
        bool ok;
        if (size == 3) { Vec3 v; ok = toVec3(it, v); if (ok) a.setV(size_t(i), v); }
        else { float f; ok = toFloat(it, f); if (ok) a.setF(size_t(i), f); }
        A.Py_DecRef(it);
        if (!ok) return nullptr;
    }
    return none();
}
HD_FN(f_g_attrib_remove) {
    long h; const char* cls; const char* name;
    if (!A.PyArg_ParseTuple(args, "lss", &h, &cls, &name)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    g->removeAttrib(classArg(cls), name);
    return none();
}
HD_FN(f_g_bounds) {
    long h;
    if (!A.PyArg_ParseTuple(args, "l", &h)) return nullptr;
    Geometry* g = geoArg(h, false);
    if (!g) return nullptr;
    Vec3 mn, mx;
    g->bounds(mn, mx);
    return A.Py_BuildValue("(NN)", tuple3(mn), tuple3(mx));
}
HD_FN(f_g_clear) {
    long h;
    if (!A.PyArg_ParseTuple(args, "l", &h)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    g->clear();
    return none();
}
HD_FN(f_g_normals) {
    long h;
    if (!A.PyArg_ParseTuple(args, "l", &h)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    g->computePointNormals();
    return none();
}
HD_FN(f_g_keep_points) {
    long h; PyObject* seq;
    if (!A.PyArg_ParseTuple(args, "lO", &h, &seq)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    Py_ssize_t n = A.PySequence_Size(seq);
    if (n < 0) return nullptr;
    if (size_t(n) != g->numPoints()) return fail(A.ExcValueError, "expected one flag per point");
    std::vector<char> keep(g->numPoints());
    for (Py_ssize_t i = 0; i < n; ++i) {
        PyObject* it = A.PySequence_GetItem(seq, i);
        if (!it) return nullptr;
        int t = A.PyObject_IsTrue(it);
        A.Py_DecRef(it);
        if (t < 0) return nullptr;
        keep[size_t(i)] = char(t);
    }
    Geometry kept = g->extractPoints(keep);
    kept.volumes = g->volumes;
    kept.images = g->images;
    kept.packed = g->packed;
    *g = std::move(kept);
    return none();
}
HD_FN(f_g_merge) {
    long h, other;
    if (!A.PyArg_ParseTuple(args, "ll", &h, &other)) return nullptr;
    Geometry* g = geoArg(h, true);
    if (!g) return nullptr;
    Geometry* o = geoArg(other, false);
    if (!o) return nullptr;
    if (o == g) { Geometry copy = *o; g->merge(copy); }
    else g->merge(*o);
    return none();
}

PyMethodDef g_methods[] = {
    {"_write", f_write, METH_VARARGS, nullptr},
    {"nodes", f_nodes, METH_VARARGS, nullptr},
    {"exists", f_exists, METH_VARARGS, nullptr},
    {"node_type", f_node_type, METH_VARARGS, nullptr},
    {"node_types", f_node_types, METH_VARARGS, nullptr},
    {"create", f_create, METH_VARARGS, nullptr},
    {"delete", f_delete, METH_VARARGS, nullptr},
    {"rename", f_rename, METH_VARARGS, nullptr},
    {"connect", f_connect, METH_VARARGS, nullptr},
    {"disconnect", f_disconnect, METH_VARARGS, nullptr},
    {"inputs", f_inputs, METH_VARARGS, nullptr},
    {"outputs", f_outputs, METH_VARARGS, nullptr},
    {"display", f_display, METH_VARARGS, nullptr},
    {"position", f_position, METH_VARARGS, nullptr},
    {"parms", f_parms, METH_VARARGS, nullptr},
    {"parm_get", f_parm_get, METH_VARARGS, nullptr},
    {"parm_set", f_parm_set, METH_VARARGS, nullptr},
    {"parm_set_str", f_parm_set_str, METH_VARARGS, nullptr},
    {"parm_expr", f_parm_expr, METH_VARARGS, nullptr},
    {"parm_key", f_parm_key, METH_VARARGS, nullptr},
    {"parm_clear_keys", f_parm_clear_keys, METH_VARARGS, nullptr},
    {"parm_keys", f_parm_keys, METH_VARARGS, nullptr},
    {"cook", f_cook, METH_VARARGS, nullptr},
    {"frame", f_frame, METH_VARARGS, nullptr},
    {"fps", f_fps, METH_VARARGS, nullptr},
    {"range", f_range, METH_VARARGS, nullptr},
    {"save", f_save, METH_VARARGS, nullptr},
    {"load", f_load, METH_VARARGS, nullptr},
    {"clear", f_clear, METH_VARARGS, nullptr},
    {"pwd", f_pwd, METH_VARARGS, nullptr},
    {"node_geo", f_node_geo, METH_VARARGS, nullptr},
    {"g_info", f_g_info, METH_VARARGS, nullptr},
    {"g_points", f_g_points, METH_VARARGS, nullptr},
    {"g_set_points", f_g_set_points, METH_VARARGS, nullptr},
    {"g_add_points", f_g_add_points, METH_VARARGS, nullptr},
    {"g_prims", f_g_prims, METH_VARARGS, nullptr},
    {"g_add_prim", f_g_add_prim, METH_VARARGS, nullptr},
    {"g_attribs", f_g_attribs, METH_VARARGS, nullptr},
    {"g_attrib_get", f_g_attrib_get, METH_VARARGS, nullptr},
    {"g_attrib_set", f_g_attrib_set, METH_VARARGS, nullptr},
    {"g_attrib_remove", f_g_attrib_remove, METH_VARARGS, nullptr},
    {"g_bounds", f_g_bounds, METH_VARARGS, nullptr},
    {"g_clear", f_g_clear, METH_VARARGS, nullptr},
    {"g_normals", f_g_normals, METH_VARARGS, nullptr},
    {"g_keep_points", f_g_keep_points, METH_VARARGS, nullptr},
    {"g_merge", f_g_merge, METH_VARARGS, nullptr},
    {nullptr, nullptr, 0, nullptr},
};

} // namespace

// ------------------------------------------------------------------ the `hd` module (Python side)
const char* moduleSource() {
    return R"PY("""HD scripting API.

Nodes are addressed by name (or "subnet/child" paths). Typical use:

    import hd
    box = hd.createNode("box")
    box.parm("size").set((2, 1, 1))
    xf = hd.createNode("xform")
    xf.setInput(0, box)
    xf.parm("t").setExpression("sin($F * 0.1)", 1)
    xf.setDisplayFlag()
    print(xf.geometry().numPoints())

Inside a Python SOP, `geo` is the node's output (start: a copy of input 1), `inputs` holds the
read-only geometry of every input and `node` (also hd.pwd()) is the node itself.
"""
import _hd

__all__ = ["Node", "Parm", "Geometry", "node", "nodes", "createNode", "nodeTypes", "frame", "setFrame", "fps",
           "setFps", "frameRange", "setFrameRange", "displayNode", "save", "load", "clearScene", "pwd"]


class Parm:
    """A node parameter. eval() returns a float, a tuple of 3 floats (vectors) or a string."""

    def __init__(self, node, name, label="", kind="float", size=1, menu=()):
        self._node, self._name, self._label, self._kind, self._size, self._menu = node, name, label, kind, size, tuple(menu)

    def name(self): return self._name
    def label(self): return self._label
    def node(self): return self._node
    def type(self): return self._kind
    def size(self): return self._size
    def menuItems(self): return self._menu

    def eval(self):
        v = _hd.parm_get(self._node.path(), self._name)
        if isinstance(v, tuple) and len(v) == 1:
            v = v[0]
            if self._kind in ("int", "menu"):
                return int(round(v))
            if self._kind == "toggle":
                return bool(v)
        return v

    def set(self, value):
        """Numbers for numeric parameters (a sequence of 3 for vectors), strings for text ones.
        Menus also take an item label."""
        if isinstance(value, str):
            _hd.parm_set_str(self._node.path(), self._name, value)
        elif isinstance(value, (tuple, list)):
            if len(value) != self._size:
                raise ValueError("%s needs %d values" % (self._name, self._size))
            for i, v in enumerate(value):
                _hd.parm_set(self._node.path(), self._name, i, float(v))
        else:
            if self._size != 1:
                raise ValueError("%s is a vector: pass %d values" % (self._name, self._size))
            _hd.parm_set(self._node.path(), self._name, 0, float(value))
        return self

    def setExpression(self, expression, component=0):
        _hd.parm_expr(self._node.path(), self._name, component, expression)
        return self

    def expression(self, component=0):
        return _hd.parm_expr(self._node.path(), self._name, component)

    def setKeyframe(self, frame, value, component=0):
        _hd.parm_key(self._node.path(), self._name, component, float(frame), float(value))
        return self

    def keyframes(self, component=0):
        return _hd.parm_keys(self._node.path(), self._name, component)

    def deleteKeyframes(self, component=0):
        _hd.parm_clear_keys(self._node.path(), self._name, component)
        return self

    def __repr__(self):
        return "<hd.Parm %s/%s>" % (self._node.path(), self._name)


class Node:
    def __init__(self, path):
        self._path = path

    def path(self): return self._path
    def name(self): return self._path.rsplit("/", 1)[-1]
    def type(self): return _hd.node_type(self._path)[0]
    def typeLabel(self): return _hd.node_type(self._path)[1]
    def category(self): return _hd.node_type(self._path)[2]
    def exists(self): return _hd.exists(self._path)

    def parms(self):
        return [Parm(self, *p) for p in _hd.parms(self._path)]

    def parm(self, name):
        for p in _hd.parms(self._path):
            if p[0] == name:
                return Parm(self, *p)
        return None

    def evalParm(self, name):
        p = self.parm(name)
        if p is None:
            raise ValueError("%s has no parameter '%s'" % (self._path, name))
        return p.eval()

    def setParms(self, values):
        """Sets several parameters from a dict {name: value}."""
        for k, v in values.items():
            p = self.parm(k)
            if p is None:
                raise ValueError("%s has no parameter '%s'" % (self._path, k))
            p.set(v)
        return self

    def inputs(self):
        return [Node(p) if p else None for p in _hd.inputs(self._path)]

    def setInput(self, index, node):
        if node is None:
            _hd.disconnect(self._path, index)
        else:
            _hd.connect(node.path() if isinstance(node, Node) else str(node), self._path, index)
        return self

    def outputs(self):
        return [Node(p) for p, _ in _hd.outputs(self._path)]

    def setDisplayFlag(self, on=True):
        if on:
            _hd.display(self._path)
        return self

    def isDisplayFlagSet(self):
        return _hd.display() == self.name()

    def cook(self):
        """Cooks the node at the current frame; raises if the node errors."""
        err, _, _ = _hd.cook(self._path)
        if err:
            raise RuntimeError("%s: %s" % (self._path, err))
        return self

    def errors(self): return _hd.cook(self._path)[0]
    def warnings(self): return _hd.cook(self._path)[1]
    def cookTime(self): return _hd.cook(self._path)[2]

    def geometry(self):
        """The node's cooked geometry (read-only)."""
        return Geometry(_hd.node_geo(self._path))

    def destroy(self):
        _hd.delete(self._path)

    def setName(self, name):
        self._path = _hd.rename(self._path, name)
        return self

    def position(self): return _hd.position(self._path)

    def setPosition(self, xy):
        _hd.position(self._path, tuple(xy))
        return self

    def createNode(self, type, name=None):
        """Creates a node inside this subnet."""
        return Node(_hd.create(type, name or "", self._path))

    def children(self):
        return [Node(p) for p in _hd.nodes(self._path)]

    def __eq__(self, other): return isinstance(other, Node) and other._path == self._path
    def __hash__(self): return hash(self._path)
    def __repr__(self):
        try:
            return "<hd.Node %s (%s)>" % (self._path, self.type())
        except Exception:
            return "<hd.Node %s (deleted)>" % self._path


class Geometry:
    """Points, primitives and attributes. cls is 'point' or 'prim'."""

    def __init__(self, handle):
        self._h = handle

    def _info(self): return _hd.g_info(self._h)
    def numPoints(self): return self._info()[0]
    def numPrims(self): return self._info()[1]
    def numVertices(self): return self._info()[2]
    def numPacked(self): return self._info()[3]
    def numVolumes(self): return self._info()[4]
    def isWritable(self): return bool(self._info()[6])

    def pointPositions(self): return _hd.g_points(self._h)
    def setPointPositions(self, positions): _hd.g_set_points(self._h, list(positions))
    def addPoint(self, position): return _hd.g_add_points(self._h, [tuple(position)])
    def addPoints(self, positions): return _hd.g_add_points(self._h, [tuple(p) for p in positions])

    def prims(self):
        """List of (point indices, closed) per primitive."""
        return _hd.g_prims(self._h)

    def addPrim(self, points, closed=True): return _hd.g_add_prim(self._h, list(points), 1 if closed else 0)
    def addPolygon(self, points): return self.addPrim(points, True)
    def addPolyline(self, points): return self.addPrim(points, False)

    def attribNames(self, cls="point"): return [n for n, _ in _hd.g_attribs(self._h, cls)]
    def attribSizes(self, cls="point"): return dict(_hd.g_attribs(self._h, cls))
    def attribValues(self, name, cls="point"): return _hd.g_attrib_get(self._h, cls, name)

    def setAttribValues(self, name, values, cls="point", size=None):
        values = list(values)
        if size is None:
            size = 3 if values and isinstance(values[0], (tuple, list)) else 1
        _hd.g_attrib_set(self._h, cls, name, size, values)

    def removeAttrib(self, name, cls="point"): _hd.g_attrib_remove(self._h, cls, name)
    def boundingBox(self): return _hd.g_bounds(self._h)
    def clear(self): _hd.g_clear(self._h)
    def computeNormals(self): _hd.g_normals(self._h)

    def deletePoints(self, indices):
        """Removes the given points (and primitives that use them)."""
        gone = set(indices)
        _hd.g_keep_points(self._h, [i not in gone for i in range(self.numPoints())])

    def merge(self, other): _hd.g_merge(self._h, other._h)

    def __repr__(self):
        i = self._info()
        return "<hd.Geometry %d points, %d prims%s>" % (i[0], i[1], ", %d packed" % i[3] if i[3] else "")


def node(path):
    """The node at path, or None."""
    return Node(path) if _hd.exists(path) else None

def nodes(): return [Node(p) for p in _hd.nodes()]

def createNode(type, name=None):
    return Node(_hd.create(type, name or ""))

def nodeTypes():
    """[(name, label, category, description)] of every node type."""
    return _hd.node_types()

def frame(): return _hd.frame()
def setFrame(f): _hd.frame(float(f))
def fps(): return _hd.fps()
def setFps(f): _hd.fps(float(f))
def frameRange(): return _hd.range()
def setFrameRange(start, end): _hd.range(int(start), int(end))

def displayNode():
    n = _hd.display()
    return Node(n) if n else None

def save(path): _hd.save(path)
def load(path): _hd.load(path)
def clearScene(): _hd.clear()

def pwd():
    """The node running this code (Python SOP), or None."""
    p = _hd.pwd()
    return Node(p) if p else None


class _Out:
    def write(self, s):
        _hd._write(str(s))
        return len(s)
    def flush(self): pass
    def isatty(self): return False


def _run(code, filename, globals_, shell):
    import sys, traceback
    old = sys.stdout, sys.stderr
    sys.stdout = sys.stderr = _Out()
    try:
        if shell:
            try:
                expr = compile(code, filename, "eval")
            except SyntaxError:
                expr = None
            if expr is not None:
                value = eval(expr, globals_)
                if value is not None:
                    print(repr(value))
                return True
        exec(compile(code, filename, "exec"), globals_)
        return True
    except SystemExit as e:
        return not e.code
    except BaseException:
        etype, value, tb = sys.exc_info()
        traceback.print_exception(etype, value, tb.tb_next if tb is not None else None)  # skip this frame
        return False
    finally:
        sys.stdout, sys.stderr = old
)PY";
}

namespace {

bool startPython(Info& info) {
    if (const char* off = std::getenv("HD_NO_PYTHON"); off && *off && std::string(off) != "0") {
        info.error = "Python is turned off (HD_NO_PYTHON is set).";
        return false;
    }
    // 1. ask an interpreter where it lives
    std::vector<std::string> exes;
    if (const char* e = std::getenv("HD_PYTHON"); e && *e) exes.push_back(e);
#if defined(_WIN32)
    exes.push_back("python");
    exes.push_back("python3");
    exes.push_back("py -3");
#elif defined(__APPLE__)
    // apps started from Finder get a minimal PATH: look in the usual install places first
    for (const char* p : {"/opt/homebrew/bin/python3", "/usr/local/bin/python3", "/Library/Frameworks/Python.framework/Versions/Current/bin/python3"}) {
        std::error_code e2;
        if (fs::exists(p, e2)) exes.push_back(p);
    }
    exes.push_back("python3");
#else
    exes.push_back("python3");
    exes.push_back("python");
#endif
    std::error_code ec;
    fs::path tmp = fs::temp_directory_path(ec) / ("hd_python_query_" + std::to_string(std::rand()) + ".py");
    { std::ofstream f(tmp); f << kQuery; }
    Json q;
    std::string tried;
    for (const std::string& exe : exes) {
        std::string cmd = (exe.find(' ') != std::string::npos && exe.rfind("py ", 0) != 0 ? "\"" + exe + "\"" : exe) + " \"" + tmp.string() + "\"";
        std::string out;
        tried += (tried.empty() ? "" : ", ") + exe;
        if (!capture(cmd, out)) continue;
        size_t brace = out.find('{');
        if (brace == std::string::npos) continue;
        std::string perr;
        Json j = Json::parse(out.substr(brace), &perr);
        if (!perr.empty() || !j.isObject()) continue;
        if (j.get("major").num() != 3 || j.get("minor").num() < 8) {
            info.error = exe + " is Python " + j.get("version").str() + "; HD needs Python 3.8 or newer.";
            continue;
        }
        q = j;
        info.executable = j.get("executable").str();
        info.version = j.get("version").str();
        break;
    }
    fs::remove(tmp, ec);
    if (!q.isObject()) {
        if (info.error.empty())
            info.error = "No Python 3 found (tried " + tried + "). Install Python 3.8+ or set HD_PYTHON to the python executable.";
        return false;
    }
    // 2. load its shared library
    LibHandle lib = nullptr;
    std::string loadErr;
    for (const Json& l : q.get("libs").items()) {
        const std::string& p = l.str();
        if (p.empty() || !fs::exists(p, ec)) continue;
        std::string e;
        lib = openLib(p, e);
        if (lib) { info.library = p; break; }
        loadErr += e + "\n";
    }
    if (!lib) {
        info.error = info.executable + " (Python " + info.version + ") has no shared library HD can load" +
                     (loadErr.empty() ? " (it may be built statically; install a Python with libpython, such as your system or python.org Python)." : ":\n" + loadErr);
        return false;
    }
    // 3. resolve the stable ABI
    std::string missing;
#define R(name) resolve(lib, A.name, #name, missing)
    R(Py_InitializeEx); R(Py_IsInitialized); R(PyEval_SaveThread); R(PyGILState_Ensure); R(PyGILState_Release);
    R(Py_IncRef); R(Py_DecRef); R(PyImport_ImportModule); R(PyImport_AddModule); R(PyImport_GetModuleDict);
    R(PyModule_New); R(PyModule_GetDict); R(PyCFunction_NewEx); R(PyDict_New); R(PyDict_SetItemString);
    R(PyDict_GetItemString); R(Py_CompileString); R(PyEval_EvalCode); R(PyObject_CallObject); R(PyErr_Occurred);
    R(PyErr_Clear); R(PyErr_SetString); R(PyErr_Print); R(PyArg_ParseTuple); R(Py_BuildValue); R(PyLong_FromLong);
    R(PyLong_AsLong); R(PyFloat_FromDouble); R(PyFloat_AsDouble); R(PyBool_FromLong); R(PyUnicode_FromString);
    R(PyUnicode_AsUTF8String); R(PyBytes_AsString); R(PyTuple_New); R(PyTuple_SetItem); R(PyList_New);
    R(PyList_SetItem); R(PyList_Append); R(PySequence_Size); R(PySequence_GetItem); R(PyObject_IsTrue);
#undef R
    A.None = reinterpret_cast<PyObject*>(sym(lib, "_Py_NoneStruct"));
    auto exc = [&](const char* n) { PyObject** p = reinterpret_cast<PyObject**>(sym(lib, n)); return p ? *p : nullptr; };
    if (!A.None) missing += ", _Py_NoneStruct";
    if (!missing.empty()) { info.error = "Python library " + info.library + " lacks: " + missing; return false; }
    // 4. start it: PYTHONHOME points at the interpreter's install so it finds its standard library
    const char* oldHome = std::getenv("PYTHONHOME");
    std::string saved = oldHome ? oldHome : "";
    if (!A.Py_IsInitialized()) {
        setEnv("PYTHONHOME", q.get("base_prefix").str());
        A.Py_InitializeEx(0);
        if (oldHome) setEnv("PYTHONHOME", saved); else unsetEnv("PYTHONHOME");
    }
    A.ExcRuntimeError = exc("PyExc_RuntimeError");
    A.ExcValueError = exc("PyExc_ValueError");
    A.ExcIndexError = exc("PyExc_IndexError");
    if (!A.ExcRuntimeError || !A.ExcValueError || !A.ExcIndexError) { info.error = "Python exceptions not found"; return false; }
    // 5. the _hd module, sys.path, then the hd module
    PyObject* mods = A.PyImport_GetModuleDict();
    PyObject* m = A.PyModule_New("_hd");
    for (PyMethodDef* d = g_methods; d->ml_name; ++d) {
        PyObject* f = A.PyCFunction_NewEx(d, nullptr, nullptr);
        A.PyDict_SetItemString(A.PyModule_GetDict(m), d->ml_name, f);
        A.Py_DecRef(f);
    }
    A.PyDict_SetItemString(mods, "_hd", m);
    A.Py_DecRef(m);
    {
        PyObject* sys = A.PyImport_ImportModule("sys");
        PyObject* path = A.PyList_New(0);
        for (const Json& p : q.get("path").items()) { PyObject* s = str(p.str()); A.PyList_Append(path, s); A.Py_DecRef(s); }
        if (sys) {
            PyObject* d = A.PyModule_GetDict(sys);
            A.PyDict_SetItemString(d, "path", path);
            A.Py_DecRef(sys);
        }
        A.Py_DecRef(path);
    }
    PyObject* hdm = A.PyModule_New("hd");
    PyObject* hdd = A.PyModule_GetDict(hdm);
    PyObject* builtins = A.PyImport_ImportModule("builtins");
    A.PyDict_SetItemString(hdd, "__builtins__", builtins);
    PyObject* code = A.Py_CompileString(moduleSource(), "hd.py", Py_file_input);
    PyObject* res = code ? A.PyEval_EvalCode(code, hdd, hdd) : nullptr;
    if (code) A.Py_DecRef(code);
    if (!res) { A.PyErr_Print(); info.error = "The hd module failed to load (see the console)."; return false; }
    A.Py_DecRef(res);
    A.PyDict_SetItemString(mods, "hd", hdm);
    g_hdModule = hdm;
    // the shell's namespace: like __main__, with hd imported
    g_shellGlobals = A.PyDict_New();
    A.PyDict_SetItemString(g_shellGlobals, "__builtins__", builtins);
    PyObject* nm = str("__main__");
    A.PyDict_SetItemString(g_shellGlobals, "__name__", nm);
    A.Py_DecRef(nm);
    A.PyDict_SetItemString(g_shellGlobals, "hd", hdm);
    A.Py_DecRef(builtins);
    A.PyEval_SaveThread(); // release the GIL; every call takes it with PyGILState_Ensure
    return true;
}

// Runs hd._run(code, filename, globals, shell) with the GIL held and a host context pushed.
bool runIn(Ctx c, const std::string& code, const std::string& filename, PyObject* globals, bool shell, bool ownGlobals,
           const std::function<void(PyObject*)>& fillGlobals = {}) {
    int gs = A.PyGILState_Ensure();
    g_ctx.push_back(std::move(c));
    bool ok = false;
    if (!globals) {
        globals = A.PyDict_New();
        PyObject* builtins = A.PyImport_ImportModule("builtins");
        A.PyDict_SetItemString(globals, "__builtins__", builtins);
        A.Py_DecRef(builtins);
        PyObject* nm = str("__main__");
        A.PyDict_SetItemString(globals, "__name__", nm);
        A.Py_DecRef(nm);
        A.PyDict_SetItemString(globals, "hd", g_hdModule);
        ownGlobals = true;
    }
    if (fillGlobals) fillGlobals(globals);
    PyObject* fn = A.PyDict_GetItemString(A.PyModule_GetDict(g_hdModule), "_run"); // borrowed
    PyObject* args = A.Py_BuildValue("(ssOi)", code.c_str(), filename.c_str(), globals, shell ? 1 : 0);
    PyObject* r = fn && args ? A.PyObject_CallObject(fn, args) : nullptr;
    if (args) A.Py_DecRef(args);
    if (r) { ok = A.PyObject_IsTrue(r) == 1; A.Py_DecRef(r); }
    else { if (A.PyErr_Occurred()) A.PyErr_Print(); A.PyErr_Clear(); }
    if (ownGlobals) A.Py_DecRef(globals);
    g_ctx.pop_back();
    A.PyGILState_Release(gs);
    return ok;
}

} // namespace

const Info& ensure() {
    if (!g_tried) {
        g_tried = true;
        g_info.ok = startPython(g_info);
    }
    return g_info;
}

bool available() { return g_tried && g_info.ok; }

std::string takeLog() {
    std::string s;
    s.swap(g_log);
    return s;
}

// ---- Python SOP
namespace {
ParmDef codeParm() {
    ParmDef p;
    p.name = "code"; p.label = "Python Code"; p.type = ParmType::Code;
    p.defStr = "# geo: this node's output (starts as a copy of input 1)\n"
               "# inputs: read-only geometry of each input; node: this node; hd: the scripting module\n"
               "P = geo.pointPositions()\n"
               "geo.setPointPositions([(x, y + 0.1 * x * x, z) for x, y, z in P])\n";
    p.help = "Runs in the Python found at startup (see Help > Python). print() output goes to the Python Shell.";
    return p;
}
void cookPython(CookContext& c) {
    Geometry out;
    std::vector<const Geometry*> ins;
    for (int i = 0; i < 4; ++i)
        if (c.hasInput(i)) ins.push_back(&c.input(i));
    if (!ins.empty()) out = *ins[0];
    std::string output, error;
    if (!runGeometry(c.graph(), c.node(), c.s("code"), ins, out, output, error)) { c.error(error); return; }
    c.out = std::move(out);
}
} // namespace

void registerNodes(std::vector<NodeType>& types) {
    NodeType t; t.name = "python"; t.label = "Python"; t.category = "Utility";
    t.description = "Edits geometry with Python code (needs Python 3.8+ installed).";
    t.minInputs = 0; t.maxInputs = 4;
    t.inputLabels = {"Geometry (copied to geo)", "Input 2", "Input 3", "Input 4"};
    t.parms = {codeParm()};
    t.cook = cookPython;
    types.push_back(t);
}

bool run(Graph& g, const std::string& code, std::string& output, const std::string& filename, bool shell) {
    const Info& inf = ensure();
    if (!inf.ok) { output += inf.error + "\n"; return false; }
    Ctx c;
    c.graph = &g;
    c.out = &output;
    return runIn(std::move(c), code, filename, shell ? g_shellGlobals : nullptr, shell, false);
}

bool runGeometry(Graph& g, Node& node, const std::string& code, const std::vector<const Geometry*>& inputs, Geometry& out,
                 std::string& output, std::string& error) {
    const Info& inf = ensure();
    if (!inf.ok) { error = "Python is not available: " + inf.error; return false; }
    // root graph (paths are relative to it) and this node's path
    Graph* root = &g;
    while (root->parent()) root = root->parent();
    Ctx c;
    c.graph = root;
    c.out = &output;
    c.pwd = pathOf(*root, &g, &node);
    GeoHandle h;
    h.geo = &out;
    h.writable = true;
    c.geos.push_back(h);
    for (const Geometry* in : inputs) {
        GeoHandle hi;
        hi.geo = const_cast<Geometry*>(in); // read-only: writable stays false
        c.geos.push_back(hi);
    }
    const size_t nInputs = inputs.size();
    bool ok = runIn(std::move(c), code, node.name(), nullptr, false, true, [&](PyObject* glob) {
        PyObject* hdd = A.PyModule_GetDict(g_hdModule);
        PyObject* GeoCls = A.PyDict_GetItemString(hdd, "Geometry");
        PyObject* NodeCls = A.PyDict_GetItemString(hdd, "Node");
        auto make = [&](PyObject* cls, PyObject* arg) {
            PyObject* args = A.Py_BuildValue("(O)", arg);
            PyObject* o = A.PyObject_CallObject(cls, args);
            A.Py_DecRef(args);
            return o;
        };
        PyObject* zero = A.PyLong_FromLong(0);
        PyObject* geo = make(GeoCls, zero);
        A.Py_DecRef(zero);
        if (geo) { A.PyDict_SetItemString(glob, "geo", geo); A.Py_DecRef(geo); }
        PyObject* list = A.PyList_New(0);
        for (size_t i = 0; i < nInputs; ++i) {
            PyObject* hi = A.PyLong_FromLong(long(i + 1));
            PyObject* gi = make(GeoCls, hi);
            A.Py_DecRef(hi);
            if (gi) { A.PyList_Append(list, gi); A.Py_DecRef(gi); }
        }
        A.PyDict_SetItemString(glob, "inputs", list);
        A.Py_DecRef(list);
        PyObject* p = str(g_ctx.back().pwd);
        PyObject* nd = make(NodeCls, p);
        A.Py_DecRef(p);
        if (nd) { A.PyDict_SetItemString(glob, "node", nd); A.Py_DecRef(nd); }
        if (A.PyErr_Occurred()) A.PyErr_Print();
    });
    if (!output.empty()) g_log += "[" + node.name() + "] " + output + (output.back() == '\n' ? "" : "\n");
    if (g_log.size() > 1000000) g_log.erase(0, g_log.size() - 1000000);
    if (!ok) {
        // the last non-empty line of the traceback is the error message
        std::string last;
        std::stringstream ss(output);
        std::string line;
        while (std::getline(ss, line)) if (!line.empty() && line[0] != ' ') last = line;
        error = last.empty() ? "Python code failed" : last;
    }
    return ok;
}

} // namespace python
} // namespace hd
