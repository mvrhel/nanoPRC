#include <algorithm>
#include <array>
#include <cctype>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

extern "C" {
#include "prc_api.h"
}

namespace py = pybind11;

namespace {

static std::string safe_string(const char* cstr) {
    return cstr ? std::string(cstr) : std::string();
}

static py::object attribute_value_to_python(const prc_api_attribute_entry& entry)
{
    switch (entry.type) {
    case PRC_API_INTEGER_ATTRIBUTE:
        return py::int_(entry.value_integer);
    case PRC_API_DOUBLE_ATTRIBUTE:
        return py::float_(entry.value_double);
    case PRC_API_VALUE_SECS_INTEGER_ATTRIBUTE:
        return py::int_(entry.value_secs_integer);
    case PRC_API_STRING_ATTRIBUTE:
        return py::str(safe_string(entry.value_string));
    case PRC_API_VALUE_TIME_ATTRIBUTE:
        return py::int_(entry.value_time);
    default:
        return py::none();
    }
}

static py::list attributes_to_python(const prc_api_attributes* attrs)
{
    py::list bases;
    if (!attrs || attrs->num_base_attributes == 0 || !attrs->base_attributes) {
        return bases;
    }

    for (uint32_t base_index = 0; base_index < attrs->num_base_attributes; ++base_index) {
        const prc_api_attribute_base& base = attrs->base_attributes[base_index];
        py::dict base_obj;
        base_obj["base_title"] = py::str(safe_string(base.attribute_base_title));

        py::list entries;
        if (base.num_attributes > 0 && base.attributes) {
            for (size_t entry_index = 0; entry_index < base.num_attributes; ++entry_index) {
                const prc_api_attribute_entry& entry = base.attributes[entry_index];
                py::dict entry_obj;
                entry_obj["entry_title"] = py::str(safe_string(entry.entry_title));
                entry_obj["type"] = py::int_(static_cast<int>(entry.type));
                entry_obj["value"] = attribute_value_to_python(entry);
                entries.append(entry_obj);
            }
        }

        base_obj["entries"] = entries;
        bases.append(base_obj);
    }

    return bases;
}

static py::array make_float_array_from_vertex_field(
    const prc_api_tess_vertex_buffer& buffer,
    const float* field_ptr,
    size_t component_count,
    const py::object& owner)
{
    if (!buffer.vertices || buffer.num_vertices == 0) {
        std::vector<py::ssize_t> shape = {0, static_cast<py::ssize_t>(component_count)};
        std::vector<py::ssize_t> strides = { static_cast<py::ssize_t>(sizeof(prc_api_vertex)), static_cast<py::ssize_t>(sizeof(float)) };
        py::buffer_info info(
            nullptr,
            sizeof(float),
            py::format_descriptor<float>::format(),
            2,
            shape,
            strides
        );
        return py::array(info, owner);
    }

    std::vector<py::ssize_t> shape = { static_cast<py::ssize_t>(buffer.num_vertices), static_cast<py::ssize_t>(component_count) };
    std::vector<py::ssize_t> strides = { static_cast<py::ssize_t>(sizeof(prc_api_vertex)), static_cast<py::ssize_t>(sizeof(float)) };
    py::buffer_info info(
        const_cast<float*>(field_ptr),
        sizeof(float),
        py::format_descriptor<float>::format(),
        2,
        shape,
        strides
    );

    return py::array(info, owner);
}

static py::array make_uint32_array(const uint32_t* data, size_t count, const py::object& owner)
{
    if (!data || count == 0) {
        std::vector<py::ssize_t> shape = {0};
        std::vector<py::ssize_t> strides = { static_cast<py::ssize_t>(sizeof(uint32_t)) };
        py::buffer_info info(
            nullptr,
            sizeof(uint32_t),
            py::format_descriptor<uint32_t>::format(),
            1,
            shape,
            strides
        );
        return py::array(info, owner);
    }

    std::vector<py::ssize_t> shape = { static_cast<py::ssize_t>(count) };
    std::vector<py::ssize_t> strides = { static_cast<py::ssize_t>(sizeof(uint32_t)) };
    py::buffer_info info(
        const_cast<uint32_t*>(data),
        sizeof(uint32_t),
        py::format_descriptor<uint32_t>::format(),
        1,
        shape,
        strides
    );

    return py::array(info, owner);
}

struct OwnedWriteTessellation
{
    prc_api_write_tessellation tess = {};
    std::vector<double> positions;
    std::vector<double> normals;
    std::vector<uint32_t> tri_indices;
    std::vector<uint32_t> norm_indices;
    std::vector<uint32_t> face_tri_counts;
};

struct OwnedWriteNode
{
    prc_api_write_node node = {};
    std::vector<prc_api_write_rep_item> rep_items;
    std::vector<std::unique_ptr<OwnedWriteNode>> children_storage;
    std::vector<prc_api_write_node*> child_ptrs;
    std::string name_storage;
    std::string part_name_storage;
};

static std::string to_lower_ascii(std::string value)
{
    for (char &c : value) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return value;
}

static prc_api_write_ri_kind_t parse_ri_kind(const py::handle &h)
{
    if (py::isinstance<py::str>(h)) {
        std::string s = to_lower_ascii(py::cast<std::string>(h));
        if (s == "surface")
            return PRC_API_WRITE_RI_SURFACE;
        if (s == "wire")
            return PRC_API_WRITE_RI_WIRE;
    }

    int v = py::cast<int>(h);
    if (v == static_cast<int>(PRC_API_WRITE_RI_SURFACE))
        return PRC_API_WRITE_RI_SURFACE;
    if (v == static_cast<int>(PRC_API_WRITE_RI_WIRE))
        return PRC_API_WRITE_RI_WIRE;
    throw std::runtime_error("invalid rep-item kind");
}

static prc_api_write_tess_kind_t parse_tess_kind(const py::handle &h)
{
    if (py::isinstance<py::str>(h)) {
        std::string s = to_lower_ascii(py::cast<std::string>(h));
        if (s == "triangles")
            return PRC_API_WRITE_TESS_KIND_TRIANGLES;
        if (s == "wire")
            return PRC_API_WRITE_TESS_KIND_WIRE;
        if (s == "compressed")
            return PRC_API_WRITE_TESS_KIND_COMPRESSED;
    }

    int v = py::cast<int>(h);
    if (v == static_cast<int>(PRC_API_WRITE_TESS_KIND_TRIANGLES))
        return PRC_API_WRITE_TESS_KIND_TRIANGLES;
    if (v == static_cast<int>(PRC_API_WRITE_TESS_KIND_WIRE))
        return PRC_API_WRITE_TESS_KIND_WIRE;
    if (v == static_cast<int>(PRC_API_WRITE_TESS_KIND_COMPRESSED))
        return PRC_API_WRITE_TESS_KIND_COMPRESSED;
    throw std::runtime_error("invalid tessellation kind");
}

static std::vector<double> read_vec3d_array(const py::handle &h, const char *field_name)
{
    py::object obj = py::reinterpret_borrow<py::object>(h);
    py::array_t<double, py::array::c_style | py::array::forcecast> arr(obj);
    py::buffer_info info = arr.request();

    if (info.ndim != 2 || info.shape[1] != 3)
        throw std::runtime_error(std::string(field_name) + " must be shape (N,3)");

    const size_t count = static_cast<size_t>(info.shape[0]) * 3;
    const auto *src = static_cast<const double*>(info.ptr);
    return std::vector<double>(src, src + count);
}

static std::vector<uint32_t> read_u32_index_array(const py::handle &h, const char *field_name)
{
    py::object obj = py::reinterpret_borrow<py::object>(h);
    py::array_t<uint32_t, py::array::c_style | py::array::forcecast> arr(obj);
    py::buffer_info info = arr.request();

    if (info.ndim == 1) {
        const size_t count = static_cast<size_t>(info.shape[0]);
        const auto *src = static_cast<const uint32_t*>(info.ptr);
        return std::vector<uint32_t>(src, src + count);
    }

    if (info.ndim == 2 && info.shape[1] == 3) {
        const size_t count = static_cast<size_t>(info.shape[0]) * 3;
        const auto *src = static_cast<const uint32_t*>(info.ptr);
        return std::vector<uint32_t>(src, src + count);
    }

    throw std::runtime_error(std::string(field_name) + " must be shape (N,3) or flat 1D");
}

static std::vector<uint32_t> read_u32_1d_array(const py::handle &h, const char *field_name)
{
    py::object obj = py::reinterpret_borrow<py::object>(h);
    py::array_t<uint32_t, py::array::c_style | py::array::forcecast> arr(obj);
    py::buffer_info info = arr.request();
    if (info.ndim != 1)
        throw std::runtime_error(std::string(field_name) + " must be 1D");

    const size_t count = static_cast<size_t>(info.shape[0]);
    const auto *src = static_cast<const uint32_t*>(info.ptr);
    return std::vector<uint32_t>(src, src + count);
}

static void set_vec3(double dst[3], const py::dict &d, const char *key, const double defaults[3])
{
    if (!d.contains(py::str(key))) {
        dst[0] = defaults[0];
        dst[1] = defaults[1];
        dst[2] = defaults[2];
        return;
    }

    py::sequence seq = py::cast<py::sequence>(d[py::str(key)]);
    if (py::len(seq) != 3)
        throw std::runtime_error(std::string(key) + " must have 3 elements");
    for (int i = 0; i < 3; ++i)
        dst[i] = py::cast<double>(seq[i]);
}

static OwnedWriteTessellation parse_write_tessellation(const py::dict &d)
{
    OwnedWriteTessellation out;
    out.tess.kind = parse_tess_kind(d[py::str("kind")]);

    if (out.tess.kind == PRC_API_WRITE_TESS_KIND_WIRE) {
        throw std::runtime_error("WIRE write tessellations are not yet exposed by this Python binding");
    }

    out.positions = read_vec3d_array(d[py::str("positions")], "positions");
    out.tess.positions = out.positions.data();
    out.tess.num_positions = static_cast<uint32_t>(out.positions.size() / 3);

    if (d.contains(py::str("normals")) && !d[py::str("normals")].is_none()) {
        out.normals = read_vec3d_array(d[py::str("normals")], "normals");
        out.tess.normals = out.normals.data();
        out.tess.num_normals = static_cast<uint32_t>(out.normals.size() / 3);
    }

    out.tri_indices = read_u32_index_array(d[py::str("tri_indices")], "tri_indices");
    if ((out.tri_indices.size() % 3) != 0)
        throw std::runtime_error("tri_indices length must be divisible by 3");
    out.tess.tri_indices = out.tri_indices.data();
    out.tess.num_triangles = static_cast<uint32_t>(out.tri_indices.size() / 3);

    if (d.contains(py::str("norm_indices")) && !d[py::str("norm_indices")].is_none()) {
        out.norm_indices = read_u32_index_array(d[py::str("norm_indices")], "norm_indices");
        if (out.norm_indices.size() != out.tri_indices.size())
            throw std::runtime_error("norm_indices must match tri_indices element count");
        out.tess.norm_indices = out.norm_indices.data();
    } else if (!out.normals.empty()) {
        out.norm_indices = out.tri_indices;
        out.tess.norm_indices = out.norm_indices.data();
    }

    if (d.contains(py::str("face_tri_counts")) && !d[py::str("face_tri_counts")].is_none()) {
        out.face_tri_counts = read_u32_1d_array(d[py::str("face_tri_counts")], "face_tri_counts");
    } else {
        out.face_tri_counts.push_back(out.tess.num_triangles);
    }

    uint64_t face_sum = 0;
    for (uint32_t v : out.face_tri_counts)
        face_sum += v;
    if (face_sum != out.tess.num_triangles)
        throw std::runtime_error("sum(face_tri_counts) must equal num_triangles");

    out.tess.face_tri_counts = out.face_tri_counts.data();
    out.tess.num_faces = static_cast<uint32_t>(out.face_tri_counts.size());

    if (d.contains(py::str("crease_angle_degrees")))
        out.tess.crease_angle_degrees = py::cast<double>(d[py::str("crease_angle_degrees")]);
    if (d.contains(py::str("must_calculate_normals")))
        out.tess.must_calculate_normals = py::cast<bool>(d[py::str("must_calculate_normals")]) ? 1 : 0;

    return out;
}

static std::unique_ptr<OwnedWriteNode> parse_write_node(const py::dict &d)
{
    auto out = std::make_unique<OwnedWriteNode>();

    if (d.contains(py::str("rep_items")) && !d[py::str("rep_items")].is_none()) {
        py::sequence rep_seq = py::cast<py::sequence>(d[py::str("rep_items")]);
        out->rep_items.reserve(static_cast<size_t>(py::len(rep_seq)));

        for (py::handle rep_h : rep_seq) {
            py::dict rep = py::cast<py::dict>(rep_h);
            prc_api_write_rep_item item = {};
            item.kind = parse_ri_kind(rep[py::str("kind")]);
            item.biased_tessellation_index = py::cast<uint32_t>(rep[py::str("biased_tessellation_index")]);
            if (rep.contains(py::str("is_closed")))
                item.is_closed = py::cast<bool>(rep[py::str("is_closed")]) ? 1 : 0;
            out->rep_items.push_back(item);
        }
    }

    out->node.rep_items = out->rep_items.empty() ? nullptr : out->rep_items.data();
    out->node.num_rep_items = static_cast<uint32_t>(out->rep_items.size());

    const double bbox_defaults_min[3] = {0.0, 0.0, 0.0};
    const double bbox_defaults_max[3] = {0.0, 0.0, 0.0};
    set_vec3(out->node.bbox_min, d, "bbox_min", bbox_defaults_min);
    set_vec3(out->node.bbox_max, d, "bbox_max", bbox_defaults_max);

    if (d.contains(py::str("has_empty_part")))
        out->node.has_empty_part = py::cast<bool>(d[py::str("has_empty_part")]) ? 1 : 0;

    if (d.contains(py::str("name")) && !d[py::str("name")].is_none()) {
        out->name_storage = py::cast<std::string>(d[py::str("name")]);
        out->node.name = out->name_storage.c_str();
    }

    if (d.contains(py::str("part_name")) && !d[py::str("part_name")].is_none()) {
        out->part_name_storage = py::cast<std::string>(d[py::str("part_name")]);
        out->node.part_name = out->part_name_storage.c_str();
    }

    if (d.contains(py::str("has_transform")))
        out->node.has_transform = py::cast<bool>(d[py::str("has_transform")]) ? 1 : 0;
    if (d.contains(py::str("is_identity")))
        out->node.is_identity = py::cast<bool>(d[py::str("is_identity")]) ? 1 : 0;

    if (d.contains(py::str("transform")) && !d[py::str("transform")].is_none()) {
        py::sequence seq = py::cast<py::sequence>(d[py::str("transform")]);
        if (py::len(seq) != 16)
            throw std::runtime_error("transform must have 16 elements");
        for (int i = 0; i < 16; ++i)
            out->node.transform[i] = py::cast<double>(seq[i]);
    }

    if (d.contains(py::str("children")) && !d[py::str("children")].is_none()) {
        py::sequence child_seq = py::cast<py::sequence>(d[py::str("children")]);
        out->children_storage.reserve(static_cast<size_t>(py::len(child_seq)));
        for (py::handle child_h : child_seq) {
            out->children_storage.push_back(parse_write_node(py::cast<py::dict>(child_h)));
        }
    }

    out->child_ptrs.reserve(out->children_storage.size());
    for (const auto &child : out->children_storage)
        out->child_ptrs.push_back(&child->node);

    out->node.children = out->child_ptrs.empty() ? nullptr : out->child_ptrs.data();
    out->node.num_children = static_cast<uint32_t>(out->child_ptrs.size());

    return out;
}

class Context;

static void context_write_prc_file(
    Context &ctx,
    const std::string &filename,
    const std::string &model_name,
    const py::dict &root,
    const py::sequence &tess_entries);

static py::bytes context_write_prc_buffer(
    Context &ctx,
    const std::string &model_name,
    const py::dict &root,
    const py::sequence &tess_entries);

static void context_pdf_embed_prc(
    Context &ctx,
    const std::string &pdf_path,
    const py::bytes &prc_bytes,
    py::object options_obj);

class Context {
public:
    Context() : ctx_(prc_api_new_context(nullptr)) {
        if (!ctx_) {
            throw std::runtime_error("Failed to create prc_context");
        }
    }

    ~Context() {
        if (ctx_) {
            (void)prc_api_release_context(ctx_);
            ctx_ = nullptr;
        }
    }

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    Context(Context&& other) noexcept : ctx_(other.ctx_) {
        other.ctx_ = nullptr;
    }

    Context& operator=(Context&& other) noexcept {
        if (this != &other) {
            if (ctx_) {
                (void)prc_api_release_context(ctx_);
            }
            ctx_ = other.ctx_;
            other.ctx_ = nullptr;
        }
        return *this;
    }

    prc_context* raw() const {
        return ctx_;
    }

    void print_error_stack() const {
        prc_api_print_error_stack(ctx_);
    }

private:
    prc_context* ctx_;
};

static std::vector<OwnedWriteTessellation> parse_write_tessellations(const py::sequence &tess_entries)
{
    if (py::len(tess_entries) == 0)
        throw std::runtime_error("tess_entries must contain at least one tessellation");

    std::vector<OwnedWriteTessellation> out;
    out.reserve(static_cast<size_t>(py::len(tess_entries)));
    for (py::handle h : tess_entries) {
        out.push_back(parse_write_tessellation(py::cast<py::dict>(h)));
    }
    return out;
}

static void context_write_prc_file(
    Context &ctx,
    const std::string &filename,
    const std::string &model_name,
    const py::dict &root,
    const py::sequence &tess_entries)
{
    std::vector<OwnedWriteTessellation> tess_owned = parse_write_tessellations(tess_entries);
    std::vector<prc_api_write_tessellation> tess_raw;
    tess_raw.reserve(tess_owned.size());
    for (const auto &t : tess_owned)
        tess_raw.push_back(t.tess);

    std::unique_ptr<OwnedWriteNode> root_owned = parse_write_node(root);
    const char *model_name_c = model_name.empty() ? nullptr : model_name.c_str();

    int code = prc_api_write_prc_file(
        ctx.raw(),
        filename.c_str(),
        model_name_c,
        &root_owned->node,
        tess_raw.data(),
        static_cast<uint32_t>(tess_raw.size()));

    if (code != 0)
        throw std::runtime_error("prc_api_write_prc_file failed");
}

static py::bytes context_write_prc_buffer(
    Context &ctx,
    const std::string &model_name,
    const py::dict &root,
    const py::sequence &tess_entries)
{
    std::vector<OwnedWriteTessellation> tess_owned = parse_write_tessellations(tess_entries);
    std::vector<prc_api_write_tessellation> tess_raw;
    tess_raw.reserve(tess_owned.size());
    for (const auto &t : tess_owned)
        tess_raw.push_back(t.tess);

    std::unique_ptr<OwnedWriteNode> root_owned = parse_write_node(root);
    const char *model_name_c = model_name.empty() ? nullptr : model_name.c_str();

    uint8_t *buf = nullptr;
    size_t out_size = 0;
    int code = prc_api_write_prc_buffer(
        ctx.raw(),
        model_name_c,
        &root_owned->node,
        tess_raw.data(),
        static_cast<uint32_t>(tess_raw.size()),
        &buf,
        &out_size);

    if (code != 0)
        throw std::runtime_error("prc_api_write_prc_buffer failed");

    py::bytes out(reinterpret_cast<const char*>(buf), out_size);
    prc_api_write_prc_buffer_free(ctx.raw(), buf);
    return out;
}

static void context_pdf_embed_prc(
    Context &ctx,
    const std::string &pdf_path,
    const py::bytes &prc_bytes,
    py::object options_obj)
{
    std::string prc = py::cast<std::string>(prc_bytes);

    std::vector<prc_pdf_view_spec> views_storage;
    std::vector<std::string> view_names_storage;
    prc_pdf_write_options options = {};
    const prc_pdf_write_options *options_ptr = nullptr;

    if (!options_obj.is_none()) {
        py::dict d = py::cast<py::dict>(options_obj);
        options_ptr = &options;

        if (d.contains(py::str("page_width_pt")))
            options.page_width_pt = py::cast<double>(d[py::str("page_width_pt")]);
        if (d.contains(py::str("page_height_pt")))
            options.page_height_pt = py::cast<double>(d[py::str("page_height_pt")]);
        if (d.contains(py::str("margin_pt")))
            options.margin_pt = py::cast<double>(d[py::str("margin_pt")]);

        if (d.contains(py::str("views")) && !d[py::str("views")].is_none()) {
            py::sequence views = py::cast<py::sequence>(d[py::str("views")]);
            views_storage.reserve(static_cast<size_t>(py::len(views)));
            view_names_storage.reserve(static_cast<size_t>(py::len(views)));

            for (py::handle vh : views) {
                py::dict vd = py::cast<py::dict>(vh);
                prc_pdf_view_spec v = {};

                if (vd.contains(py::str("name")) && !vd[py::str("name")].is_none()) {
                    view_names_storage.push_back(py::cast<std::string>(vd[py::str("name")]));
                } else {
                    view_names_storage.push_back(std::string());
                }

                auto set_vec3_from_dict = [&](const char *key, double out[3]) {
                    py::sequence seq = py::cast<py::sequence>(vd[py::str(key)]);
                    if (py::len(seq) != 3)
                        throw std::runtime_error(std::string("view ") + key + " must have 3 elements");
                    out[0] = py::cast<double>(seq[0]);
                    out[1] = py::cast<double>(seq[1]);
                    out[2] = py::cast<double>(seq[2]);
                };

                set_vec3_from_dict("eye", v.eye);
                set_vec3_from_dict("target", v.target);
                set_vec3_from_dict("up", v.up);

                if (vd.contains(py::str("is_default")))
                    v.is_default = py::cast<bool>(vd[py::str("is_default")]) ? 1 : 0;

                views_storage.push_back(v);
            }

            for (size_t i = 0; i < views_storage.size(); ++i) {
                if (!view_names_storage[i].empty())
                    views_storage[i].name = view_names_storage[i].c_str();
            }

            if (!views_storage.empty()) {
                options.views = views_storage.data();
                options.num_views = static_cast<uint32_t>(views_storage.size());
            }
        }
    }

    int code = prc_api_pdf_embed_prc(
        ctx.raw(),
        pdf_path.c_str(),
        reinterpret_cast<const uint8_t*>(prc.data()),
        prc.size(),
        options_ptr);

    if (code != 0)
        throw std::runtime_error("prc_api_pdf_embed_prc failed");
}

class Document;

class ModelNode {
public:
    ModelNode(std::shared_ptr<Document> owner, prc_api_product* node)
        : owner_(std::move(owner)), node_(node) {
        children_.reserve(node_->num_children);
        for (size_t i = 0; i < node_->num_children; ++i) {
            children_.push_back(std::make_shared<ModelNode>(owner_, &node_->children[i]));
        }
    }

    std::string name() const {
        return node_->name ? std::string(node_->name) : std::string();
    }

    bool is_model() const {
        return node_->is_model != 0;
    }

    uint32_t node_type() const {
        return static_cast<uint32_t>(node_->type);
    }

    uint32_t num_children() const {
        return static_cast<uint32_t>(node_->num_children);
    }

    py::list children() const {
        py::list result;
        for (const auto& child : children_) {
            result.append(child);
        }
        return result;
    }

    uint32_t num_markups() const {
        return node_->num_markups;
    }

    bool has_part() const {
        return node_->part != nullptr;
    }

    std::string part_name() const {
        return node_->part ? safe_string(node_->part->name) : std::string();
    }

    bool part_name_same_as_product() const {
        return node_->part ? (node_->part->name_same_as_product != 0) : false;
    }

    py::list attributes() const {
        return attributes_to_python(&node_->attributes);
    }

    py::list part_attributes() const {
        return node_->part ? attributes_to_python(&node_->part->attributes) : py::list();
    }

    std::string repr() const {
        return "ModelNode(name='" + name() + "', children=" + std::to_string(num_children()) + ")";
    }

private:
    std::shared_ptr<Document> owner_;
    prc_api_product* node_;
    std::vector<std::shared_ptr<ModelNode>> children_;
};

class View {
public:
    View(std::string name, std::array<double, 16> matrix, double camera_z)
        : name_(std::move(name)), matrix_(matrix), camera_z_(camera_z) {}

    const std::string& name() const {
        return name_;
    }

    const std::array<double, 16>& matrix() const {
        return matrix_;
    }

    double camera_z() const {
        return camera_z_;
    }

private:
    std::string name_;
    std::array<double, 16> matrix_;
    double camera_z_;
};

class Document : public std::enable_shared_from_this<Document> {
public:
    Document(std::shared_ptr<Context> owner, prc_api_data data)
        : owner_(std::move(owner)), data_(data) {}

    ~Document() {
        if (data_ && owner_ && owner_->raw()) {
            prc_api_release_data(
                owner_->raw(),
                data_,
                tessellations_.empty() ? nullptr : tessellations_.data(),
                static_cast<uint32_t>(tessellations_.size()),
                line_tessellations_.empty() ? nullptr : line_tessellations_.data(),
                static_cast<uint32_t>(line_tessellations_.size()),
                model_tree_);
            data_ = nullptr;
            model_tree_ = nullptr;
        }
    }

    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;

    Document(Document&& other) noexcept
        : owner_(std::move(other.owner_)), data_(other.data_), model_tree_(other.model_tree_), num_parts_(other.num_parts_), num_products_(other.num_products_), num_markups_(other.num_markups_), tessellations_(std::move(other.tessellations_)), line_tessellations_(std::move(other.line_tessellations_)), tess_faces_storage_(std::move(other.tess_faces_storage_)), line_tess_faces_storage_(std::move(other.line_tess_faces_storage_)), line_tess_map_(std::move(other.line_tess_map_)), tessellation_initialized_(other.tessellation_initialized_), num_tessellations_(other.num_tessellations_), num_line_tessellations_(other.num_line_tessellations_) {
        other.data_ = nullptr;
        other.model_tree_ = nullptr;
        other.tessellation_initialized_ = false;
        other.num_tessellations_ = 0;
        other.num_line_tessellations_ = 0;
    }

    Document& operator=(Document&& other) noexcept {
        if (this != &other) {
            if (data_ && owner_ && owner_->raw()) {
                prc_api_release_data(
                    owner_->raw(),
                    data_,
                    tessellations_.empty() ? nullptr : tessellations_.data(),
                    static_cast<uint32_t>(tessellations_.size()),
                    line_tessellations_.empty() ? nullptr : line_tessellations_.data(),
                    static_cast<uint32_t>(line_tessellations_.size()),
                    model_tree_);
            }
            owner_ = std::move(other.owner_);
            data_ = other.data_;
            model_tree_ = other.model_tree_;
            num_parts_ = other.num_parts_;
            num_products_ = other.num_products_;
            num_markups_ = other.num_markups_;
            tessellations_ = std::move(other.tessellations_);
            line_tessellations_ = std::move(other.line_tessellations_);
            tess_faces_storage_ = std::move(other.tess_faces_storage_);
            line_tess_faces_storage_ = std::move(other.line_tess_faces_storage_);
            line_tess_map_ = std::move(other.line_tess_map_);
            tessellation_initialized_ = other.tessellation_initialized_;
            num_tessellations_ = other.num_tessellations_;
            num_line_tessellations_ = other.num_line_tessellations_;
            other.data_ = nullptr;
            other.model_tree_ = nullptr;
            other.tessellation_initialized_ = false;
            other.num_tessellations_ = 0;
            other.num_line_tessellations_ = 0;
        }
        return *this;
    }

    prc_context* raw() const {
        return owner_->raw();
    }

    void print_error_stack() const {
        owner_->print_error_stack();
    }

    bool is_open() const {
        return data_ != nullptr;
    }

    uint32_t number_of_views() const {
        return prc_api_get_number_of_view(owner_->raw(), data_);
    }

    View get_view(uint32_t view_index) const {
        char* name = nullptr;
        double* matrix = nullptr;
        double camera_z = 0.0;
        int result = prc_api_get_view(owner_->raw(), data_, view_index, &name, &matrix, &camera_z);
        if (result != 0) {
            throw std::runtime_error("prc_api_get_view failed");
        }

        std::array<double, 16> matrix_array{};
        if (matrix) {
            std::copy_n(matrix, matrix_array.size(), matrix_array.begin());
        }

        return View(name ? std::string(name) : std::string(), matrix_array, camera_z);
    }

    py::tuple prepare_model_tree() {
        uint32_t num_parts = 0;
        uint32_t num_products = 0;
        uint32_t num_markups = 0;
        int result = prc_api_prep_model_tree(owner_->raw(), data_, &num_parts, &num_products, &num_markups);
        if (result != 0) {
            throw std::runtime_error("prc_api_prep_model_tree failed");
        }
        num_parts_ = num_parts;
        num_products_ = num_products;
        num_markups_ = num_markups;
        return py::make_tuple(num_parts, num_products, num_markups);
    }

    bool has_model_tree() const {
        return model_tree_ != nullptr;
    }

    std::shared_ptr<ModelNode> create_model_tree() {
        if (!model_tree_) {
            if (num_parts_ == 0 && num_products_ == 0 && num_markups_ == 0) {
                prepare_model_tree();
            }

            prc_api_product* root = nullptr;
            int result = prc_api_create_model_tree(owner_->raw(), data_, &root, num_parts_, num_products_, num_markups_);
            if (result != 0 || !root) {
                throw std::runtime_error("prc_api_create_model_tree failed");
            }
            model_tree_ = root;
        }
        return std::make_shared<ModelNode>(shared_from_this(), model_tree_);
    }

    py::tuple tessellation_counts() {
        if (!model_tree_) {
            throw std::runtime_error("Model tree must be created before requesting tessellation counts");
        }
        initialize_tessellations();
        return py::make_tuple(num_tessellations_, num_line_tessellations_);
    }

    uint32_t number_of_faces(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        return static_cast<uint32_t>(tess.num_faces);
    }

    uint32_t tessellation_vertex_count(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        return static_cast<uint32_t>(tess.tess_vertices.num_vertices);
    }

    py::array tessellation_vertex_positions(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        py::object owner = py::cast(shared_from_this());
        return make_float_array_from_vertex_field(
            tess.tess_vertices,
            tess.tess_vertices.vertices ? tess.tess_vertices.vertices[0].position : nullptr,
            3,
            owner);
    }

    py::array tessellation_vertex_normals(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        py::object owner = py::cast(shared_from_this());
        return make_float_array_from_vertex_field(
            tess.tess_vertices,
            tess.tess_vertices.vertices ? tess.tess_vertices.vertices[0].normal : nullptr,
            3,
            owner);
    }

    py::array tessellation_vertex_colors(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        py::object owner = py::cast(shared_from_this());
        return make_float_array_from_vertex_field(
            tess.tess_vertices,
            tess.tess_vertices.vertices ? tess.tess_vertices.vertices[0].color : nullptr,
            4,
            owner);
    }

    py::array tessellation_vertex_uvs(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        py::object owner = py::cast(shared_from_this());
        return make_float_array_from_vertex_field(
            tess.tess_vertices,
            tess.tess_vertices.vertices ? tess.tess_vertices.vertices[0].uv : nullptr,
            2,
            owner);
    }

    py::dict tessellation_info(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);

        py::dict result;
        result["name"] = py::str(safe_string(tess.name));
        result["tess_index"] = py::int_(tess.tess_index);
        result["part_index"] = py::int_(tess.part_index);
        result["product_index"] = py::int_(tess.product_index);
        result["mark_up_index"] = py::int_(tess.mark_up_index);
        return result;
    }

    prc_api_tess_vertex_buffer get_face_vertex_buffer(uint32_t tess_index, uint32_t face_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        prc_api_vertex* vertices = nullptr;
        uint32_t num_vertices = 0;
        int result = prc_api_get_face_vertices(owner_->raw(), &tess, face_index, &num_vertices, &vertices);
        if (result != 0) {
            throw std::runtime_error("prc_api_get_face_vertices failed");
        }

        prc_api_tess_vertex_buffer face_vertices;
        face_vertices.num_vertices = num_vertices;
        face_vertices.capacity = num_vertices;
        face_vertices.vertices = vertices;
        return face_vertices;
    }

    uint32_t face_vertex_count(uint32_t tess_index, uint32_t face_index) {
        prc_api_tess_vertex_buffer face_vertices = get_face_vertex_buffer(tess_index, face_index);
        return static_cast<uint32_t>(face_vertices.num_vertices);
    }

    py::array face_vertex_positions(uint32_t tess_index, uint32_t face_index) {
        prc_api_tess_vertex_buffer face_vertices = get_face_vertex_buffer(tess_index, face_index);
        py::object owner = py::cast(shared_from_this());
        return make_float_array_from_vertex_field(
            face_vertices,
            face_vertices.vertices ? face_vertices.vertices[0].position : nullptr,
            3,
            owner);
    }

    py::array face_vertex_normals(uint32_t tess_index, uint32_t face_index) {
        prc_api_tess_vertex_buffer face_vertices = get_face_vertex_buffer(tess_index, face_index);
        py::object owner = py::cast(shared_from_this());
        return make_float_array_from_vertex_field(
            face_vertices,
            face_vertices.vertices ? face_vertices.vertices[0].normal : nullptr,
            3,
            owner);
    }

    py::list face_vertices(uint32_t tess_index, uint32_t face_index) {
        py::array positions = face_vertex_positions(tess_index, face_index);
        py::list result;
        auto buf = positions.request();
        const float* data = static_cast<const float*>(buf.ptr);
        for (size_t i = 0; i < static_cast<size_t>(buf.shape[0]); ++i) {
            result.append(py::make_tuple(
                data[i * 3 + 0],
                data[i * 3 + 1],
                data[i * 3 + 2]
            ));
        }
        return result;
    }

    py::dict face_material(uint32_t tess_index, uint32_t face_index) {
        initialize_tessellations();
        const prc_api_face& face = get_face(get_tessellation(tess_index), face_index);
        py::object owner = py::cast(shared_from_this());
        py::dict material;
        std::vector<py::ssize_t> material_shape = {3};
        std::vector<py::ssize_t> material_strides = { static_cast<py::ssize_t>(sizeof(float)) };
        material["emissive"] = py::array(py::buffer_info(const_cast<float*>(face.material.emissive), sizeof(float), py::format_descriptor<float>::format(), 1, material_shape, material_strides), owner);
        material["diffuse"] = py::array(py::buffer_info(const_cast<float*>(face.material.diffuse), sizeof(float), py::format_descriptor<float>::format(), 1, material_shape, material_strides), owner);
        material["specular"] = py::array(py::buffer_info(const_cast<float*>(face.material.specular), sizeof(float), py::format_descriptor<float>::format(), 1, material_shape, material_strides), owner);
        material["ambient"] = py::array(py::buffer_info(const_cast<float*>(face.material.ambient), sizeof(float), py::format_descriptor<float>::format(), 1, material_shape, material_strides), owner);
        material["shininess"] = face.material.shininess;
        material["emissive_alpha"] = face.material.emissive_alpha;
        material["diffuse_alpha"] = face.material.diffuse_alpha;
        material["specular_alpha"] = face.material.specular_alpha;
        material["ambient_alpha"] = face.material.ambient_alpha;

        py::dict result;
        result["is_material"] = face.is_material != 0;
        result["has_transparency"] = face.has_transparency != 0;
        result["is_texture"] = face.is_texture != 0;
        result["material"] = material;
        return result;
    }

    size_t face_graphics_primitive_count(uint32_t tess_index, uint32_t face_index) {
        initialize_tessellations();
        const prc_api_face& face = get_face(get_tessellation(tess_index), face_index);
        return face.num_graphic_primitives;
    }

    py::dict get_graphics_primitive(uint32_t tess_index, uint32_t face_index, size_t primitive_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        prc_api_graphic_primitive primitive = {};
        int rc = prc_api_get_graphics_primitive(owner_->raw(), data_, &tess, face_index, primitive_index, &primitive);
        if (rc != 0) {
            throw std::runtime_error("prc_api_get_graphics_primitive failed");
        }
        py::dict result;
        result["type"] = static_cast<uint32_t>(primitive.type);
        result["indices"] = make_uint32_array(primitive.indices, primitive.num_indices, py::cast(shared_from_this()));
        return result;
    }

    uint32_t number_of_text_primitives(uint32_t tess_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        return static_cast<uint32_t>(tess.num_text_primitives);
    }

    py::dict get_text_primitive(uint32_t tess_index, uint32_t text_index) {
        initialize_tessellations();
        const prc_api_tess& tess = get_tessellation(tess_index);
        prc_api_text_primitive primitive = {};
        int rc = prc_api_get_text_primitive(owner_->raw(), data_, &tess, text_index, &primitive);
        if (rc != 0) {
            throw std::runtime_error("prc_api_get_text_primitive failed");
        }
        py::dict result;
        result["text"] = safe_string(primitive.text);
        result["text_height"] = primitive.text_height;
        result["text_width"] = primitive.text_width;
        result["origin"] = py::make_tuple(primitive.origin[0], primitive.origin[1], primitive.origin[2]);
        py::array_t<double> color(4);
        auto color_buf = color.mutable_unchecked<1>();
        for (size_t i = 0; i < 4; ++i) {
            color_buf(i) = primitive.color[i];
        }
        result["color"] = std::move(color);
        result["mode"] = static_cast<uint32_t>(primitive.mode);
        return result;
    }

private:
    void initialize_tessellations() {
        if (tessellation_initialized_) {
            return;
        }

        if (!model_tree_) {
            create_model_tree();
        }

        uint32_t num_tess = 0;
        uint32_t num_line_tess = 0;
        int result = prc_api_get_number_tessellations(owner_->raw(), data_, model_tree_, &num_tess, &num_line_tess);
        if (result != 0) {
            throw std::runtime_error("prc_api_get_number_tessellations failed");
        }

        num_tessellations_ = num_tess;
        num_line_tessellations_ = num_line_tess;
        tessellations_.assign(num_tessellations_, prc_api_tess{});
        line_tessellations_.assign(num_line_tessellations_, prc_api_tess{});
        tess_faces_storage_.clear();
        tess_faces_storage_.reserve(num_tessellations_);
        line_tess_faces_storage_.clear();
        line_tess_faces_storage_.reserve(num_line_tessellations_);
        line_tess_map_.assign(num_tessellations_, -1);

        uint32_t line_tess_index = 0;
        for (uint32_t tess_index = 0; tess_index < num_tessellations_; ++tess_index) {
            uint32_t faces = prc_api_get_number_faces(owner_->raw(), data_, tess_index);
            tessellations_[tess_index].num_faces = faces;
            tess_faces_storage_.emplace_back(faces);
            tessellations_[tess_index].tess_faces = faces ? tess_faces_storage_.back().data() : nullptr;

            prc_api_tess temp_line_tess{};
            std::vector<prc_api_face> temp_line_faces;
            prc_api_tess* line_ptr = nullptr;
            if (num_line_tessellations_ > 0) {
                temp_line_tess.num_faces = faces;
                temp_line_faces.resize(faces);
                temp_line_tess.tess_faces = faces ? temp_line_faces.data() : nullptr;
                line_ptr = &temp_line_tess;
            }

            uint8_t has_line = 0;
            int code = prc_api_initialize_tessellation(owner_->raw(), data_, model_tree_, tess_index, &tessellations_[tess_index], line_ptr, &has_line);
            if (code != 0) {
                throw std::runtime_error("prc_api_initialize_tessellation failed");
            }

            if (has_line) {
                if (line_tess_index >= num_line_tessellations_) {
                    throw std::runtime_error("Unexpected line tessellation count mismatch");
                }
                line_tess_map_[tess_index] = static_cast<int32_t>(line_tess_index);
                line_tessellations_[line_tess_index] = temp_line_tess;
                line_tess_faces_storage_.push_back(std::move(temp_line_faces));
                line_tessellations_[line_tess_index].tess_faces = line_tess_faces_storage_.back().data();

                for (uint32_t face_index = 0; face_index < faces; ++face_index) {
                    code = prc_api_get_line_tessellation_vertices(owner_->raw(), data_, model_tree_, tess_index, face_index,
                        &line_tessellations_[line_tess_index].tess_faces[face_index], &line_tessellations_[line_tess_index]);
                    if (code != 0) {
                        throw std::runtime_error("prc_api_get_line_tessellation_vertices failed");
                    }
                }

                ++line_tess_index;
            }

            if (tessellations_[tess_index].type == PRC_API_TESS_3D_Wire || tessellations_[tess_index].type == PRC_API_TESS_MarkUp) {
                code = prc_api_get_tessellation_vertices(owner_->raw(), data_, model_tree_, tess_index, 0, nullptr, &tessellations_[tess_index]);
                if (code != 0) {
                    throw std::runtime_error("prc_api_get_tessellation_vertices failed");
                }
            } else {
                for (uint32_t face_index = 0; face_index < faces; ++face_index) {
                    code = prc_api_get_tessellation_vertices(owner_->raw(), data_, model_tree_, tess_index, face_index,
                        &tessellations_[tess_index].tess_faces[face_index], &tessellations_[tess_index]);
                    if (code != 0) {
                        throw std::runtime_error("prc_api_get_tessellation_vertices failed");
                    }
                }
            }
        }

        tessellation_initialized_ = true;
    }

    const prc_api_tess& get_tessellation(uint32_t tess_index) const {
        if (tess_index >= num_tessellations_) {
            throw std::out_of_range("Tessellation index out of range");
        }
        return tessellations_[tess_index];
    }

    const prc_api_face& get_face(const prc_api_tess& tess, uint32_t face_index) const {
        if (face_index >= tess.num_faces) {
            throw std::out_of_range("Face index out of range");
        }
        return tess.tess_faces[face_index];
    }

private:
    std::shared_ptr<Context> owner_;
    prc_api_data data_;
    prc_api_product* model_tree_ = nullptr;
    uint32_t num_parts_ = 0;
    uint32_t num_products_ = 0;
    uint32_t num_markups_ = 0;
    std::vector<prc_api_tess> tessellations_;
    std::vector<prc_api_tess> line_tessellations_;
    std::vector<std::vector<prc_api_face>> tess_faces_storage_;
    std::vector<std::vector<prc_api_face>> line_tess_faces_storage_;
    std::vector<int32_t> line_tess_map_;
    bool tessellation_initialized_ = false;
    uint32_t num_tessellations_ = 0;
    uint32_t num_line_tessellations_ = 0;
};

std::shared_ptr<Document> context_open(const std::shared_ptr<Context>& ctx, const std::string& infile) {
    if (!ctx || !ctx->raw()) {
        throw std::runtime_error("Context is not initialized");
    }

    prc_api_data data = prc_api_open_contents(ctx->raw(), infile.c_str());
    if (!data) {
        throw std::runtime_error("prc_api_open_contents failed");
    }

    return std::make_shared<Document>(ctx, data);
}

} // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() = "Initial nanoPRC Python bindings";

    m.attr("PRC_API_ERROR_MEMORY") = py::int_(PRC_API_ERROR_MEMORY);
    m.attr("PRC_API_ERROR_PARAMETER") = py::int_(PRC_API_ERROR_PARAMETER);
    m.attr("PRC_API_ERROR_PARSER") = py::int_(PRC_API_ERROR_PARSER);
    m.attr("PRC_API_ERROR_UNSUPPORTED") = py::int_(PRC_API_ERROR_UNSUPPORTED);
        m.attr("PRC_API_WRITE_RI_SURFACE") = py::int_(static_cast<int>(PRC_API_WRITE_RI_SURFACE));
        m.attr("PRC_API_WRITE_RI_WIRE") = py::int_(static_cast<int>(PRC_API_WRITE_RI_WIRE));
        m.attr("PRC_API_WRITE_TESS_KIND_TRIANGLES") = py::int_(static_cast<int>(PRC_API_WRITE_TESS_KIND_TRIANGLES));
        m.attr("PRC_API_WRITE_TESS_KIND_WIRE") = py::int_(static_cast<int>(PRC_API_WRITE_TESS_KIND_WIRE));
        m.attr("PRC_API_WRITE_TESS_KIND_COMPRESSED") = py::int_(static_cast<int>(PRC_API_WRITE_TESS_KIND_COMPRESSED));

    py::class_<Context, std::shared_ptr<Context>>(m, "Context")
        .def(py::init<>())
        .def("open", &context_open, py::arg("infile"), "Open PRC/PDF contents")
        .def("print_error_stack", &Context::print_error_stack,
               "Print current nanoPRC error stack to stdout")
           .def("write_prc_file", &context_write_prc_file,
               py::arg("filename"), py::arg("model_name"), py::arg("root"), py::arg("tess_entries"),
               "Write a PRC file using write-node/tessellation dictionaries.")
           .def("write_prc_buffer", &context_write_prc_buffer,
               py::arg("model_name"), py::arg("root"), py::arg("tess_entries"),
               "Encode PRC bytes in memory and return them as Python bytes.")
           .def("pdf_embed_prc", &context_pdf_embed_prc,
               py::arg("pdf_path"), py::arg("prc_bytes"), py::arg("options") = py::none(),
               "Embed PRC bytes into a single-page 3D PDF file.");

    py::class_<View>(m, "View")
        .def_property_readonly("name", &View::name)
        .def_property_readonly("matrix", &View::matrix)
        .def_property_readonly("camera_z", &View::camera_z);

    py::class_<ModelNode, std::shared_ptr<ModelNode>>(m, "ModelNode")
        .def_property_readonly("name", &ModelNode::name)
        .def_property_readonly("is_model", &ModelNode::is_model)
        .def_property_readonly("node_type", &ModelNode::node_type)
        .def_property_readonly("num_children", &ModelNode::num_children)
        .def_property_readonly("num_markups", &ModelNode::num_markups)
        .def_property_readonly("has_part", &ModelNode::has_part)
           .def_property_readonly("part_name", &ModelNode::part_name)
           .def_property_readonly("part_name_same_as_product", &ModelNode::part_name_same_as_product)
           .def("attributes", &ModelNode::attributes,
               "Return model-node attributes as a list of {base_title, entries[]} dictionaries.")
           .def("part_attributes", &ModelNode::part_attributes,
               "Return part attributes as a list of {base_title, entries[]} dictionaries.")
        .def("children", &ModelNode::children)
        .def("__repr__", &ModelNode::repr);

    py::class_<Document, std::shared_ptr<Document>>(m, "Document")
        .def_property_readonly("is_open", &Document::is_open)
        .def_property_readonly("number_of_views", &Document::number_of_views)
        .def("get_view", &Document::get_view, py::arg("view_index"), "Return metadata for a named view.")
        .def("prepare_model_tree", &Document::prepare_model_tree,
             "Compute model tree counts required before tree creation.")
        .def("has_model_tree", &Document::has_model_tree,
             "Return whether a model tree has already been created.")
        .def("create_model_tree", &Document::create_model_tree,
             "Create the model tree and return its root node.")
        .def("tessellation_counts", &Document::tessellation_counts,
             "Return a tuple (num_tess, num_line_tess) after the model tree exists.")
        .def("number_of_faces", &Document::number_of_faces, py::arg("tess_index"),
             "Return the number of faces in a tessellation.")
        .def("tessellation_vertex_count", &Document::tessellation_vertex_count, py::arg("tess_index"),
             "Return the number of vertices in the tessellation vertex buffer.")
        .def("tessellation_vertex_positions", &Document::tessellation_vertex_positions, py::arg("tess_index"),
             "Return a zero-copy (n,3) float array of tessellation vertex positions.")
        .def("tessellation_vertex_normals", &Document::tessellation_vertex_normals, py::arg("tess_index"),
             "Return a zero-copy (n,3) float array of tessellation vertex normals.")
        .def("tessellation_vertex_colors", &Document::tessellation_vertex_colors, py::arg("tess_index"),
             "Return a zero-copy (n,4) float array of tessellation vertex colors.")
        .def("tessellation_vertex_uvs", &Document::tessellation_vertex_uvs, py::arg("tess_index"),
             "Return a zero-copy (n,2) float array of tessellation vertex UV coordinates.")
           .def("tessellation_info", &Document::tessellation_info, py::arg("tess_index"),
               "Return metadata for one tessellation (name and source indices).")
        .def("face_vertex_count", &Document::face_vertex_count,
             py::arg("tess_index"), py::arg("face_index"),
             "Return the number of vertices in a tessellation face.")
        .def("face_vertex_positions", &Document::face_vertex_positions,
             py::arg("tess_index"), py::arg("face_index"),
             "Return a zero-copy (m,3) float array of face vertex positions.")
           .def("face_vertex_normals", &Document::face_vertex_normals,
               py::arg("tess_index"), py::arg("face_index"),
               "Return a zero-copy (m,3) float array of face vertex normals.")
        .def("face_vertices", &Document::face_vertices,
             py::arg("tess_index"), py::arg("face_index"),
             "Return a list of vertex positions for a tessellation face.")
        .def("face_material", &Document::face_material,
             py::arg("tess_index"), py::arg("face_index"),
             "Return material and style metadata for a tessellation face.")
        .def("face_graphics_primitive_count", &Document::face_graphics_primitive_count,
             py::arg("tess_index"), py::arg("face_index"),
             "Return the number of graphics primitives for a tessellation face.")
        .def("get_graphics_primitive", &Document::get_graphics_primitive,
             py::arg("tess_index"), py::arg("face_index"), py::arg("primitive_index"),
             "Return a graphics primitive descriptor with index buffer.")
        .def("number_of_text_primitives", &Document::number_of_text_primitives,
             py::arg("tess_index"),
             "Return the number of text primitives for a tessellation.")
        .def("get_text_primitive", &Document::get_text_primitive,
             py::arg("tess_index"), py::arg("text_index"),
             "Return text primitive metadata for a tessellation.");
}
