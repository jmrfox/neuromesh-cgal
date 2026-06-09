#pragma once

#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/helpers.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "defaults.hpp"

struct MergeOptions {
    std::size_t min_faces = neuromesh::MergeDefaults::min_faces;
    std::size_t min_spine_faces = neuromesh::MergeDefaults::min_spine_faces;
    std::size_t bridge_max_faces = neuromesh::MergeDefaults::bridge_max_faces;
    double spine_sdf_percentile = neuromesh::MergeDefaults::spine_sdf_percentile;
    std::optional<std::size_t> soma_id;
    std::size_t max_passes = neuromesh::MergeDefaults::max_passes;
};

struct MergeRecord {
    std::size_t source = 0;
    std::size_t target = 0;
    const char* rule = "";
};

struct MergeResult {
    std::size_t segments_before = 0;
    std::size_t segments_after = 0;
    std::size_t soma_id = 0;
    double spine_sdf_threshold = 0.0;
    std::vector<MergeRecord> merges;
};

struct UnionFind {
    std::vector<std::size_t> parent;
    std::vector<std::size_t> rank;

    explicit UnionFind(std::size_t n) : parent(n), rank(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }

    std::size_t find(std::size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    void unite(std::size_t root_target, std::size_t root_source) {
        root_target = find(root_target);
        root_source = find(root_source);
        if (root_target == root_source) {
            return;
        }
        if (rank[root_target] < rank[root_source]) {
            std::swap(root_target, root_source);
        }
        parent[root_source] = root_target;
        if (rank[root_target] == rank[root_source]) {
            ++rank[root_target];
        }
    }
};

struct SegmentInfo {
    std::size_t id = 0;
    std::size_t face_count = 0;
    double median_sdf = 0.0;
    std::size_t degree = 0;
    std::map<std::size_t, std::size_t> neighbors;
};

template<typename Mesh, typename SegmentMap>
std::size_t label_of(
    typename Mesh::Face_index face,
    const SegmentMap& segment_map,
    UnionFind& uf) {
    return uf.find(segment_map[face]);
}

template<typename Mesh, typename SegmentMap, typename SdfMap>
std::vector<SegmentInfo> build_segment_stats(
    const Mesh& mesh,
    const SegmentMap& segment_map,
    const SdfMap& sdf_map,
    UnionFind& uf,
    std::size_t num_labels) {
    using FaceIndex = typename Mesh::Face_index;
    std::vector<std::vector<double>> sdf_values(num_labels);
    std::vector<std::size_t> face_counts(num_labels, 0);

    for (FaceIndex f : mesh.faces()) {
        const std::size_t label = label_of<Mesh>(f, segment_map, uf);
        if (label >= num_labels) {
            continue;
        }
        ++face_counts[label];
        sdf_values[label].push_back(sdf_map[f]);
    }

    std::map<std::pair<std::size_t, std::size_t>, std::size_t> edge_counts;
    for (typename Mesh::Edge_index e : mesh.edges()) {
        const typename Mesh::Halfedge_index h = mesh.halfedge(e);
        if (mesh.is_border(h)) {
            continue;
        }

        const FaceIndex f1 = mesh.face(h);
        const FaceIndex f2 = mesh.face(mesh.opposite(h));
        if (f1 == Mesh::null_face() || f2 == Mesh::null_face()) {
            continue;
        }

        std::size_t s1 = label_of<Mesh>(f1, segment_map, uf);
        std::size_t s2 = label_of<Mesh>(f2, segment_map, uf);
        if (s1 == s2) {
            continue;
        }
        if (s1 > s2) {
            std::swap(s1, s2);
        }
        ++edge_counts[{s1, s2}];
    }

    std::vector<SegmentInfo> stats;
    stats.reserve(num_labels);
    for (std::size_t id = 0; id < num_labels; ++id) {
        if (face_counts[id] == 0) {
            continue;
        }

        SegmentInfo info;
        info.id = id;
        info.face_count = face_counts[id];

        auto& values = sdf_values[id];
        if (!values.empty()) {
            std::sort(values.begin(), values.end());
            const std::size_t mid = values.size() / 2;
            if (values.size() % 2 == 0) {
                info.median_sdf = 0.5 * (values[mid - 1] + values[mid]);
            } else {
                info.median_sdf = values[mid];
            }
        }

        stats.push_back(info);
    }

    for (const auto& [edge, count] : edge_counts) {
        const std::size_t s1 = edge.first;
        const std::size_t s2 = edge.second;
        for (SegmentInfo& info : stats) {
            if (info.id == s1) {
                info.neighbors[s2] += count;
            } else if (info.id == s2) {
                info.neighbors[s1] += count;
            }
        }
    }

    for (SegmentInfo& info : stats) {
        info.degree = info.neighbors.size();
    }

    return stats;
}

inline double compute_percentile(std::vector<double> values, double percentile) {
    if (values.empty()) {
        return 0.0;
    }
    percentile = std::clamp(percentile, 0.0, 100.0);
    if (values.size() == 1) {
        return values.front();
    }
    const double rank = (percentile / 100.0) * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(rank));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(rank));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(lower), values.end());
    const double lower_value = values[lower];
    if (lower == upper) {
        return lower_value;
    }
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(upper), values.end());
    const double upper_value = values[upper];
    const double fraction = rank - static_cast<double>(lower);
    return lower_value + fraction * (upper_value - lower_value);
}

inline double compute_spine_sdf_threshold(
    const std::vector<SegmentInfo>& stats,
    double percentile) {
    std::vector<double> medians;
    medians.reserve(stats.size());
    for (const SegmentInfo& info : stats) {
        medians.push_back(info.median_sdf);
    }
    return compute_percentile(std::move(medians), percentile);
}

inline std::size_t detect_soma_id(const std::vector<SegmentInfo>& stats) {
    std::size_t soma = 0;
    std::size_t max_faces = 0;
    for (const SegmentInfo& info : stats) {
        if (info.face_count > max_faces) {
            max_faces = info.face_count;
            soma = info.id;
        }
    }
    return soma;
}

inline const SegmentInfo* find_stat(const std::vector<SegmentInfo>& stats, std::size_t id) {
    for (const SegmentInfo& info : stats) {
        if (info.id == id) {
            return &info;
        }
    }
    return nullptr;
}

inline bool is_spine_leaf(
    const SegmentInfo& info,
    double spine_sdf_threshold,
    std::size_t min_spine_faces) {
    return info.degree == 1 &&
           info.median_sdf < spine_sdf_threshold &&
           info.face_count >= min_spine_faces;
}

inline bool is_bridge(
    const SegmentInfo& info,
    const std::vector<SegmentInfo>& stats,
    std::size_t bridge_max_faces,
    std::size_t soma_id) {
    if (info.face_count >= bridge_max_faces || info.degree < 2) {
        return false;
    }

    std::size_t large_neighbors = 0;
    bool has_soma_neighbor = false;
    for (const auto& [neighbor_id, edge_count] : info.neighbors) {
        (void)edge_count;
        if (neighbor_id == soma_id) {
            has_soma_neighbor = true;
        }
        const SegmentInfo* neighbor = find_stat(stats, neighbor_id);
        if (neighbor != nullptr && neighbor->face_count > bridge_max_faces) {
            ++large_neighbors;
        }
    }

    return large_neighbors >= 2 || has_soma_neighbor;
}

inline std::size_t choose_neighbor_by_boundary(
    const SegmentInfo& info,
    const std::vector<SegmentInfo>& stats,
    const std::vector<std::size_t>& candidates) {
    std::size_t best = candidates.front();
    std::size_t best_edges = 0;
    double best_sdf_delta = std::numeric_limits<double>::infinity();

    for (std::size_t candidate : candidates) {
        const auto it = info.neighbors.find(candidate);
        const std::size_t edge_count = (it != info.neighbors.end()) ? it->second : 0;
        const SegmentInfo* neighbor = find_stat(stats, candidate);
        const double sdf_delta = neighbor != nullptr
            ? std::abs(neighbor->median_sdf - info.median_sdf)
            : std::numeric_limits<double>::infinity();

        if (edge_count > best_edges ||
            (edge_count == best_edges && sdf_delta < best_sdf_delta)) {
            best = candidate;
            best_edges = edge_count;
            best_sdf_delta = sdf_delta;
        }
    }
    return best;
}

inline std::size_t choose_bridge_target(
    const SegmentInfo& info,
    const std::vector<SegmentInfo>& stats,
    std::size_t soma_id) {
    std::vector<std::size_t> candidates;
    candidates.reserve(info.neighbors.size());
    for (const auto& [neighbor_id, edge_count] : info.neighbors) {
        (void)edge_count;
        if (neighbor_id == info.id) {
            continue;
        }
        candidates.push_back(neighbor_id);
    }

    const bool has_soma = std::find(candidates.begin(), candidates.end(), soma_id) != candidates.end();
    if (has_soma && candidates.size() >= 2) {
        std::vector<std::size_t> non_soma;
        for (std::size_t id : candidates) {
            if (id != soma_id) {
                non_soma.push_back(id);
            }
        }
        if (non_soma.size() == 1) {
            return non_soma.front();
        }
        return choose_neighbor_by_boundary(info, stats, non_soma);
    }

    std::size_t best = candidates.front();
    double best_sdf = std::numeric_limits<double>::infinity();
    std::size_t best_faces = 0;
    for (std::size_t candidate : candidates) {
        const SegmentInfo* neighbor = find_stat(stats, candidate);
        if (neighbor == nullptr) {
            continue;
        }
        if (neighbor->median_sdf < best_sdf ||
            (neighbor->median_sdf == best_sdf && neighbor->face_count > best_faces)) {
            best = candidate;
            best_sdf = neighbor->median_sdf;
            best_faces = neighbor->face_count;
        }
    }
    return best;
}

inline std::optional<std::pair<std::size_t, std::size_t>> evaluate_merge(
    const SegmentInfo& info,
    const std::vector<SegmentInfo>& stats,
    const MergeOptions& opts,
    std::size_t soma_id,
    double spine_sdf_threshold) {
    if (info.neighbors.empty()) {
        return std::nullopt;
    }

    if (is_spine_leaf(info, spine_sdf_threshold, opts.min_spine_faces)) {
        return std::nullopt;
    }

    if (info.face_count < opts.min_faces) {
        std::vector<std::size_t> candidates;
        candidates.reserve(info.neighbors.size());
        for (const auto& [neighbor_id, edge_count] : info.neighbors) {
            (void)edge_count;
            candidates.push_back(neighbor_id);
        }
        const std::size_t target = choose_neighbor_by_boundary(info, stats, candidates);
        return std::make_pair(info.id, target);
    }

    if (is_bridge(info, stats, opts.bridge_max_faces, soma_id)) {
        const std::size_t target = choose_bridge_target(info, stats, soma_id);
        return std::make_pair(info.id, target);
    }

    if (info.degree == 1 &&
        info.median_sdf >= spine_sdf_threshold &&
        info.face_count < opts.bridge_max_faces) {
        const std::size_t target = info.neighbors.begin()->first;
        return std::make_pair(info.id, target);
    }

    return std::nullopt;
}

template<typename Mesh, typename SegmentMap, typename SdfMap>
MergeResult merge_segments(
    const Mesh& mesh,
    SegmentMap& segment_map,
    const SdfMap& sdf_map,
    const MergeOptions& opts) {
    using FaceIndex = typename Mesh::Face_index;
    MergeResult result;

    std::size_t max_label = 0;
    for (FaceIndex f : mesh.faces()) {
        max_label = std::max(max_label, segment_map[f]);
    }
    const std::size_t num_labels = max_label + 1;

    {
        std::vector<bool> seen(num_labels, false);
        for (FaceIndex f : mesh.faces()) {
            seen[segment_map[f]] = true;
        }
        result.segments_before = static_cast<std::size_t>(
            std::count(seen.begin(), seen.end(), true));
    }
    UnionFind uf(num_labels);
    std::size_t soma_id = opts.soma_id.value_or(0);
    double spine_sdf_threshold = 0.0;

    for (std::size_t pass = 0; pass < opts.max_passes; ++pass) {
        std::vector<SegmentInfo> stats = build_segment_stats<Mesh>(mesh, segment_map, sdf_map, uf, num_labels);
        if (stats.empty()) {
            break;
        }

        soma_id = opts.soma_id.value_or(detect_soma_id(stats));
        spine_sdf_threshold = compute_spine_sdf_threshold(stats, opts.spine_sdf_percentile);

        std::vector<SegmentInfo> sorted_stats = stats;
        std::sort(sorted_stats.begin(), sorted_stats.end(),
            [](const SegmentInfo& a, const SegmentInfo& b) {
                return a.face_count < b.face_count;
            });

        bool changed = false;
        for (const SegmentInfo& info : sorted_stats) {
            const std::optional<std::pair<std::size_t, std::size_t>> decision =
                evaluate_merge(info, stats, opts, soma_id, spine_sdf_threshold);
            if (!decision.has_value()) {
                continue;
            }

            const std::size_t source_root = uf.find(decision->first);
            const std::size_t target_root = uf.find(decision->second);
            if (source_root == target_root) {
                continue;
            }

            const char* rule = "unknown";
            if (info.face_count < opts.min_faces) {
                rule = "artifact";
            } else if (is_bridge(info, stats, opts.bridge_max_faces, soma_id)) {
                rule = "bridge";
            } else {
                rule = "leaf_stub";
            }

            uf.unite(target_root, source_root);
            result.merges.push_back({decision->first, decision->second, rule});
            changed = true;
        }

        if (!changed) {
            break;
        }
    }

    result.soma_id = soma_id;
    result.spine_sdf_threshold = spine_sdf_threshold;

    std::map<std::size_t, std::size_t> root_to_compact;
    for (FaceIndex f : mesh.faces()) {
        const std::size_t root = uf.find(segment_map[f]);
        if (root_to_compact.find(root) == root_to_compact.end()) {
            root_to_compact[root] = root_to_compact.size();
        }
        segment_map[f] = root_to_compact[root];
    }

    result.segments_after = root_to_compact.size();
    return result;
}

inline bool write_merge_log(const std::string& path, const MergeResult& result, const MergeOptions& opts) {
    std::ofstream out(path);
    if (!out) {
        return false;
    }

    out << "# neuromesh-merge v1\n";
    out << "segments_before " << result.segments_before << "\n";
    out << "segments_after " << result.segments_after << "\n";
    out << "soma_id " << result.soma_id << "\n";
    out << "spine_sdf_threshold " << result.spine_sdf_threshold << "\n";
    out << "min_faces " << opts.min_faces << "\n";
    out << "min_spine_faces " << opts.min_spine_faces << "\n";
    out << "bridge_max_faces " << opts.bridge_max_faces << "\n";
    out << "spine_sdf_percentile " << opts.spine_sdf_percentile << "\n";
    out << "merge_count " << result.merges.size() << "\n";
    out << "\n# source target rule\n";
    for (const MergeRecord& record : result.merges) {
        out << record.source << " " << record.target << " " << record.rule << "\n";
    }
    return true;
}
