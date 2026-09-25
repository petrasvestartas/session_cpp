#pragma once
#include "json.h"
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace session_cpp {
namespace file_encoders {

// ═══════════════════════════════════════════════════════════════════════════
// JSON string
// ═══════════════════════════════════════════════════════════════════════════
/// Serialize an object to a JSON string.
template <typename T>
inline std::string file_json_dumps(const T& data, bool pretty = true) {

    const nlohmann::ordered_json json = data.jsondump();

    return pretty ? json.dump(4) : json.dump();
}

/// Deserialize an object from a JSON string.
template <typename T>
inline T file_json_loads(const std::string& json_str) {

    const nlohmann::json json = nlohmann::json::parse(json_str);

    return T::jsonload(json);
}

// ═══════════════════════════════════════════════════════════════════════════
// JSON file
// ═══════════════════════════════════════════════════════════════════════════
/// Write a JSON value to a file.
inline void file_json_dump(const nlohmann::ordered_json& data, const std::string& filepath, bool pretty = true) {

    std::ofstream file(filepath);

    if (!file.is_open())
        throw std::runtime_error("Failed to open file for writing: " + filepath);

    file << (pretty ? data.dump(4) : data.dump());

    if (!file.good())
        throw std::runtime_error("Failed to write file: " + filepath);
}

/// Write an object to a JSON file.
template <typename T>
inline void file_json_dump(const T& data, const std::string& filepath, bool pretty = true) {
    file_json_dump(data.jsondump(), filepath, pretty);
}

/// Read a JSON value from a file.
inline nlohmann::ordered_json file_json_load_data(const std::string& filepath) {

    std::ifstream file(filepath);

    if (!file.is_open())
        throw std::runtime_error("Failed to open file for reading: " + filepath);

    nlohmann::ordered_json json;
    file >> json;

    return json;
}

/// Read an object from a JSON file.
template <typename T>
inline T file_json_load(const std::string& filepath) {

    const nlohmann::ordered_json json = file_json_load_data(filepath);

    return T::jsonload(json);
}

// ═══════════════════════════════════════════════════════════════════════════
// Collections
// ═══════════════════════════════════════════════════════════════════════════
/// Encode a collection of objects to a JSON array.
template <typename T>
inline nlohmann::ordered_json file_encode_collection(const std::vector<T>& collection) {

    nlohmann::ordered_json array = nlohmann::ordered_json::array();

    for (const T& item : collection)
        array.push_back(item.jsondump());

    return array;
}

/// Encode a collection of shared pointers to a JSON array, skipping null entries.
template <typename T>
inline nlohmann::ordered_json file_encode_collection(const std::vector<std::shared_ptr<T>>& collection) {

    nlohmann::ordered_json array = nlohmann::ordered_json::array();

    for (const std::shared_ptr<T>& item : collection)
        if (item)
            array.push_back(item->jsondump());

    return array;
}

/// Decode a JSON array to a collection of objects.
template <typename T>
inline std::vector<T> file_decode_collection(const nlohmann::json& data) {

    std::vector<T> result;

    if (!data.is_array())
        return result;

    for (const nlohmann::json& item : data)
        result.push_back(T::jsonload(item));

    return result;
}

/// Decode a JSON array to a collection of shared pointers.
template <typename T>
inline std::vector<std::shared_ptr<T>> file_decode_collection_ptr(const nlohmann::json& data) {

    std::vector<std::shared_ptr<T>> result;

    if (!data.is_array())
        return result;

    for (const nlohmann::json& item : data)
        result.push_back(std::make_shared<T>(T::jsonload(item)));

    return result;
}

} // namespace file_encoders
} // namespace session_cpp
