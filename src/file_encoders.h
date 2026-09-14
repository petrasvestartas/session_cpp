#pragma once
#include "json.h"
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace session_cpp {
namespace file_encoders {

/// Write json to a file
inline void file_json_dump(const nlohmann::ordered_json& j, const std::string& filepath, bool pretty = true) {
  std::ofstream ofs(filepath);
  if (!ofs.is_open())
    throw std::runtime_error("Failed to open file for writing: " + filepath);
  ofs << (pretty ? j.dump(2) : j.dump());
}

/// Read json from a file
inline nlohmann::ordered_json file_json_load_data(const std::string& filepath) {
  std::ifstream ifs(filepath);
  if (!ifs.is_open())
    throw std::runtime_error("Failed to open file for reading: " + filepath);
  nlohmann::ordered_json j;
  ifs >> j;
  return j;
}

/// Serialize an object to a json string
template <typename T>
inline std::string file_json_dumps(const T& obj, bool pretty = true) {
  nlohmann::ordered_json j = obj.jsondump();
  return pretty ? j.dump(2) : j.dump();
}

/// Deserialize an object from a json string
template <typename T>
inline auto file_json_loads(const std::string& json_str) -> decltype(T::jsonload(std::declval<nlohmann::json>())) {
  nlohmann::json j = nlohmann::json::parse(json_str);
  return T::jsonload(j);
}

/// Write an object to a json file
template <typename T>
inline void file_json_dump(const T& obj, const std::string& filepath, bool pretty = true) {
  file_json_dump(obj.jsondump(), filepath, pretty);
}

/// Read an object from a json file
template <typename T>
inline auto file_json_load(const std::string& filepath) -> decltype(T::jsonload(std::declval<nlohmann::json>())) {
  nlohmann::ordered_json j = file_json_load_data(filepath);
  return T::jsonload(j);
}

/// Encode a collection of objects to a json array
template <typename T>
inline nlohmann::ordered_json file_encode_collection(const std::vector<T>& collection) {
  nlohmann::ordered_json arr = nlohmann::json::array();
  for (const T& item : collection)
    arr.push_back(item.jsondump());
  return arr;
}

/// Encode a collection of shared pointers to a json array
template <typename T>
inline nlohmann::ordered_json file_encode_collection(const std::vector<std::shared_ptr<T>>& collection) {
  nlohmann::ordered_json arr = nlohmann::json::array();
  for (const std::shared_ptr<T>& item : collection)
    if (item)
      arr.push_back(item->jsondump());
  return arr;
}

/// Decode a json array to a collection of objects
template <typename T>
inline std::vector<T> file_decode_collection(const nlohmann::json& j) {
  std::vector<T> result;
  if (!j.is_array())
    return result;
  for (const nlohmann::json& item : j)
    result.push_back(T::jsonload(item));
  return result;
}

/// Decode a json array to a collection of shared pointers
template <typename T>
inline std::vector<std::shared_ptr<T>> file_decode_collection_ptr(const nlohmann::json& j) {
  std::vector<std::shared_ptr<T>> result;
  if (!j.is_array())
    return result;
  for (const nlohmann::json& item : j)
    result.push_back(std::make_shared<T>(T::jsonload(item)));
  return result;
}

} // namespace file_encoders
} // namespace session_cpp
