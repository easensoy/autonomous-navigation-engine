#include "utilities/ConfigManager.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <typeinfo>
#include <vector>
#include <any>
#include <unordered_map>
#include <utility>

// ConfigManager stores typed values (std::any) under dotted keys and persists them through the
// JSON and YAML loaders in this folder (json_config_loader.cpp, yaml_config_loader.cpp).

bool loadJSONConfig(const std::string& filename, std::unordered_map<std::string, std::any>& config);
bool saveJSONConfig(const std::string& filename, const std::unordered_map<std::string, std::any>& config);
bool loadYAMLConfig(const std::string& filename, std::unordered_map<std::string, std::any>& config);
bool saveYAMLConfig(const std::string& filename, const std::unordered_map<std::string, std::any>& config);

namespace {
bool isYamlFile(const std::string& filename) {
    auto endsWith = [&filename](const std::string& suffix) {
        return filename.size() >= suffix.size() &&
               filename.compare(filename.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    return endsWith(".yaml") || endsWith(".yml");
}
}  // namespace

ConfigManager::ConfigManager() : autoSave(false) {}

ConfigManager::ConfigManager(const std::string& configFile) : configFilePath(configFile), autoSave(false) {
    std::ifstream probe(configFile);
    if (probe.good()) {
        parseConfigFile(configFile);
    }
}

ConfigManager::~ConfigManager() {
    if (autoSave && !configFilePath.empty()) {
        saveConfigFile();
    }
}

void ConfigManager::parseConfigFile(const std::string& filename) {
    if (!loadFromFile(filename)) {
        std::cerr << "[ConfigManager] Could not parse configuration file " << filename << std::endl;
    }
}

void ConfigManager::saveConfigFile() const {
    if (configFilePath.empty()) {
        return;  // nothing to save to until a file has been chosen
    }
    if (!saveToFile(configFilePath)) {
        std::cerr << "[ConfigManager] Could not save configuration to " << configFilePath << std::endl;
    }
}

bool ConfigManager::loadFromFile(const std::string& filename) {
    std::unordered_map<std::string, std::any> loaded;
    const bool ok = isYamlFile(filename) ? loadYAMLConfig(filename, loaded) : loadJSONConfig(filename, loaded);
    if (!ok) {
        return false;
    }
    for (auto& entry : loaded) {
        configValues[entry.first] = std::move(entry.second);
    }
    configFilePath = filename;
    return true;
}

bool ConfigManager::saveToFile(const std::string& filename) const {
    return isYamlFile(filename) ? saveYAMLConfig(filename, configValues) : saveJSONConfig(filename, configValues);
}

void ConfigManager::setAutoSave(bool enable) {
    autoSave = enable;
}

bool ConfigManager::hasKey(const std::string& key) const {
    return configValues.find(key) != configValues.end();
}

void ConfigManager::removeKey(const std::string& key) {
    configValues.erase(key);
    if (autoSave) {
        saveConfigFile();
    }
}

void ConfigManager::clearAll() {
    configValues.clear();
    if (autoSave) {
        saveConfigFile();
    }
}

std::vector<std::string> ConfigManager::getAllKeys() const {
    std::vector<std::string> keys;
    for (const auto& entry : configValues) {
        keys.push_back(entry.first);
    }
    std::sort(keys.begin(), keys.end());
    return keys;
}

std::unordered_map<std::string, std::string> ConfigManager::getAllValues() const {
    std::unordered_map<std::string, std::string> values;
    for (const auto& entry : configValues) {
        values[entry.first] = serializeValue(entry.second);
    }
    return values;
}

void ConfigManager::setSection(const std::string& section, const std::unordered_map<std::string, std::any>& values) {
    for (const auto& entry : values) {
        configValues[section + "." + entry.first] = entry.second;
    }
    if (autoSave) {
        saveConfigFile();
    }
}

std::unordered_map<std::string, std::any> ConfigManager::getSection(const std::string& section) const {
    std::unordered_map<std::string, std::any> values;
    const std::string prefix = section + ".";
    for (const auto& entry : configValues) {
        if (entry.first.compare(0, prefix.size(), prefix) == 0) {
            values[entry.first.substr(prefix.size())] = entry.second;
        }
    }
    return values;
}

bool ConfigManager::validateConfiguration() const {
    for (const auto& entry : configValues) {
        if (entry.first.empty() || !entry.second.has_value()) {
            return false;
        }
    }
    return true;
}

void ConfigManager::mergeConfiguration(const ConfigManager& other) {
    for (const auto& entry : other.configValues) {
        configValues[entry.first] = entry.second;
    }
}

void ConfigManager::exportToJSON(const std::string& filename) const {
    if (!saveJSONConfig(filename, configValues)) {
        std::cerr << "[ConfigManager] Could not export configuration to " << filename << std::endl;
    }
}

void ConfigManager::importFromJSON(const std::string& filename) {
    std::unordered_map<std::string, std::any> loaded;
    if (!loadJSONConfig(filename, loaded)) {
        std::cerr << "[ConfigManager] Could not import configuration from " << filename << std::endl;
        return;
    }
    for (auto& entry : loaded) {
        configValues[entry.first] = std::move(entry.second);
    }
}

void ConfigManager::printConfiguration() const {
    std::cout << "=== Configuration (" << configValues.size() << " values) ===" << std::endl;
    const auto values = getAllValues();
    for (const std::string& key : getAllKeys()) {
        std::cout << "  " << key << " = " << values.at(key) << std::endl;
    }
}

std::string ConfigManager::serializeValue(const std::any& value) const {
    if (!value.has_value()) return "";
    if (value.type() == typeid(std::string)) return std::any_cast<std::string>(value);
    if (value.type() == typeid(const char*)) return std::any_cast<const char*>(value);
    if (value.type() == typeid(bool)) return std::any_cast<bool>(value) ? "true" : "false";
    if (value.type() == typeid(int)) return std::to_string(std::any_cast<int>(value));
    if (value.type() == typeid(long)) return std::to_string(std::any_cast<long>(value));
    if (value.type() == typeid(long long)) return std::to_string(std::any_cast<long long>(value));
    if (value.type() == typeid(size_t)) return std::to_string(std::any_cast<size_t>(value));
    if (value.type() == typeid(float)) return std::to_string(std::any_cast<float>(value));
    if (value.type() == typeid(double)) {
        std::ostringstream out;
        out << std::any_cast<double>(value);
        return out.str();
    }
    return "<unsupported type>";
}

std::any ConfigManager::deserializeValue(const std::string& valueStr, const std::string& type) const {
    if (type == "int") return std::stoi(valueStr);
    if (type == "double" || type == "float") return std::stod(valueStr);
    if (type == "bool") return valueStr == "true" || valueStr == "1";
    return valueStr;
}
