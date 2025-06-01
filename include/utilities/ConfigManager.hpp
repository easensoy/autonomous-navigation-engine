#pragma once
#include <string>
#include <unordered_map>
#include <vector>
#include <any>

class ConfigManager {
private:
    std::unordered_map<std::string, std::any> configValues;
    std::string configFilePath;
    bool autoSave;
    
    void parseConfigFile(const std::string& filename);
    void saveConfigFile() const;
    std::string serializeValue(const std::any& value) const;
    std::any deserializeValue(const std::string& valueStr, const std::string& type) const;

public:
    ConfigManager();
    explicit ConfigManager(const std::string& configFile);
    ~ConfigManager();
    
    bool loadFromFile(const std::string& filename);
    bool saveToFile(const std::string& filename) const;
    void setAutoSave(bool enable);
    
    template<typename T>
    void setValue(const std::string& key, const T& value);
    
    template<typename T>
    T getValue(const std::string& key, const T& defaultValue = T{}) const;
    
    bool hasKey(const std::string& key) const;
    void removeKey(const std::string& key);
    void clearAll();
    
    std::vector<std::string> getAllKeys() const;
    std::unordered_map<std::string, std::string> getAllValues() const;
    
    void setSection(const std::string& section, const std::unordered_map<std::string, std::any>& values);
    std::unordered_map<std::string, std::any> getSection(const std::string& section) const;
    
    bool validateConfiguration() const;
    void setDefaultValues();
    void mergeConfiguration(const ConfigManager& other);
    
    void exportToJSON(const std::string& filename) const;
    void importFromJSON(const std::string& filename);
    void printConfiguration() const;
};

template<typename T>
void ConfigManager::setValue(const std::string& key, const T& value) {
    configValues[key] = value;
    if (autoSave) {
        saveConfigFile();
    }
}

template<typename T>
T ConfigManager::getValue(const std::string& key, const T& defaultValue) const {
    auto it = configValues.find(key);
    if (it != configValues.end()) {
        try {
            return std::any_cast<T>(it->second);
        } catch (const std::bad_any_cast& e) {
            return defaultValue;
        }
    }
    return defaultValue;
}