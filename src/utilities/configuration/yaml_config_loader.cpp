#include "utilities/ConfigManager.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <regex>

class YAMLConfigLoader {
private:
    struct YAMLValue {
        enum Type { STRING, NUMBER, BOOLEAN, OBJECT, ARRAY, NULL_VALUE };
        Type type;
        std::string stringValue;
        double numberValue;
        bool boolValue;
        std::unordered_map<std::string, YAMLValue> objectValue;
        std::vector<YAMLValue> arrayValue;
        
        YAMLValue() : type(NULL_VALUE), numberValue(0.0), boolValue(false) {}
    };
    
    std::string trim(const std::string& str) {
        size_t start = str.find_first_not_of(" \t\r");
        if (start == std::string::npos) return "";
        size_t end = str.find_last_not_of(" \t\r");
        return str.substr(start, end - start + 1);
    }
    
    std::vector<std::string> splitLines(const std::string& content) {
        std::vector<std::string> lines;
        std::istringstream stream(content);
        std::string line;
        
        while (std::getline(stream, line)) {
            lines.push_back(line);
        }
        
        return lines;
    }
    
    std::string removeComments(const std::string& line) {
        bool inQuotes = false;
        char quoteChar = '\0';
        
        for (size_t i = 0; i < line.length(); ++i) {
            char c = line[i];
            
            if (!inQuotes && (c == '"' || c == '\'')) {
                inQuotes = true;
                quoteChar = c;
            } else if (inQuotes && c == quoteChar) {
                if (i == 0 || line[i-1] != '\\') {
                    inQuotes = false;
                }
            } else if (!inQuotes && c == '#') {
                return line.substr(0, i);
            }
        }
        
        return line;
    }
    
    int getIndentLevel(const std::string& line) {
        int indent = 0;
        for (char c : line) {
            if (c == ' ') indent++;
            else if (c == '\t') indent += 4; // Tab = 4 spaces
            else break;
        }
        return indent;
    }
    
    YAMLValue parseValue(const std::string& value) {
        std::string trimmedValue = trim(value);
        
        if (trimmedValue.empty() || trimmedValue == "null" || trimmedValue == "~") {
            YAMLValue yamlValue;
            yamlValue.type = YAMLValue::NULL_VALUE;
            return yamlValue;
        }
        
        // Boolean values
        if (trimmedValue == "true" || trimmedValue == "True" || trimmedValue == "TRUE" ||
            trimmedValue == "yes" || trimmedValue == "Yes" || trimmedValue == "YES") {
            YAMLValue yamlValue;
            yamlValue.type = YAMLValue::BOOLEAN;
            yamlValue.boolValue = true;
            return yamlValue;
        }
        
        if (trimmedValue == "false" || trimmedValue == "False" || trimmedValue == "FALSE" ||
            trimmedValue == "no" || trimmedValue == "No" || trimmedValue == "NO") {
            YAMLValue yamlValue;
            yamlValue.type = YAMLValue::BOOLEAN;
            yamlValue.boolValue = false;
            return yamlValue;
        }
        
        // Number values
        if (isNumber(trimmedValue)) {
            YAMLValue yamlValue;
            yamlValue.type = YAMLValue::NUMBER;
            yamlValue.numberValue = std::stod(trimmedValue);
            return yamlValue;
        }
        
        // String values
        YAMLValue yamlValue;
        yamlValue.type = YAMLValue::STRING;
        yamlValue.stringValue = unquoteString(trimmedValue);
        return yamlValue;
    }
    
    bool isNumber(const std::string& str) {
        if (str.empty()) return false;
        
        size_t start = 0;
        if (str[0] == '+' || str[0] == '-') start = 1;
        
        bool hasDecimal = false;
        bool hasExponent = false;
        
        for (size_t i = start; i < str.length(); ++i) {
            char c = str[i];
            
            if (isdigit(c)) {
                continue;
            } else if (c == '.' && !hasDecimal && !hasExponent) {
                hasDecimal = true;
            } else if ((c == 'e' || c == 'E') && !hasExponent) {
                hasExponent = true;
                if (i + 1 < str.length() && (str[i + 1] == '+' || str[i + 1] == '-')) {
                    ++i;
                }
            } else {
                return false;
            }
        }
        
        return true;
    }
    
    std::string unquoteString(const std::string& str) {
        if (str.length() >= 2) {
            if ((str.front() == '"' && str.back() == '"') ||
                (str.front() == '\'' && str.back() == '\'')) {
                std::string unquoted = str.substr(1, str.length() - 2);
                return processEscapeSequences(unquoted);
            }
        }
        return str;
    }
    
    std::string processEscapeSequences(const std::string& str) {
        std::string result;
        for (size_t i = 0; i < str.length(); ++i) {
            if (str[i] == '\\' && i + 1 < str.length()) {
                switch (str[i + 1]) {
                    case 'n': result += '\n'; ++i; break;
                    case 't': result += '\t'; ++i; break;
                    case 'r': result += '\r'; ++i; break;
                    case '\\': result += '\\'; ++i; break;
                    case '"': result += '"'; ++i; break;
                    case '\'': result += '\''; ++i; break;
                    default: result += str[i]; break;
                }
            } else {
                result += str[i];
            }
        }
        return result;
    }
    
    YAMLValue parseDocument(const std::vector<std::string>& lines) {
        YAMLValue root;
        root.type = YAMLValue::OBJECT;
        
        std::vector<std::pair<int, std::string*>> contextStack;
        contextStack.push_back({-1, nullptr});
        
        for (size_t lineNum = 0; lineNum < lines.size(); ++lineNum) {
            std::string line = removeComments(lines[lineNum]);
            if (trim(line).empty()) continue;
            
            int indent = getIndentLevel(line);
            std::string content = trim(line);
            
            // Pop context stack to current indent level
            while (contextStack.size() > 1 && contextStack.back().first >= indent) {
                contextStack.pop_back();
            }
            
            if (content.find(':') != std::string::npos) {
                // Key-value pair
                size_t colonPos = content.find(':');
                std::string key = trim(content.substr(0, colonPos));
                std::string value = trim(content.substr(colonPos + 1));
                
                // Remove quotes from key if present
                key = unquoteString(key);
                
                YAMLValue* currentObject = getCurrentObject(root, contextStack);
                if (!currentObject) continue;
                
                if (value.empty()) {
                    // Empty value - might be an object
                    currentObject->objectValue[key] = YAMLValue();
                    currentObject->objectValue[key].type = YAMLValue::OBJECT;
                    contextStack.push_back({indent, &key});
                } else if (value == "[" || value.front() == '[') {
                    // Inline array
                    currentObject->objectValue[key] = parseInlineArray(value);
                } else if (value == "{" || value.front() == '{') {
                    // Inline object
                    currentObject->objectValue[key] = parseInlineObject(value);
                } else {
                    // Regular value
                    currentObject->objectValue[key] = parseValue(value);
                }
            } else if (content.front() == '-') {
                // Array item
                std::string itemValue = trim(content.substr(1));
                
                YAMLValue* currentArray = getCurrentArray(root, contextStack);
                if (currentArray) {
                    if (itemValue.empty()) {
                        YAMLValue arrayItem;
                        arrayItem.type = YAMLValue::OBJECT;
                        currentArray->arrayValue.push_back(arrayItem);
                    } else {
                        currentArray->arrayValue.push_back(parseValue(itemValue));
                    }
                }
            }
        }
        
        return root;
    }
    
    YAMLValue* getCurrentObject(YAMLValue& root, const std::vector<std::pair<int, std::string*>>& contextStack) {
        YAMLValue* current = &root;
        
        for (size_t i = 1; i < contextStack.size(); ++i) {
            if (contextStack[i].second) {
                const std::string& key = *contextStack[i].second;
                if (current->objectValue.find(key) != current->objectValue.end()) {
                    current = &current->objectValue[key];
                }
            }
        }
        
        return (current->type == YAMLValue::OBJECT) ? current : nullptr;
    }
    
    YAMLValue* getCurrentArray(YAMLValue& root, const std::vector<std::pair<int, std::string*>>& contextStack) {
        YAMLValue* current = &root;
        
        for (size_t i = 1; i < contextStack.size(); ++i) {
            if (contextStack[i].second) {
                const std::string& key = *contextStack[i].second;
                if (current->objectValue.find(key) != current->objectValue.end()) {
                    current = &current->objectValue[key];
                }
            }
        }
        
        return (current->type == YAMLValue::ARRAY) ? current : nullptr;
    }
    
    YAMLValue parseInlineArray(const std::string& arrayStr) {
        YAMLValue array;
        array.type = YAMLValue::ARRAY;
        
        std::string content = arrayStr;
        if (content.front() == '[') {
            size_t end = content.find(']');
            if (end != std::string::npos) {
                content = content.substr(1, end - 1);
            }
        }
        
        std::istringstream stream(content);
        std::string item;
        
        while (std::getline(stream, item, ',')) {
            array.arrayValue.push_back(parseValue(trim(item)));
        }
        
        return array;
    }
    
    YAMLValue parseInlineObject(const std::string& objectStr) {
        YAMLValue object;
        object.type = YAMLValue::OBJECT;
        
        std::string content = objectStr;
        if (content.front() == '{') {
            size_t end = content.find('}');
            if (end != std::string::npos) {
                content = content.substr(1, end - 1);
            }
        }
        
        std::istringstream stream(content);
        std::string pair;
        
        while (std::getline(stream, pair, ',')) {
            size_t colonPos = pair.find(':');
            if (colonPos != std::string::npos) {
                std::string key = trim(pair.substr(0, colonPos));
                std::string value = trim(pair.substr(colonPos + 1));
                object.objectValue[unquoteString(key)] = parseValue(value);
            }
        }
        
        return object;
    }
    
    void extractConfigValues(const YAMLValue& value, const std::string& prefix, 
                           std::unordered_map<std::string, std::any>& config) {
        
        switch (value.type) {
            case YAMLValue::STRING:
                config[prefix] = value.stringValue;
                break;
            case YAMLValue::NUMBER:
                config[prefix] = value.numberValue;
                break;
            case YAMLValue::BOOLEAN:
                config[prefix] = value.boolValue;
                break;
            case YAMLValue::OBJECT:
                for (const auto& [key, objValue] : value.objectValue) {
                    std::string newPrefix = prefix.empty() ? key : prefix + "." + key;
                    extractConfigValues(objValue, newPrefix, config);
                }
                break;
            case YAMLValue::ARRAY:
                for (size_t i = 0; i < value.arrayValue.size(); ++i) {
                    std::string newPrefix = prefix + "[" + std::to_string(i) + "]";
                    extractConfigValues(value.arrayValue[i], newPrefix, config);
                }
                break;
            case YAMLValue::NULL_VALUE:
                // Skip null values
                break;
        }
    }
    
public:
    bool loadFromFile(const std::string& filename, std::unordered_map<std::string, std::any>& config) {
        std::cout << "[YAML_LOADER] Loading configuration from " << filename << std::endl;
        
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "[YAML_LOADER] Failed to open file: " << filename << std::endl;
            return false;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        file.close();
        
        return loadFromString(content, config);
    }
    
    bool loadFromString(const std::string& yamlContent, std::unordered_map<std::string, std::any>& config) {
        try {
            if (yamlContent.empty()) {
                std::cout << "[YAML_LOADER] Empty YAML content" << std::endl;
                return false;
            }
            
            std::vector<std::string> lines = splitLines(yamlContent);
            YAMLValue root = parseDocument(lines);
            
            extractConfigValues(root, "", config);
            
            std::cout << "[YAML_LOADER] Successfully loaded " << config.size() 
                      << " configuration values" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[YAML_LOADER] Parse error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool saveToFile(const std::string& filename, const std::unordered_map<std::string, std::any>& config) {
        std::cout << "[YAML_LOADER] Saving configuration to " << filename << std::endl;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "[YAML_LOADER] Failed to create file: " << filename << std::endl;
            return false;
        }
        
        std::string yamlContent = generateYAML(config);
        file << yamlContent;
        file.close();
        
        std::cout << "[YAML_LOADER] Configuration saved successfully" << std::endl;
        return true;
    }
    
private:
    std::string generateYAML(const std::unordered_map<std::string, std::any>& config) {
        std::ostringstream yaml;
        yaml << "# Auto-generated YAML configuration\n\n";
        
        // Group config values by top-level keys
        std::unordered_map<std::string, std::vector<std::pair<std::string, std::any>>> groups;
        
        for (const auto& [key, value] : config) {
            size_t dotPos = key.find('.');
            std::string topLevel = (dotPos != std::string::npos) ? key.substr(0, dotPos) : key;
            groups[topLevel].emplace_back(key, value);
        }
        
        for (const auto& [groupName, items] : groups) {
            if (items.size() == 1 && items[0].first == groupName) {
                // Simple key-value pair
                yaml << groupName << ": " << valueToYAML(items[0].second) << "\n";
            } else {
                // Group with nested values
                yaml << groupName << ":\n";
                for (const auto& [fullKey, value] : items) {
                    std::string subKey = (fullKey.length() > groupName.length() + 1) ? 
                                        fullKey.substr(groupName.length() + 1) : "";
                    if (!subKey.empty()) {
                        yaml << "  " << subKey << ": " << valueToYAML(value) << "\n";
                    }
                }
            }
            yaml << "\n";
        }
        
        return yaml.str();
    }
    
    std::string valueToYAML(const std::any& value) {
        try {
            if (auto strVal = std::any_cast<std::string>(&value)) {
                return needsQuotes(*strVal) ? ("\"" + escapeString(*strVal) + "\"") : *strVal;
            } else if (auto intVal = std::any_cast<int>(&value)) {
                return std::to_string(*intVal);
            } else if (auto doubleVal = std::any_cast<double>(&value)) {
                return std::to_string(*doubleVal);
            } else if (auto boolVal = std::any_cast<bool>(&value)) {
                return *boolVal ? "true" : "false";
            } else {
                return "null";
            }
        } catch (const std::bad_any_cast&) {
            return "null";
        }
    }
    
    bool needsQuotes(const std::string& str) {
        if (str.empty()) return true;
        
        // Check for special YAML values
        if (str == "true" || str == "false" || str == "null" || str == "yes" || str == "no") {
            return true;
        }
        
        // Check if it looks like a number
        if (isNumber(str)) return true;
        
        // Check for special characters
        for (char c : str) {
            if (c == ':' || c == '#' || c == '[' || c == ']' || c == '{' || c == '}' || 
                c == ',' || c == '&' || c == '*' || c == '!' || c == '|' || c == '>' ||
                c == '\'' || c == '"' || c == '%' || c == '@' || c == '`') {
                return true;
            }
        }
        
        return false;
    }
    
    std::string escapeString(const std::string& str) {
        std::string escaped;
        for (char c : str) {
            switch (c) {
                case '"': escaped += "\\\""; break;
                case '\\': escaped += "\\\\"; break;
                case '\n': escaped += "\\n"; break;
                case '\t': escaped += "\\t"; break;
                case '\r': escaped += "\\r"; break;
                default: escaped += c; break;
            }
        }
        return escaped;
    }
};

// Global YAML loader instance
static std::unique_ptr<YAMLConfigLoader> g_yamlLoader;

bool loadYAMLConfig(const std::string& filename, std::unordered_map<std::string, std::any>& config) {
    if (!g_yamlLoader) {
        g_yamlLoader = std::make_unique<YAMLConfigLoader>();
    }
    return g_yamlLoader->loadFromFile(filename, config);
}

bool saveYAMLConfig(const std::string& filename, const std::unordered_map<std::string, std::any>& config) {
    if (!g_yamlLoader) {
        g_yamlLoader = std::make_unique<YAMLConfigLoader>();
    }
    return g_yamlLoader->saveToFile(filename, config);
}

bool parseYAMLString(const std::string& yamlContent, std::unordered_map<std::string, std::any>& config) {
    if (!g_yamlLoader) {
        g_yamlLoader = std::make_unique<YAMLConfigLoader>();
    }
    return g_yamlLoader->loadFromString(yamlContent, config);
}