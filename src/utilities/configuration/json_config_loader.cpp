#include "utilities/ConfigManager.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <regex>

class JSONConfigLoader {
private:
    struct JSONValue {
        enum Type { STRING, NUMBER, BOOLEAN, OBJECT, ARRAY, NULL_VALUE };
        Type type;
        std::string stringValue;
        double numberValue;
        bool boolValue;
        std::unordered_map<std::string, JSONValue> objectValue;
        std::vector<JSONValue> arrayValue;
        
        JSONValue() : type(NULL_VALUE), numberValue(0.0), boolValue(false) {}
    };
    
    std::string trim(const std::string& str) {
        size_t start = str.find_first_not_of(" \t\n\r");
        if (start == std::string::npos) return "";
        size_t end = str.find_last_not_of(" \t\n\r");
        return str.substr(start, end - start + 1);
    }
    
    std::string removeComments(const std::string& json) {
        std::string result;
        bool inString = false;
        bool escaped = false;
        
        for (size_t i = 0; i < json.length(); ++i) {
            char c = json[i];
            
            if (escaped) {
                result += c;
                escaped = false;
                continue;
            }
            
            if (c == '\\' && inString) {
                escaped = true;
                result += c;
                continue;
            }
            
            if (c == '"') {
                inString = !inString;
                result += c;
                continue;
            }
            
            if (!inString && c == '/' && i + 1 < json.length()) {
                if (json[i + 1] == '/') {
                    // Skip line comment
                    while (i < json.length() && json[i] != '\n') ++i;
                    if (i < json.length()) result += json[i]; // Add the newline
                    continue;
                } else if (json[i + 1] == '*') {
                    // Skip block comment
                    i += 2;
                    while (i + 1 < json.length() && !(json[i] == '*' && json[i + 1] == '/')) ++i;
                    if (i + 1 < json.length()) i += 2;
                    continue;
                }
            }
            
            result += c;
        }
        
        return result;
    }
    
    JSONValue parseValue(const std::string& json, size_t& pos) {
        skipWhitespace(json, pos);
        
        if (pos >= json.length()) {
            throw std::runtime_error("Unexpected end of JSON");
        }
        
        char c = json[pos];
        
        if (c == '"') {
            return parseString(json, pos);
        } else if (c == '{') {
            return parseObject(json, pos);
        } else if (c == '[') {
            return parseArray(json, pos);
        } else if (c == 't' || c == 'f') {
            return parseBoolean(json, pos);
        } else if (c == 'n') {
            return parseNull(json, pos);
        } else if (isdigit(c) || c == '-' || c == '+') {
            return parseNumber(json, pos);
        } else {
            throw std::runtime_error("Unexpected character: " + std::string(1, c));
        }
    }
    
    void skipWhitespace(const std::string& json, size_t& pos) {
        while (pos < json.length() && isspace(json[pos])) {
            ++pos;
        }
    }
    
    JSONValue parseString(const std::string& json, size_t& pos) {
        JSONValue value;
        value.type = JSONValue::STRING;
        
        ++pos; // Skip opening quote
        std::string result;
        
        while (pos < json.length() && json[pos] != '"') {
            if (json[pos] == '\\' && pos + 1 < json.length()) {
                ++pos;
                switch (json[pos]) {
                    case '"': result += '"'; break;
                    case '\\': result += '\\'; break;
                    case '/': result += '/'; break;
                    case 'b': result += '\b'; break;
                    case 'f': result += '\f'; break;
                    case 'n': result += '\n'; break;
                    case 'r': result += '\r'; break;
                    case 't': result += '\t'; break;
                    default: result += json[pos]; break;
                }
            } else {
                result += json[pos];
            }
            ++pos;
        }
        
        if (pos >= json.length()) {
            throw std::runtime_error("Unterminated string");
        }
        
        ++pos; // Skip closing quote
        value.stringValue = result;
        return value;
    }
    
    JSONValue parseNumber(const std::string& json, size_t& pos) {
        JSONValue value;
        value.type = JSONValue::NUMBER;
        
        std::string numStr;
        while (pos < json.length() && (isdigit(json[pos]) || json[pos] == '.' || 
               json[pos] == '-' || json[pos] == '+' || json[pos] == 'e' || json[pos] == 'E')) {
            numStr += json[pos];
            ++pos;
        }
        
        try {
            value.numberValue = std::stod(numStr);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid number: " + numStr);
        }
        
        return value;
    }
    
    JSONValue parseBoolean(const std::string& json, size_t& pos) {
        JSONValue value;
        value.type = JSONValue::BOOLEAN;
        
        if (json.substr(pos, 4) == "true") {
            value.boolValue = true;
            pos += 4;
        } else if (json.substr(pos, 5) == "false") {
            value.boolValue = false;
            pos += 5;
        } else {
            throw std::runtime_error("Invalid boolean value");
        }
        
        return value;
    }
    
    JSONValue parseNull(const std::string& json, size_t& pos) {
        if (json.substr(pos, 4) != "null") {
            throw std::runtime_error("Invalid null value");
        }
        
        pos += 4;
        JSONValue value;
        value.type = JSONValue::NULL_VALUE;
        return value;
    }
    
    JSONValue parseObject(const std::string& json, size_t& pos) {
        JSONValue value;
        value.type = JSONValue::OBJECT;
        
        ++pos; // Skip opening brace
        skipWhitespace(json, pos);
        
        if (pos < json.length() && json[pos] == '}') {
            ++pos;
            return value; // Empty object
        }
        
        while (pos < json.length()) {
            skipWhitespace(json, pos);
            
            // Parse key
            if (json[pos] != '"') {
                throw std::runtime_error("Expected string key in object");
            }
            
            JSONValue keyValue = parseString(json, pos);
            std::string key = keyValue.stringValue;
            
            skipWhitespace(json, pos);
            
            if (pos >= json.length() || json[pos] != ':') {
                throw std::runtime_error("Expected ':' after object key");
            }
            ++pos;
            
            // Parse value
            JSONValue objValue = parseValue(json, pos);
            value.objectValue[key] = objValue;
            
            skipWhitespace(json, pos);
            
            if (pos >= json.length()) {
                throw std::runtime_error("Unexpected end of object");
            }
            
            if (json[pos] == '}') {
                ++pos;
                break;
            } else if (json[pos] == ',') {
                ++pos;
            } else {
                throw std::runtime_error("Expected ',' or '}' in object");
            }
        }
        
        return value;
    }
    
    JSONValue parseArray(const std::string& json, size_t& pos) {
        JSONValue value;
        value.type = JSONValue::ARRAY;
        
        ++pos; // Skip opening bracket
        skipWhitespace(json, pos);
        
        if (pos < json.length() && json[pos] == ']') {
            ++pos;
            return value; // Empty array
        }
        
        while (pos < json.length()) {
            JSONValue arrayValue = parseValue(json, pos);
            value.arrayValue.push_back(arrayValue);
            
            skipWhitespace(json, pos);
            
            if (pos >= json.length()) {
                throw std::runtime_error("Unexpected end of array");
            }
            
            if (json[pos] == ']') {
                ++pos;
                break;
            } else if (json[pos] == ',') {
                ++pos;
                skipWhitespace(json, pos);
            } else {
                throw std::runtime_error("Expected ',' or ']' in array");
            }
        }
        
        return value;
    }
    
    void extractConfigValues(const JSONValue& value, const std::string& prefix, 
                           std::unordered_map<std::string, std::any>& config) {
        
        switch (value.type) {
            case JSONValue::STRING:
                config[prefix] = value.stringValue;
                break;
            case JSONValue::NUMBER:
                config[prefix] = value.numberValue;
                break;
            case JSONValue::BOOLEAN:
                config[prefix] = value.boolValue;
                break;
            case JSONValue::OBJECT:
                for (const auto& [key, objValue] : value.objectValue) {
                    std::string newPrefix = prefix.empty() ? key : prefix + "." + key;
                    extractConfigValues(objValue, newPrefix, config);
                }
                break;
            case JSONValue::ARRAY:
                for (size_t i = 0; i < value.arrayValue.size(); ++i) {
                    std::string newPrefix = prefix + "[" + std::to_string(i) + "]";
                    extractConfigValues(value.arrayValue[i], newPrefix, config);
                }
                break;
            case JSONValue::NULL_VALUE:
                // Skip null values
                break;
        }
    }
    
public:
    bool loadFromFile(const std::string& filename, std::unordered_map<std::string, std::any>& config) {
        std::cout << "[JSON_LOADER] Loading configuration from " << filename << std::endl;
        
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "[JSON_LOADER] Failed to open file: " << filename << std::endl;
            return false;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        file.close();
        
        return loadFromString(content, config);
    }
    
    bool loadFromString(const std::string& jsonContent, std::unordered_map<std::string, std::any>& config) {
        try {
            std::string cleanJson = removeComments(trim(jsonContent));
            if (cleanJson.empty()) {
                std::cout << "[JSON_LOADER] Empty JSON content" << std::endl;
                return false;
            }
            
            size_t pos = 0;
            JSONValue root = parseValue(cleanJson, pos);
            
            if (root.type != JSONValue::OBJECT) {
                std::cout << "[JSON_LOADER] Root element must be an object" << std::endl;
                return false;
            }
            
            extractConfigValues(root, "", config);
            
            std::cout << "[JSON_LOADER] Successfully loaded " << config.size() 
                      << " configuration values" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[JSON_LOADER] Parse error: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool saveToFile(const std::string& filename, const std::unordered_map<std::string, std::any>& config) {
        std::cout << "[JSON_LOADER] Saving configuration to " << filename << std::endl;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "[JSON_LOADER] Failed to create file: " << filename << std::endl;
            return false;
        }
        
        std::string jsonContent = generateJSON(config);
        file << jsonContent;
        file.close();
        
        std::cout << "[JSON_LOADER] Configuration saved successfully" << std::endl;
        return true;
    }
    
private:
    std::string generateJSON(const std::unordered_map<std::string, std::any>& config) {
        std::ostringstream json;
        json << "{\n";
        
        bool first = true;
        for (const auto& [key, value] : config) {
            if (!first) json << ",\n";
            first = false;
            
            json << "  \"" << escapeString(key) << "\": ";
            json << valueToJSON(value);
        }
        
        json << "\n}\n";
        return json.str();
    }
    
    std::string valueToJSON(const std::any& value) {
        try {
            if (auto strVal = std::any_cast<std::string>(&value)) {
                return "\"" + escapeString(*strVal) + "\"";
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
    
    std::string escapeString(const std::string& str) {
        std::string escaped;
        for (char c : str) {
            switch (c) {
                case '"': escaped += "\\\""; break;
                case '\\': escaped += "\\\\"; break;
                case '\b': escaped += "\\b"; break;
                case '\f': escaped += "\\f"; break;
                case '\n': escaped += "\\n"; break;
                case '\r': escaped += "\\r"; break;
                case '\t': escaped += "\\t"; break;
                default: escaped += c; break;
            }
        }
        return escaped;
    }
};

// Global JSON loader instance
static std::unique_ptr<JSONConfigLoader> g_jsonLoader;

bool loadJSONConfig(const std::string& filename, std::unordered_map<std::string, std::any>& config) {
    if (!g_jsonLoader) {
        g_jsonLoader = std::make_unique<JSONConfigLoader>();
    }
    return g_jsonLoader->loadFromFile(filename, config);
}

bool saveJSONConfig(const std::string& filename, const std::unordered_map<std::string, std::any>& config) {
    if (!g_jsonLoader) {
        g_jsonLoader = std::make_unique<JSONConfigLoader>();
    }
    return g_jsonLoader->saveToFile(filename, config);
}

bool parseJSONString(const std::string& jsonContent, std::unordered_map<std::string, std::any>& config) {
    if (!g_jsonLoader) {
        g_jsonLoader = std::make_unique<JSONConfigLoader>();
    }
    return g_jsonLoader->loadFromString(jsonContent, config);
}