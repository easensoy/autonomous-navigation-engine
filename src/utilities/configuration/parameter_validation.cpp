#include "utilities/ConfigManager.hpp"
#include <iostream>
#include <regex>
#include <limits>
#include <cmath>
#include <functional>

class ParameterValidator {
private:
    struct ValidationRule {
        std::string parameterName;
        std::string dataType;
        bool required;
        std::any minValue;
        std::any maxValue;
        std::vector<std::any> allowedValues;
        std::function<bool(const std::any&)> customValidator;
        std::string description;
        
        ValidationRule(const std::string& name) : parameterName(name), required(false) {}
    };
    
    std::unordered_map<std::string, ValidationRule> validationRules;
    std::vector<std::string> validationErrors;
    bool strictMode;
    
public:
    ParameterValidator() : strictMode(true) {
        std::cout << "[PARAM_VALIDATOR] Parameter validator initialized" << std::endl;
        setupDefaultRules();
    }
    
    void addRule(const std::string& paramName, const std::string& type, bool required = false) {
        ValidationRule rule(paramName);
        rule.dataType = type;
        rule.required = required;
        validationRules[paramName] = rule;
        
        std::cout << "[PARAM_VALIDATOR] Added rule for " << paramName 
                  << " (type: " << type << ", required: " << (required ? "yes" : "no") << ")" << std::endl;
    }
    
    void setRange(const std::string& paramName, double minVal, double maxVal) {
        auto it = validationRules.find(paramName);
        if (it != validationRules.end()) {
            it->second.minValue = minVal;
            it->second.maxValue = maxVal;
            std::cout << "[PARAM_VALIDATOR] Set range for " << paramName 
                      << ": [" << minVal << ", " << maxVal << "]" << std::endl;
        }
    }
    
    void setAllowedValues(const std::string& paramName, const std::vector<std::string>& values) {
        auto it = validationRules.find(paramName);
        if (it != validationRules.end()) {
            it->second.allowedValues.clear();
            for (const auto& val : values) {
                it->second.allowedValues.push_back(val);
            }
            std::cout << "[PARAM_VALIDATOR] Set allowed values for " << paramName 
                      << " (" << values.size() << " values)" << std::endl;
        }
    }
    
    void setCustomValidator(const std::string& paramName, std::function<bool(const std::any&)> validator) {
        auto it = validationRules.find(paramName);
        if (it != validationRules.end()) {
            it->second.customValidator = validator;
            std::cout << "[PARAM_VALIDATOR] Set custom validator for " << paramName << std::endl;
        }
    }
    
    bool validateConfiguration(const std::unordered_map<std::string, std::any>& config) {
        validationErrors.clear();
        
        std::cout << "[PARAM_VALIDATOR] Validating configuration with " << config.size() 
                  << " parameters" << std::endl;
        
        // Check required parameters
        for (const auto& [ruleName, rule] : validationRules) {
            if (rule.required && config.find(ruleName) == config.end()) {
                addError("Required parameter '" + ruleName + "' is missing");
            }
        }
        
        // Validate existing parameters
        for (const auto& [paramName, value] : config) {
            validateParameter(paramName, value);
        }
        
        // Check for unknown parameters in strict mode
        if (strictMode) {
            for (const auto& [paramName, value] : config) {
                if (validationRules.find(paramName) == validationRules.end()) {
                    addError("Unknown parameter '" + paramName + "' (strict mode)");
                }
            }
        }
        
        bool isValid = validationErrors.empty();
        std::cout << "[PARAM_VALIDATOR] Validation " << (isValid ? "passed" : "failed") 
                  << " (" << validationErrors.size() << " errors)" << std::endl;
        
        return isValid;
    }
    
    std::vector<std::string> getValidationErrors() const {
        return validationErrors;
    }
    
    void printValidationErrors() const {
        if (validationErrors.empty()) {
            std::cout << "[PARAM_VALIDATOR] No validation errors" << std::endl;
            return;
        }
        
        std::cout << "[PARAM_VALIDATOR] Validation errors:" << std::endl;
        for (const auto& error : validationErrors) {
            std::cout << "  - " << error << std::endl;
        }
    }
    
    void setStrictMode(bool strict) {
        strictMode = strict;
        std::cout << "[PARAM_VALIDATOR] Strict mode " << (strict ? "enabled" : "disabled") << std::endl;
    }
    
    bool fixCommonIssues(std::unordered_map<std::string, std::any>& config) {
        bool fixed = false;
        std::vector<std::string> fixes;
        
        for (auto& [paramName, value] : config) {
            auto it = validationRules.find(paramName);
            if (it == validationRules.end()) continue;
            
            const ValidationRule& rule = it->second;
            
            // Type conversion fixes
            if (rule.dataType == "double" || rule.dataType == "float") {
                if (auto strVal = std::any_cast<std::string>(&value)) {
                    try {
                        double numVal = std::stod(*strVal);
                        value = numVal;
                        fixes.push_back("Converted '" + paramName + "' from string to number");
                        fixed = true;
                    } catch (const std::exception&) {
                        // Cannot convert
                    }
                }
            } else if (rule.dataType == "int" || rule.dataType == "integer") {
                if (auto strVal = std::any_cast<std::string>(&value)) {
                    try {
                        int intVal = std::stoi(*strVal);
                        value = intVal;
                        fixes.push_back("Converted '" + paramName + "' from string to integer");
                        fixed = true;
                    } catch (const std::exception&) {
                        // Cannot convert
                    }
                } else if (auto doubleVal = std::any_cast<double>(&value)) {
                    int intVal = static_cast<int>(*doubleVal);
                    value = intVal;
                    fixes.push_back("Converted '" + paramName + "' from double to integer");
                    fixed = true;
                }
            } else if (rule.dataType == "bool" || rule.dataType == "boolean") {
                if (auto strVal = std::any_cast<std::string>(&value)) {
                    std::string lowerStr = *strVal;
                    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);
                    if (lowerStr == "true" || lowerStr == "1" || lowerStr == "yes") {
                        value = true;
                        fixes.push_back("Converted '" + paramName + "' to boolean true");
                        fixed = true;
                    } else if (lowerStr == "false" || lowerStr == "0" || lowerStr == "no") {
                        value = false;
                        fixes.push_back("Converted '" + paramName + "' to boolean false");
                        fixed = true;
                    }
                }
            }
            
            // Range fixes
            if (rule.dataType == "double" && rule.minValue.has_value() && rule.maxValue.has_value()) {
                if (auto doubleVal = std::any_cast<double>(&value)) {
                    double minVal = std::any_cast<double>(rule.minValue);
                    double maxVal = std::any_cast<double>(rule.maxValue);
                    
                    if (*doubleVal < minVal) {
                        *doubleVal = minVal;
                        fixes.push_back("Clamped '" + paramName + "' to minimum value");
                        fixed = true;
                    } else if (*doubleVal > maxVal) {
                        *doubleVal = maxVal;
                        fixes.push_back("Clamped '" + paramName + "' to maximum value");
                        fixed = true;
                    }
                }
            }
        }
        
        if (fixed) {
            std::cout << "[PARAM_VALIDATOR] Applied " << fixes.size() << " automatic fixes:" << std::endl;
            for (const auto& fix : fixes) {
                std::cout << "  - " << fix << std::endl;
            }
        }
        
        return fixed;
    }
    
    void generateValidationReport() const {
        std::cout << "\n[PARAM_VALIDATOR] === VALIDATION RULES REPORT ===" << std::endl;
        std::cout << "Total rules: " << validationRules.size() << std::endl;
        std::cout << "Strict mode: " << (strictMode ? "Enabled" : "Disabled") << std::endl;
        
        for (const auto& [paramName, rule] : validationRules) {
            std::cout << "\nParameter: " << paramName << std::endl;
            std::cout << "  Type: " << rule.dataType << std::endl;
            std::cout << "  Required: " << (rule.required ? "Yes" : "No") << std::endl;
            
            if (rule.minValue.has_value() && rule.maxValue.has_value()) {
                try {
                    double minVal = std::any_cast<double>(rule.minValue);
                    double maxVal = std::any_cast<double>(rule.maxValue);
                    std::cout << "  Range: [" << minVal << ", " << maxVal << "]" << std::endl;
                } catch (const std::bad_any_cast&) {
                    std::cout << "  Range: defined" << std::endl;
                }
            }
            
            if (!rule.allowedValues.empty()) {
                std::cout << "  Allowed values: " << rule.allowedValues.size() << " values" << std::endl;
            }
            
            if (rule.customValidator) {
                std::cout << "  Custom validator: Yes" << std::endl;
            }
        }
        
        std::cout << "[PARAM_VALIDATOR] === END REPORT ===" << std::endl;
    }
    
private:
    void setupDefaultRules() {
        // Navigation algorithm parameters
        addRule("algorithm.type", "string", true);
        setAllowedValues("algorithm.type", {"astar", "dijkstra", "bfs", "bellman_ford"});
        
        addRule("algorithm.heuristic", "string", false);
        setAllowedValues("algorithm.heuristic", {"euclidean", "manhattan", "chebyshev"});
        
        // Performance parameters
        addRule("performance.max_iterations", "integer", false);
        setRange("performance.max_iterations", 1, 1000000);
        
        addRule("performance.timeout_seconds", "double", false);
        setRange("performance.timeout_seconds", 0.1, 3600.0);
        
        // Graph parameters
        addRule("graph.max_nodes", "integer", false);
        setRange("graph.max_nodes", 1, 100000);
        
        addRule("graph.allow_negative_weights", "boolean", false);
        
        // Visualization parameters
        addRule("visualization.enabled", "boolean", false);
        addRule("visualization.canvas_width", "integer", false);
        setRange("visualization.canvas_width", 100, 5000);
        
        addRule("visualization.canvas_height", "integer", false);
        setRange("visualization.canvas_height", 100, 5000);
        
        // Logging parameters
        addRule("logging.level", "string", false);
        setAllowedValues("logging.level", {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"});
        
        addRule("logging.console_output", "boolean", false);
        addRule("logging.file_output", "boolean", false);
        
        std::cout << "[PARAM_VALIDATOR] Set up " << validationRules.size() << " default rules" << std::endl;
    }
    
    void validateParameter(const std::string& paramName, const std::any& value) {
        auto it = validationRules.find(paramName);
        if (it == validationRules.end()) {
            // Unknown parameter handled in main validation
            return;
        }
        
        const ValidationRule& rule = it->second;
        
        // Type validation
        if (!validateType(paramName, value, rule.dataType)) {
            return; // Error already added
        }
        
        // Range validation
        if (rule.minValue.has_value() || rule.maxValue.has_value()) {
            if (!validateRange(paramName, value, rule)) {
                return; // Error already added
            }
        }
        
        // Allowed values validation
        if (!rule.allowedValues.empty()) {
            if (!validateAllowedValues(paramName, value, rule.allowedValues)) {
                return; // Error already added
            }
        }
        
        // Custom validation
        if (rule.customValidator) {
            if (!rule.customValidator(value)) {
                addError("Parameter '" + paramName + "' failed custom validation");
                return;
            }
        }
    }
    
    bool validateType(const std::string& paramName, const std::any& value, const std::string& expectedType) {
        try {
            if (expectedType == "string") {
                std::any_cast<std::string>(value);
            } else if (expectedType == "int" || expectedType == "integer") {
                std::any_cast<int>(value);
            } else if (expectedType == "double" || expectedType == "float") {
                std::any_cast<double>(value);
            } else if (expectedType == "bool" || expectedType == "boolean") {
                std::any_cast<bool>(value);
            } else {
                addError("Parameter '" + paramName + "' has unknown expected type: " + expectedType);
                return false;
            }
            return true;
        } catch (const std::bad_any_cast&) {
            addError("Parameter '" + paramName + "' has incorrect type (expected " + expectedType + ")");
            return false;
        }
    }
    
    bool validateRange(const std::string& paramName, const std::any& value, const ValidationRule& rule) {
        try {
            if (rule.dataType == "double" || rule.dataType == "float") {
                double val = std::any_cast<double>(value);
                
                if (rule.minValue.has_value()) {
                    double minVal = std::any_cast<double>(rule.minValue);
                    if (val < minVal) {
                        addError("Parameter '" + paramName + "' (" + std::to_string(val) + 
                                ") is below minimum (" + std::to_string(minVal) + ")");
                        return false;
                    }
                }
                
                if (rule.maxValue.has_value()) {
                    double maxVal = std::any_cast<double>(rule.maxValue);
                    if (val > maxVal) {
                        addError("Parameter '" + paramName + "' (" + std::to_string(val) + 
                                ") is above maximum (" + std::to_string(maxVal) + ")");
                        return false;
                    }
                }
            } else if (rule.dataType == "int" || rule.dataType == "integer") {
                int val = std::any_cast<int>(value);
                
                if (rule.minValue.has_value()) {
                    int minVal = std::any_cast<int>(rule.minValue);
                    if (val < minVal) {
                        addError("Parameter '" + paramName + "' (" + std::to_string(val) + 
                                ") is below minimum (" + std::to_string(minVal) + ")");
                        return false;
                    }
                }
                
                if (rule.maxValue.has_value()) {
                    int maxVal = std::any_cast<int>(rule.maxValue);
                    if (val > maxVal) {
                        addError("Parameter '" + paramName + "' (" + std::to_string(val) + 
                                ") is above maximum (" + std::to_string(maxVal) + ")");
                        return false;
                    }
                }
            }
        } catch (const std::bad_any_cast&) {
            // Type error already handled
        }
        
        return true;
    }
    
    bool validateAllowedValues(const std::string& paramName, const std::any& value, 
                              const std::vector<std::any>& allowedValues) {
        
        for (const auto& allowedValue : allowedValues) {
            if (valuesEqual(value, allowedValue)) {
                return true;
            }
        }
        
        std::string valueStr = valueToString(value);
        addError("Parameter '" + paramName + "' value '" + valueStr + "' is not in allowed values");
        return false;
    }
    
    bool valuesEqual(const std::any& a, const std::any& b) {
        try {
            // Try string comparison
            auto strA = std::any_cast<std::string>(a);
            auto strB = std::any_cast<std::string>(b);
            return strA == strB;
        } catch (const std::bad_any_cast&) {}
        
        try {
            // Try integer comparison
            auto intA = std::any_cast<int>(a);
            auto intB = std::any_cast<int>(b);
            return intA == intB;
        } catch (const std::bad_any_cast&) {}
        
        try {
            // Try double comparison
            auto doubleA = std::any_cast<double>(a);
            auto doubleB = std::any_cast<double>(b);
            return std::abs(doubleA - doubleB) < 1e-9;
        } catch (const std::bad_any_cast&) {}
        
        try {
            // Try boolean comparison
            auto boolA = std::any_cast<bool>(a);
            auto boolB = std::any_cast<bool>(b);
            return boolA == boolB;
        } catch (const std::bad_any_cast&) {}
        
        return false;
    }
    
    std::string valueToString(const std::any& value) {
        try {
            if (auto strVal = std::any_cast<std::string>(&value)) {
                return *strVal;
            } else if (auto intVal = std::any_cast<int>(&value)) {
                return std::to_string(*intVal);
            } else if (auto doubleVal = std::any_cast<double>(&value)) {
                return std::to_string(*doubleVal);
            } else if (auto boolVal = std::any_cast<bool>(&value)) {
                return *boolVal ? "true" : "false";
            }
        } catch (const std::bad_any_cast&) {}
        
        return "[unknown type]";
    }
    
    void addError(const std::string& error) {
        validationErrors.push_back(error);
    }
};

// Global validator instance
static std::unique_ptr<ParameterValidator> g_paramValidator;

bool validateParameters(const std::unordered_map<std::string, std::any>& config) {
    if (!g_paramValidator) {
        g_paramValidator = std::make_unique<ParameterValidator>();
    }
    return g_paramValidator->validateConfiguration(config);
}

std::vector<std::string> getParameterValidationErrors() {
    if (!g_paramValidator) {
        return {};
    }
    return g_paramValidator->getValidationErrors();
}

void addParameterRule(const std::string& paramName, const std::string& type, bool required) {
    if (!g_paramValidator) {
        g_paramValidator = std::make_unique<ParameterValidator>();
    }
    g_paramValidator->addRule(paramName, type, required);
}

void setParameterRange(const std::string& paramName, double minVal, double maxVal) {
    if (!g_paramValidator) {
        g_paramValidator = std::make_unique<ParameterValidator>();
    }
    g_paramValidator->setRange(paramName, minVal, maxVal);
}

bool fixConfigurationIssues(std::unordered_map<std::string, std::any>& config) {
    if (!g_paramValidator) {
        g_paramValidator = std::make_unique<ParameterValidator>();
    }
    return g_paramValidator->fixCommonIssues(config);
}