#include "environment/EnvironmentManager.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <filesystem>

class EnvironmentPersistenceManager {
private:
    EnvironmentManager* environmentManager;
    std::string defaultStoragePath;
    std::string backupDirectory;
    size_t maxBackupCount;
    bool compressionEnabled;
    
public:
    explicit EnvironmentPersistenceManager(EnvironmentManager* envManager) 
        : environmentManager(envManager), defaultStoragePath("environments/"),
          backupDirectory("environments/backups/"), maxBackupCount(10), compressionEnabled(false) {
        
        initializeStorageDirectories();
    }
    
    bool saveEnvironmentToFile(const std::string& filename) const {
        std::cout << "[ENV_PERSISTENCE] Saving environment to file: " << filename << std::endl;
        
        auto environment = environmentManager->getCurrentEnvironment();
        if (!environment) {
            std::cout << "[ENV_PERSISTENCE] No environment available to save" << std::endl;
            return false;
        }
        
        std::string fullPath = constructFullPath(filename);
        
        try {
            std::ofstream file(fullPath, std::ios::binary);
            if (!file.is_open()) {
                std::cout << "[ENV_PERSISTENCE] Failed to open file for writing: " << fullPath << std::endl;
                return false;
            }
            
            // Write environment metadata
            writeEnvironmentHeader(file);
            
            // Write node data
            if (!writeNodeData(file, environment.get())) {
                std::cout << "[ENV_PERSISTENCE] Failed to write node data" << std::endl;
                return false;
            }
            
            // Write edge data
            if (!writeEdgeData(file, environment.get())) {
                std::cout << "[ENV_PERSISTENCE] Failed to write edge data" << std::endl;
                return false;
            }
            
            // Write environment properties
            writeEnvironmentProperties(file);
            
            file.close();
            
            std::cout << "[ENV_PERSISTENCE] Environment saved successfully to: " << fullPath << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[ENV_PERSISTENCE] Exception during save operation: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool loadEnvironmentFromFile(const std::string& filename) {
        std::cout << "[ENV_PERSISTENCE] Loading environment from file: " << filename << std::endl;
        
        std::string fullPath = constructFullPath(filename);
        
        if (!std::filesystem::exists(fullPath)) {
            std::cout << "[ENV_PERSISTENCE] Environment file does not exist: " << fullPath << std::endl;
            return false;
        }
        
        try {
            std::ifstream file(fullPath, std::ios::binary);
            if (!file.is_open()) {
                std::cout << "[ENV_PERSISTENCE] Failed to open file for reading: " << fullPath << std::endl;
                return false;
            }
            
            // Validate file format
            if (!validateFileFormat(file)) {
                std::cout << "[ENV_PERSISTENCE] Invalid file format: " << fullPath << std::endl;
                return false;
            }
            
            // Create new environment
            auto newEnvironment = std::make_shared<Graph>();
            
            // Read node data
            if (!readNodeData(file, newEnvironment.get())) {
                std::cout << "[ENV_PERSISTENCE] Failed to read node data" << std::endl;
                return false;
            }
            
            // Read edge data
            if (!readEdgeData(file, newEnvironment.get())) {
                std::cout << "[ENV_PERSISTENCE] Failed to read edge data" << std::endl;
                return false;
            }
            
            // Read environment properties
            readEnvironmentProperties(file);
            
            file.close();
            
            // Set the loaded environment
            std::string environmentName = extractEnvironmentName(filename);
            environmentManager->setEnvironment(newEnvironment, environmentName);
            
            std::cout << "[ENV_PERSISTENCE] Environment loaded successfully from: " << fullPath << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[ENV_PERSISTENCE] Exception during load operation: " << e.what() << std::endl;
            return false;
        }
    }
    
    void createEnvironmentBackup() {
        std::cout << "[ENV_PERSISTENCE] Creating environment backup" << std::endl;
        
        // Generate backup filename with timestamp
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);
        
        std::ostringstream backupName;
        backupName << "env_backup_" << std::put_time(std::localtime(&timeT), "%Y%m%d_%H%M%S") << ".env";
        
        std::string backupPath = backupDirectory + backupName.str();
        
        if (saveEnvironmentToPath(backupPath)) {
            std::cout << "[ENV_PERSISTENCE] Backup created: " << backupPath << std::endl;
            
            // Manage backup count
            manageBackupFiles();
        } else {
            std::cout << "[ENV_PERSISTENCE] Failed to create backup" << std::endl;
        }
    }
    
    bool restoreFromBackup(const std::string& backupFilename) {
        std::cout << "[ENV_PERSISTENCE] Restoring from backup: " << backupFilename << std::endl;
        
        std::string backupPath = backupDirectory + backupFilename;
        
        if (!std::filesystem::exists(backupPath)) {
            std::cout << "[ENV_PERSISTENCE] Backup file does not exist: " << backupPath << std::endl;
            return false;
        }
        
        return loadEnvironmentFromPath(backupPath);
    }
    
    std::vector<std::string> listAvailableBackups() const {
        std::vector<std::string> backups;
        
        try {
            if (std::filesystem::exists(backupDirectory)) {
                for (const auto& entry : std::filesystem::directory_iterator(backupDirectory)) {
                    if (entry.is_regular_file() && entry.path().extension() == ".env") {
                        backups.push_back(entry.path().filename().string());
                    }
                }
                
                // Sort backups by modification time (newest first)
                std::sort(backups.begin(), backups.end(), [&](const std::string& a, const std::string& b) {
                    auto timeA = std::filesystem::last_write_time(backupDirectory + a);
                    auto timeB = std::filesystem::last_write_time(backupDirectory + b);
                    return timeA > timeB;
                });
            }
        } catch (const std::exception& e) {
            std::cout << "[ENV_PERSISTENCE] Error listing backups: " << e.what() << std::endl;
        }
        
        return backups;
    }
    
    void setStoragePath(const std::string& path) {
        defaultStoragePath = path;
        if (!defaultStoragePath.empty() && defaultStoragePath.back() != '/') {
            defaultStoragePath += '/';
        }
        
        initializeStorageDirectories();
        std::cout << "[ENV_PERSISTENCE] Storage path set to: " << defaultStoragePath << std::endl;
    }
    
    void enableCompression(bool enable) {
        compressionEnabled = enable;
        std::cout << "[ENV_PERSISTENCE] Compression " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void setMaxBackupCount(size_t count) {
        maxBackupCount = count;
        std::cout << "[ENV_PERSISTENCE] Maximum backup count set to: " << count << std::endl;
    }
    
    bool exportEnvironmentToJSON(const std::string& jsonFilename) const {
        std::cout << "[ENV_PERSISTENCE] Exporting environment to JSON: " << jsonFilename << std::endl;
        
        auto environment = environmentManager->getCurrentEnvironment();
        if (!environment) {
            std::cout << "[ENV_PERSISTENCE] No environment available to export" << std::endl;
            return false;
        }
        
        std::string fullPath = constructFullPath(jsonFilename);
        
        try {
            std::ofstream file(fullPath);
            if (!file.is_open()) {
                std::cout << "[ENV_PERSISTENCE] Failed to open JSON file for writing" << std::endl;
                return false;
            }
            
            file << "{\n";
            file << "  \"environment\": {\n";
            file << "    \"metadata\": {\n";
            file << "      \"name\": \"" << environmentManager->getEnvironmentName() << "\",\n";
            file << "      \"nodeCount\": " << environment->getNodeCount() << ",\n";
            file << "      \"edgeCount\": " << environment->getEdgeCount() << ",\n";
            file << "      \"exportTimestamp\": \"" << getCurrentTimestamp() << "\"\n";
            file << "    },\n";
            
            // Export nodes
            file << "    \"nodes\": [\n";
            exportNodesAsJSON(file, environment.get());
            file << "    ],\n";
            
            // Export edges
            file << "    \"edges\": [\n";
            exportEdgesAsJSON(file, environment.get());
            file << "    ]\n";
            
            file << "  }\n";
            file << "}\n";
            
            file.close();
            
            std::cout << "[ENV_PERSISTENCE] Environment exported to JSON successfully" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[ENV_PERSISTENCE] Exception during JSON export: " << e.what() << std::endl;
            return false;
        }
    }
    
private:
    void initializeStorageDirectories() {
        try {
            std::filesystem::create_directories(defaultStoragePath);
            std::filesystem::create_directories(backupDirectory);
        } catch (const std::exception& e) {
            std::cout << "[ENV_PERSISTENCE] Failed to create storage directories: " << e.what() << std::endl;
        }
    }
    
    std::string constructFullPath(const std::string& filename) const {
        if (filename.find('/') != std::string::npos || filename.find('\\') != std::string::npos) {
            return filename; // Already a full path
        }
        return defaultStoragePath + filename;
    }
    
    bool saveEnvironmentToPath(const std::string& fullPath) const {
        auto environment = environmentManager->getCurrentEnvironment();
        if (!environment) {
            return false;
        }
        
        try {
            std::ofstream file(fullPath, std::ios::binary);
            if (!file.is_open()) {
                return false;
            }
            
            writeEnvironmentHeader(file);
            writeNodeData(file, environment.get());
            writeEdgeData(file, environment.get());
            writeEnvironmentProperties(file);
            
            file.close();
            return true;
            
        } catch (const std::exception&) {
            return false;
        }
    }
    
    bool loadEnvironmentFromPath(const std::string& fullPath) {
        try {
            std::ifstream file(fullPath, std::ios::binary);
            if (!file.is_open()) {
                return false;
            }
            
            if (!validateFileFormat(file)) {
                return false;
            }
            
            auto newEnvironment = std::make_shared<Graph>();
            
            readNodeData(file, newEnvironment.get());
            readEdgeData(file, newEnvironment.get());
            readEnvironmentProperties(file);
            
            file.close();
            
            std::string environmentName = std::filesystem::path(fullPath).stem().string();
            environmentManager->setEnvironment(newEnvironment, environmentName);
            
            return true;
            
        } catch (const std::exception&) {
            return false;
        }
    }
    
    void writeEnvironmentHeader(std::ofstream& file) const {
        // Write file format identifier
        const std::string header = "NAVENV_V1.0";
        file.write(header.c_str(), header.length());
        
        // Write timestamp
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);
        file.write(reinterpret_cast<const char*>(&timeT), sizeof(timeT));
        
        // Write environment name length and name
        std::string envName = environmentManager->getEnvironmentName();
        uint32_t nameLength = static_cast<uint32_t>(envName.length());
        file.write(reinterpret_cast<const char*>(&nameLength), sizeof(nameLength));
        file.write(envName.c_str(), nameLength);
    }
    
    bool writeNodeData(std::ofstream& file, const Graph* environment) const {
        uint32_t nodeCount = static_cast<uint32_t>(environment->getNodeCount());
        file.write(reinterpret_cast<const char*>(&nodeCount), sizeof(nodeCount));
        
        std::vector<int> nodeIds = environment->getAllNodeIds();
        for (int nodeId : nodeIds) {
            const Node& node = environment->getNode(nodeId);
            
            // Write node ID
            file.write(reinterpret_cast<const char*>(&nodeId), sizeof(nodeId));
            
            // Write node name
            std::string nodeName = node.getName();
            uint32_t nameLength = static_cast<uint32_t>(nodeName.length());
            file.write(reinterpret_cast<const char*>(&nameLength), sizeof(nameLength));
            file.write(nodeName.c_str(), nameLength);
            
            // Write node coordinates
            double x = node.getX();
            double y = node.getY();
            file.write(reinterpret_cast<const char*>(&x), sizeof(x));
            file.write(reinterpret_cast<const char*>(&y), sizeof(y));
        }
        
        return file.good();
    }
    
    bool writeEdgeData(std::ofstream& file, const Graph* environment) const {
        // Count total edges
        uint32_t totalEdges = 0;
        std::vector<int> nodeIds = environment->getAllNodeIds();
        for (int nodeId : nodeIds) {
            totalEdges += static_cast<uint32_t>(environment->getEdgesFrom(nodeId).size());
        }
        
        file.write(reinterpret_cast<const char*>(&totalEdges), sizeof(totalEdges));
        
        // Write edge data
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = environment->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                int fromNode = edge.getFromNode();
                int toNode = edge.getToNode();
                double weight = edge.getWeight();
                bool bidirectional = edge.isBidirectional();
                
                file.write(reinterpret_cast<const char*>(&fromNode), sizeof(fromNode));
                file.write(reinterpret_cast<const char*>(&toNode), sizeof(toNode));
                file.write(reinterpret_cast<const char*>(&weight), sizeof(weight));
                file.write(reinterpret_cast<const char*>(&bidirectional), sizeof(bidirectional));
            }
        }
        
        return file.good();
    }
    
    void writeEnvironmentProperties(std::ofstream& file) const {
        // Write environment complexity
        size_t complexity = environmentManager->getEnvironmentComplexity();
        file.write(reinterpret_cast<const char*>(&complexity), sizeof(complexity));
        
        // Write validation status
        bool isValid = environmentManager->validateCurrentEnvironment();
        file.write(reinterpret_cast<const char*>(&isValid), sizeof(isValid));
    }
    
    bool validateFileFormat(std::ifstream& file) const {
        const std::string expectedHeader = "NAVENV_V1.0";
        std::string fileHeader(expectedHeader.length(), '\0');
        
        file.read(&fileHeader[0], expectedHeader.length());
        
        return fileHeader == expectedHeader;
    }
    
    bool readNodeData(std::ifstream& file, Graph* environment) const {
        uint32_t nodeCount;
        file.read(reinterpret_cast<char*>(&nodeCount), sizeof(nodeCount));
        
        for (uint32_t i = 0; i < nodeCount; ++i) {
            int nodeId;
            file.read(reinterpret_cast<char*>(&nodeId), sizeof(nodeId));
            
            uint32_t nameLength;
            file.read(reinterpret_cast<char*>(&nameLength), sizeof(nameLength));
            
            std::string nodeName(nameLength, '\0');
            file.read(&nodeName[0], nameLength);
            
            double x, y;
            file.read(reinterpret_cast<char*>(&x), sizeof(x));
            file.read(reinterpret_cast<char*>(&y), sizeof(y));
            
            environment->addNode(nodeId, nodeName, x, y);
        }
        
        return file.good();
    }
    
    bool readEdgeData(std::ifstream& file, Graph* environment) const {
        uint32_t edgeCount;
        file.read(reinterpret_cast<char*>(&edgeCount), sizeof(edgeCount));
        
        for (uint32_t i = 0; i < edgeCount; ++i) {
            int fromNode, toNode;
            double weight;
            bool bidirectional;
            
            file.read(reinterpret_cast<char*>(&fromNode), sizeof(fromNode));
            file.read(reinterpret_cast<char*>(&toNode), sizeof(toNode));
            file.read(reinterpret_cast<char*>(&weight), sizeof(weight));
            file.read(reinterpret_cast<char*>(&bidirectional), sizeof(bidirectional));
            
            environment->addEdge(fromNode, toNode, weight, bidirectional);
        }
        
        return file.good();
    }
    
    void readEnvironmentProperties(std::ifstream& file) const {
        size_t complexity;
        file.read(reinterpret_cast<char*>(&complexity), sizeof(complexity));
        
        bool isValid;
        file.read(reinterpret_cast<char*>(&isValid), sizeof(isValid));
    }
    
    void manageBackupFiles() {
        try {
            std::vector<std::filesystem::path> backupFiles;
            
            if (std::filesystem::exists(backupDirectory)) {
                for (const auto& entry : std::filesystem::directory_iterator(backupDirectory)) {
                    if (entry.is_regular_file() && entry.path().extension() == ".env") {
                        backupFiles.push_back(entry.path());
                    }
                }
                
                if (backupFiles.size() > maxBackupCount) {
                    // Sort by modification time (oldest first)
                    std::sort(backupFiles.begin(), backupFiles.end(), 
                             [](const std::filesystem::path& a, const std::filesystem::path& b) {
                                 return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
                             });
                    
                    // Remove oldest backups
                    size_t filesToRemove = backupFiles.size() - maxBackupCount;
                    for (size_t i = 0; i < filesToRemove; ++i) {
                        std::filesystem::remove(backupFiles[i]);
                        std::cout << "[ENV_PERSISTENCE] Removed old backup: " << backupFiles[i].filename() << std::endl;
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cout << "[ENV_PERSISTENCE] Error managing backup files: " << e.what() << std::endl;
        }
    }
    
    std::string extractEnvironmentName(const std::string& filename) const {
        std::filesystem::path filePath(filename);
        return filePath.stem().string();
    }
    
    std::string getCurrentTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);
        
        std::ostringstream timestamp;
        timestamp << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S");
        return timestamp.str();
    }
    
    void exportNodesAsJSON(std::ofstream& file, const Graph* environment) const {
        std::vector<int> nodeIds = environment->getAllNodeIds();
        
        for (size_t i = 0; i < nodeIds.size(); ++i) {
            const Node& node = environment->getNode(nodeIds[i]);
            
            file << "      {\n";
            file << "        \"id\": " << nodeIds[i] << ",\n";
            file << "        \"name\": \"" << node.getName() << "\",\n";
            file << "        \"x\": " << node.getX() << ",\n";
            file << "        \"y\": " << node.getY() << "\n";
            file << "      }";
            
            if (i < nodeIds.size() - 1) {
                file << ",";
            }
            file << "\n";
        }
    }
    
    void exportEdgesAsJSON(std::ofstream& file, const Graph* environment) const {
        std::vector<int> nodeIds = environment->getAllNodeIds();
        bool firstEdge = true;
        
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = environment->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                if (!firstEdge) {
                    file << ",\n";
                }
                firstEdge = false;
                
                file << "      {\n";
                file << "        \"from\": " << edge.getFromNode() << ",\n";
                file << "        \"to\": " << edge.getToNode() << ",\n";
                file << "        \"weight\": " << edge.getWeight() << ",\n";
                file << "        \"bidirectional\": " << (edge.isBidirectional() ? "true" : "false") << "\n";
                file << "      }";
            }
        }
        
        if (!firstEdge) {
            file << "\n";
        }
    }
};

// Implementation methods for EnvironmentManager
void EnvironmentManager::saveEnvironment(const std::string& filename) const {
    static EnvironmentPersistenceManager persistenceManager(const_cast<EnvironmentManager*>(this));
    persistenceManager.saveEnvironmentToFile(filename);
}

bool EnvironmentManager::loadEnvironment(const std::string& filename) {
    static EnvironmentPersistenceManager persistenceManager(this);
    return persistenceManager.loadEnvironmentFromFile(filename);
}

void EnvironmentManager::backupCurrentEnvironment() {
    static EnvironmentPersistenceManager persistenceManager(this);
    persistenceManager.createEnvironmentBackup();
}

bool EnvironmentManager::restorePreviousEnvironment() {
    static EnvironmentPersistenceManager persistenceManager(this);
    
    std::vector<std::string> backups = persistenceManager.listAvailableBackups();
    if (!backups.empty()) {
        return persistenceManager.restoreFromBackup(backups[0]); // Restore most recent
    }
    
    std::cout << "[ENV_MANAGER] No backups available for restoration" << std::endl;
    return false;
}