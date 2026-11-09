#pragma once
// Minimal test helpers for the adapter-core tests (no test framework dependency).
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace ane::test {

inline int& failureCount() {
    static int count = 0;
    return count;
}

inline int& checkCount() {
    static int count = 0;
    return count;
}

inline void checkThat(bool ok, const char* expression, const char* file, int line, const std::string& note = {}) {
    ++checkCount();
    if (!ok) {
        ++failureCount();
        std::cerr << file << ":" << line << ": CHECK failed: " << expression;
        if (!note.empty()) {
            std::cerr << " (" << note << ")";
        }
        std::cerr << std::endl;
    }
}

struct Case {
    std::string name;
    std::function<void()> body;
};

inline int runCases(const char* program, const std::vector<Case>& cases) {
    for (const Case& c : cases) {
        const int before = failureCount();
        try {
            c.body();
        } catch (const std::exception& error) {
            ++failureCount();
            std::cerr << c.name << ": unexpected exception: " << error.what() << std::endl;
        }
        std::cout << (failureCount() == before ? "[PASS] " : "[FAIL] ") << c.name << std::endl;
    }
    std::cout << program << ": " << cases.size() << " cases, " << checkCount() << " checks, " << failureCount()
              << " failures" << std::endl;
    return failureCount() == 0 ? 0 : 1;
}

}  // namespace ane::test

#define CHECK(cond) ::ane::test::checkThat(static_cast<bool>(cond), #cond, __FILE__, __LINE__)
#define CHECK_NOTE(cond, note) ::ane::test::checkThat(static_cast<bool>(cond), #cond, __FILE__, __LINE__, (note))
#define CHECK_NEAR(a, b, tol) \
    ::ane::test::checkThat(std::abs((a) - (b)) <= (tol), #a " ~ " #b, __FILE__, __LINE__, std::to_string(a) + " vs " + std::to_string(b))
