#include <gtest/gtest.h>

#include <drone_mapper/utils/ErrorLogger.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

// The fixture is at global scope and named "ErrorLogger" so --gtest_filter=ErrorLogger.*
// routes here; production-type references use the fully-qualified drone_mapper::ErrorLogger
// so the fixture name does not shadow the production class.

namespace fs = std::filesystem;

namespace {

// Reads an entire file into a string (empty string if it does not exist).
std::string readFile(const fs::path& path) {
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// Counts log lines by counting newline terminators (each entry ends with '\n').
std::size_t countLines(const std::string& text) {
    return static_cast<std::size_t>(std::count(text.begin(), text.end(), '\n'));
}

} // namespace

// Each test gets its own unique temp directory, created in SetUp and removed in TearDown,
// so tests never collide and leave no artifacts behind.
class ErrorLogger : public ::testing::Test {
protected:
    fs::path dir_;

    void SetUp() override {
        dir_ = fs::temp_directory_path() /
               ("errlog_" + std::to_string(::getpid()) + "_" +
                std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        fs::create_directories(dir_);
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    fs::path logPath() const { return dir_ / "errors.log"; }
};

// Constructing the logger only stores the path; it must not create the file (lazy creation).
TEST_F(ErrorLogger, NoFileCreatedByConstructor) {
    drone_mapper::ErrorLogger logger{logPath()};
    EXPECT_FALSE(fs::exists(logPath()));
}

// The first log() call creates the file.
TEST_F(ErrorLogger, FileCreatedOnFirstLog) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("CODE", "message");
    EXPECT_TRUE(fs::exists(logPath()));
}

// A nested path whose directories do not exist must be created lazily, so callers can pass
// paths like "output_results/run_1/errors.log" without pre-creating the tree.
TEST_F(ErrorLogger, CreatesParentDirectories) {
    const fs::path nested = dir_ / "a" / "b" / "c" / "errors.log";
    drone_mapper::ErrorLogger logger{nested};
    logger.log("CODE", "message");
    EXPECT_TRUE(fs::exists(nested));
}

// The exact code and message strings must appear unchanged (guards garbled/swapped fields).
TEST_F(ErrorLogger, CodeAndMessageAppearVerbatim) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("DRONE_HITS_OBSTACLE", "collision at (1,2,3)");
    const std::string content = readFile(logPath());
    EXPECT_NE(content.find("DRONE_HITS_OBSTACLE"), std::string::npos);
    EXPECT_NE(content.find("collision at (1,2,3)"), std::string::npos);
}

// The line follows "[<timestamp>] ERROR <code>: <message>" (guards format regressions).
TEST_F(ErrorLogger, LineFormatHasBracketTimestampErrorKeywordAndColon) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("CODE_X", "the message");
    const std::string content = readFile(logPath());
    ASSERT_FALSE(content.empty());
    EXPECT_EQ(content.front(), '[') << "line should start with the [timestamp] bracket";
    EXPECT_NE(content.find("] ERROR "), std::string::npos);
    EXPECT_NE(content.find("CODE_X: the message"), std::string::npos);
}

// The timestamp is ISO 8601 UTC: it carries the 'T' date/time separator and a trailing 'Z'.
TEST_F(ErrorLogger, TimestampLooksLikeIso8601Utc) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("C", "m");
    const std::string content = readFile(logPath());
    EXPECT_NE(content.find('T'), std::string::npos);
    EXPECT_NE(content.find("Z]"), std::string::npos);
}

// Every entry is newline-terminated so consecutive entries land on their own lines.
TEST_F(ErrorLogger, EachEntryEndsWithNewline) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("C", "m");
    const std::string content = readFile(logPath());
    ASSERT_FALSE(content.empty());
    EXPECT_EQ(content.back(), '\n');
}

// A second log() appends a second line (size grows, order preserved) — guards ios::trunc.
TEST_F(ErrorLogger, SecondLogCallAppendsSecondLine) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("FIRST", "one");
    const auto size_after_first = fs::file_size(logPath());
    logger.log("SECOND", "two");
    const auto size_after_second = fs::file_size(logPath());

    EXPECT_GT(size_after_second, size_after_first) << "file must grow on the second log";
    const std::string content = readFile(logPath());
    EXPECT_EQ(countLines(content), 2u);
    EXPECT_LT(content.find("FIRST"), content.find("SECOND")) << "order must be preserved";
}

// Logging to a pre-existing file appends rather than truncating its prior contents.
TEST_F(ErrorLogger, DoesNotTruncateExistingFile) {
    { std::ofstream seed(logPath()); seed << "PRE_EXISTING\n"; }
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("NEW", "entry");
    const std::string content = readFile(logPath());
    EXPECT_NE(content.find("PRE_EXISTING"), std::string::npos);
    EXPECT_NE(content.find("NEW"), std::string::npos);
}

// Each log() flushes before returning: the file grows after every call with no explicit
// close between calls, so entries are durable even if the process dies right after.
TEST_F(ErrorLogger, EachWriteImmediatelyFlushesAndGrowsFile) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("A", "alpha");
    const auto size1 = fs::file_size(logPath());
    EXPECT_GT(size1, 0u);
    logger.log("B", "beta");
    const auto size2 = fs::file_size(logPath());
    EXPECT_GT(size2, size1);
    logger.log("C", "gamma");
    const auto size3 = fs::file_size(logPath());
    EXPECT_GT(size3, size2);
}

// Empty code and message are still written as one well-formed line (no crash, no skipped entry).
TEST_F(ErrorLogger, EmptyCodeAndMessageStillWritesOneLine) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("", "");
    const std::string content = readFile(logPath());
    EXPECT_EQ(countLines(content), 1u);
    EXPECT_NE(content.find("ERROR"), std::string::npos);
}

// Three logs produce exactly three lines (line accounting across multiple appends).
TEST_F(ErrorLogger, ThreeLogsProduceThreeLines) {
    drone_mapper::ErrorLogger logger{logPath()};
    logger.log("A", "1");
    logger.log("B", "2");
    logger.log("C", "3");
    EXPECT_EQ(countLines(readFile(logPath())), 3u);
}

// When the target path cannot be opened as a file, log() throws (it does not silently swallow).
// We make the path a directory so opening it for writing fails.
TEST_F(ErrorLogger, ThrowsWhenLogPathCannotBeOpened) {
    fs::create_directory(logPath()); // logPath() now exists as a directory, not a file
    drone_mapper::ErrorLogger logger{logPath()};
    EXPECT_THROW(logger.log("CODE", "message"), std::runtime_error);
}
