#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "MCSlam/ORBVocabulary.h"

namespace {

std::string descriptorText(int fill) {
    std::string s;
    for (int i = 0; i < 32; ++i) s += std::to_string((fill + i) % 256) + " ";
    return s;
}

class TempFile {
public:
    explicit TempFile(const std::string& contents) : path_(std::tmpnam(nullptr)) {
        std::ofstream(path_) << contents;
    }
    ~TempFile() { std::remove(path_.c_str()); }
    const std::string& path() const { return path_; }

private:
    std::string path_;
};

// k=2, L=2 tree: root -> {1, 2}; 1 -> {3, 4} leaves; 2 -> {5, 6} leaves.
std::string tinyVocabulary() {
    std::string v = "2 2 0 0\n";
    v += "0 0 " + descriptorText(0) + "0\n";
    v += "0 0 " + descriptorText(128) + "0\n";
    v += "1 1 " + descriptorText(1) + "0.5\n";
    v += "1 1 " + descriptorText(9) + "0.25\n";
    v += "2 1 " + descriptorText(129) + "0.75\n";
    v += "2 1 " + descriptorText(140) + "1.0\n";
    return v;
}

}  // namespace

TEST(OrbVocabulary, LoadsTextTree) {
    TempFile f(tinyVocabulary());
    mcslam::OrbVocabulary voc;
    ASSERT_TRUE(voc.loadFromTextFile(f.path()));
    EXPECT_EQ(voc.getBranchingFactor(), 2);
    EXPECT_EQ(voc.getDepthLevels(), 2);
    EXPECT_EQ(voc.size(), 4u);  // number of words (leaves)
    EXPECT_DOUBLE_EQ(voc.getWordWeight(0), 0.5);
    EXPECT_DOUBLE_EQ(voc.getWordWeight(3), 1.0);
}

TEST(OrbVocabulary, TransformsDescriptorToNearestWord) {
    TempFile f(tinyVocabulary());
    mcslam::OrbVocabulary voc;
    ASSERT_TRUE(voc.loadFromTextFile(f.path()));
    cv::Mat d(1, 32, CV_8U);
    for (int i = 0; i < 32; ++i) d.at<uint8_t>(i) = static_cast<uint8_t>((140 + i) % 256);
    EXPECT_EQ(voc.transform(d), 3u);
}

TEST(OrbVocabulary, RejectsMalformedFiles) {
    mcslam::OrbVocabulary voc;
    EXPECT_FALSE(voc.loadFromTextFile("/nonexistent/voc.txt"));
    TempFile bad_header("not a header\n");
    EXPECT_FALSE(voc.loadFromTextFile(bad_header.path()));
    TempFile bad_parent("2 2 0 0\n7 1 " + descriptorText(0) + "0.5\n");
    EXPECT_FALSE(voc.loadFromTextFile(bad_parent.path()));
    TempFile short_desc("2 2 0 0\n0 1 1 2 3\n");
    EXPECT_FALSE(voc.loadFromTextFile(short_desc.path()));
}
