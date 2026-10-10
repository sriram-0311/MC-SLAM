// DBoW2 bag-of-words vectors for a list of images, with the ORB extractor settings of the EuRoC
// front end and the vocabulary used for loop closure. One output line per image:
//   <number of words> <word id>:<weight> ...
// The vectors are L1-normalised TF-IDF weights, so the DBoW2 L1 score of two images is the sum of
// the element-wise minimum of their vectors.
//
//   bow_vectors <ORBvoc.txt> <image list file> <output file>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include "MCSlam/ORBVocabulary.h"
#include "MCSlam/ORBextractor.h"

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: bow_vectors <ORBvoc.txt> <image list> <output>\n";
        return 1;
    }
    mcslam::OrbVocabulary vocabulary;
    if (!vocabulary.loadFromTextFile(argv[1])) {
        std::cerr << "cannot load vocabulary " << argv[1] << "\n";
        return 1;
    }
    // params/euroc/frontend.yaml.in: nFeatures, scaleFactor, nLevels, iniThFAST, minThFAST
    ORBextractor extractor(1200, 1.2f, 8, 20, 7);

    std::ifstream list(argv[2]);
    std::ofstream out(argv[3]);
    std::string path;
    size_t n = 0;
    while (std::getline(list, path)) {
        if (path.empty())
            continue;
        const cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
        if (img.empty()) {
            std::cerr << "cannot read " << path << "\n";
            return 1;
        }
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        std::vector<int> lapping = {0, 0};
        extractor(img, cv::noArray(), kps, desc, lapping);
        std::vector<cv::Mat> rows;
        for (int r = 0; r < desc.rows; ++r)
            rows.push_back(desc.row(r));
        DBoW2::BowVector bow;
        vocabulary.transform(rows, bow);
        out << bow.size();
        for (const auto& w : bow)
            out << ' ' << w.first << ':' << w.second;
        out << '\n';
        ++n;
    }
    std::cerr << "wrote " << n << " BoW vectors\n";
    return 0;
}
