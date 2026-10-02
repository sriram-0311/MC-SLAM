#include "MCSlam/ORBVocabulary.h"

#include <fstream>
#include <sstream>

#include <glog/logging.h>

namespace mcslam {

namespace {
constexpr int kOrbDescriptorBytes = 32;
}  // namespace

bool OrbVocabulary::loadFromTextFile(const std::string& filename) {
    std::ifstream in(filename);
    if (!in) {
        LOG(ERROR) << "Cannot open vocabulary file " << filename;
        return false;
    }

    int k = 0, L = 0, scoring = 0, weighting = 0;
    {
        std::string header;
        std::getline(in, header);
        std::istringstream hs(header);
        if (!(hs >> k >> L >> scoring >> weighting) || k < 2 || k > 20 || L < 1 || L > 10 ||
            scoring < 0 || scoring > 5 || weighting < 0 || weighting > 3) {
            LOG(ERROR) << "Malformed vocabulary header in " << filename << ": '" << header << "'";
            return false;
        }
    }

    std::vector<Node> nodes;
    std::vector<DBoW2::NodeId> leaves;
    nodes.emplace_back(0);  // root

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream ls(line);
        int parent = -1, is_leaf = 0;
        if (!(ls >> parent >> is_leaf) || parent < 0 || parent >= static_cast<int>(nodes.size())) {
            LOG(ERROR) << "Malformed vocabulary node " << nodes.size() << " in " << filename;
            return false;
        }

        const auto id = static_cast<DBoW2::NodeId>(nodes.size());
        Node node(id);
        node.parent = static_cast<DBoW2::NodeId>(parent);
        node.descriptor.create(1, kOrbDescriptorBytes, CV_8U);
        for (int i = 0; i < kOrbDescriptorBytes; ++i) {
            int byte = 0;
            if (!(ls >> byte) || byte < 0 || byte > 255) {
                LOG(ERROR) << "Malformed descriptor for vocabulary node " << id << " in " << filename;
                return false;
            }
            node.descriptor.at<uint8_t>(0, i) = static_cast<uint8_t>(byte);
        }
        if (!(ls >> node.weight)) {
            LOG(ERROR) << "Missing weight for vocabulary node " << id << " in " << filename;
            return false;
        }

        nodes[parent].children.push_back(id);
        if (is_leaf) leaves.push_back(id);
        nodes.push_back(std::move(node));
    }

    // Commit only after the whole file parsed, and take word pointers into the
    // final node storage (taking them while `nodes` grows would dangle).
    m_k = k;
    m_L = L;
    m_scoring = static_cast<DBoW2::ScoringType>(scoring);
    m_weighting = static_cast<DBoW2::WeightingType>(weighting);
    m_nodes = std::move(nodes);
    m_words.resize(leaves.size());
    for (size_t w = 0; w < leaves.size(); ++w) {
        Node& leaf = m_nodes[leaves[w]];
        leaf.word_id = static_cast<DBoW2::WordId>(w);
        m_words[w] = &leaf;
    }
    createScoringObject();

    LOG(INFO) << "Loaded ORB vocabulary " << filename << ": k=" << m_k << " L=" << m_L
              << " nodes=" << m_nodes.size() << " words=" << m_words.size();
    return true;
}

}  // namespace mcslam
