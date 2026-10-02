#pragma once

#include <string>

#include "FORB.h"
#include "TemplatedDatabase.h"
#include "TemplatedVocabulary.h"
#include "fbow/fbow.h"

namespace mcslam {

using OrbVocabularyBase = DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>;

/**
 * DBoW2 ORB vocabulary that can also be loaded from the plain-text tree
 * format popularised by ORB-SLAM ("ORBvoc.txt").
 *
 * Text format:
 *   line 1:          k L scoring weighting
 *   one line / node: parent_id is_leaf d_0 ... d_31 weight
 * Nodes are listed so that a parent always precedes its children; node ids
 * are implicit (1, 2, ... in file order; 0 is the root).
 */
class OrbVocabulary : public OrbVocabularyBase {
public:
    using OrbVocabularyBase::OrbVocabularyBase;

    /// Returns false (and leaves the vocabulary empty) if the file is missing or malformed.
    bool loadFromTextFile(const std::string& filename);
};

}  // namespace mcslam

namespace fbow {
using ORBVocabularyFBow = Vocabulary;
}

namespace DBoW2 {
using ORBVocabulary = mcslam::OrbVocabulary;
using ORBDatabase = TemplatedDatabase<FORB::TDescriptor, FORB>;
}  // namespace DBoW2
