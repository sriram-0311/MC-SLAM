# Release Notes for TRI-SLAM
## Version 1.0.0
- Stable working version of GVIO with online smart factors and Incremental Smoothing and Mapping (ISAM2).
- Stable working version of Fast Tracking with cross-camera relocalization and tracking demonstrated.
- Initial version of Evaluation folder added to the repo with sufficient configuration files to conduct basic qualitative evaluation. Quantitative Evaluation to be added.

### Latest Incremental Changes
- Improved SLAM performance across all datasets with addition of FBOW.
- Reduced the occurrence of ILS during loop closures and in general during VIO SLAM.

### Known Issues
- Rare random occurence of ILS during SLAM and loop closures.