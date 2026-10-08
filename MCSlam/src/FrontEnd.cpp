//
// Created by Pushyami Kaveti on 6/6/20.
//
#include "MCSlam/FrontEnd.h"
#include "MCSlam/LoopCloser.h"
#include <DUtils/DUtils.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//sac
#include <opengv/sac/Ransac.hpp>
#include <opengv/sac_problems/point_cloud/PointCloudSacProblem.hpp>

//opencv
#include "MCSlam/Triangulation.h"



using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace boost::assign;
//using namespace opengv;

// todo: Normalized coordinates for keypoints
// todo: Intramatch class instead of independent vectors
// todo: Delete images from memory, just save the intramatches, Map


void FrontEnd::createFrame(vector<Mat> img_set, vector<Mat> segmap_set, double timeStamp){
    currentFrame = new MultiCameraFrame(img_set, segmap_set, orb_vocabulary, orBextractor, orBextractors, camconfig_, current_frameId++,
                                        timeStamp, clahe, false);
//    std::unique_ptr<MultiCameraFrame> currentFrame(new MultiCameraFrame(img_set, segmap_set, orb_vocabulary, orBextractor, orBextractors, camconfig_, current_frameId++,
//                                                                        timeStamp, clahe, false));
    ///compute overlapping regions
    if(current_frameId == 1)
        compute_overlap();

    //lfFrames.push_back(frame);
}

void FrontEnd::compute_overlap() {
    int cell_size_x = 40;
    int cell_size_y = 30;
    int grid_rows = camconfig_.im_size_.height / cell_size_y;
    int grid_cols = camconfig_.im_size_.width / cell_size_x;
    int numBlocks_ = grid_rows * grid_cols;

    std::vector<Point2f> imgGrid[grid_cols][grid_rows];
    //int imgGrid[grid_cols][grid_rows];

    overlap_masks.reserve(5);

    // for each image visualize the overlapping regions
    // and non-overlapping regions
    int total_non_overlap_cells=0;

    for (int c1 = 0; c1 < camconfig_.num_cams_; c1++) {
        Mat overlap_grid = Mat::zeros(camconfig_.im_size_.height, camconfig_.im_size_.width, CV_64FC1);
        Mat overlap_grid2 = Mat::zeros(grid_rows, grid_cols, CV_64FC1);

        for(int c2=0; c2 <camconfig_.num_cams_; c2++){
            if(c1 == c2)
                continue;

            Mat T_j0 = Mat::eye(4, 4, CV_64F);
            Mat T_i0 = Mat::eye(4, 4, CV_64F);
            camconfig_.R_mats_[c1].copyTo(T_i0(cv::Range(0, 3), cv::Range(0, 3)));
            camconfig_.t_mats_[c1].copyTo(T_i0(cv::Range(0, 3), cv::Range(3, 4)));
            camconfig_.R_mats_[c2].copyTo(T_j0(cv::Range(0, 3), cv::Range(0, 3)));
            camconfig_.t_mats_[c2].copyTo(T_j0(cv::Range(0, 3), cv::Range(3, 4)));

            Mat Tji = T_j0 * T_i0.inv();
            Mat t_ji_x =  (Mat_<double>(3,3) << 0, -Tji.at<double>(2,3), Tji.at<double>(1,3),
                    Tji.at<double>(2,3), 0, -Tji.at<double>(0,3),
                    -Tji.at<double>(1,3), Tji.at<double>(0,3), 0);
            Mat F = camconfig_.K_mats_[c2].t().inv() * t_ji_x * Tji(cv::Range(0, 3), cv::Range(0, 3)) * camconfig_.K_mats_[c1].inv();
            double dmin = 1.0;
            double dmax = 40.0;

            ///////Visualization
            // draw epiolar lines
            Mat outImg = Mat::zeros(camconfig_.im_size_.height, camconfig_.im_size_.width * 2, CV_8UC3);;
            //outImg.create(camconfig_.im_size_.height, camconfig_.im_size_.width * 2, CV_8UC1);
            //currentFrame->imgs[c].copyTo(outImg(cv::Range(0, camconfig_.im_size_.height), cv::Range(0, camconfig_.im_size_.width)));
            //currentFrame->imgs[c2].copyTo(outImg(cv::Range(0, camconfig_.im_size_.height), cv::Range(camconfig_.im_size_.width, camconfig_.im_size_.width * 2)));
            //cvtColor(outImg,outImg , COLOR_GRAY2BGR);
            ///////////////////
            for (unsigned int i = 0; i < grid_cols; i++){

                for (unsigned int j = 0; j < grid_rows; j++) {

                    double grid_center_x = cell_size_x * i + (cell_size_x/2.0);
                    double grid_center_y = cell_size_y * j + (cell_size_y/2.0);
                    //unproject this point
                    double unproj_x = ( grid_center_x -  camconfig_.K_mats_[c1].at<double>(0, 2)) /
                               camconfig_.K_mats_[c1].at<double>(0, 0);
                    double unproj_y = ( grid_center_y -  camconfig_.K_mats_[c1].at<double>(1, 2)) /
                               camconfig_.K_mats_[c1].at<double>(1, 1);
                    //convert into c2 frame
                    Mat pt3d_sfm (4,1, CV_64F);
                    pt3d_sfm.at<double>(0,0) = unproj_x * dmin;
                    pt3d_sfm.at<double>(1,0) = unproj_y * dmin;
                    pt3d_sfm.at<double>(2,0) = dmin;
                    pt3d_sfm.at<double>(3,0) = 1.0;
                    //project into c2
                    Mat p3d_conv = Tji.rowRange(0,3) * pt3d_sfm;
                    Mat projected = camconfig_.K_mats_[c2] * p3d_conv;
                    double expected_x_min = projected.at<double>(0,0) / projected.at<double>(2,0);
                    double expected_y_min = projected.at<double>(1,0) / projected.at<double>(2,0);

                    //see if it lies within bounds
                    if(expected_x_min > 10 and expected_x_min < camconfig_.im_size_.width and expected_y_min < camconfig_.im_size_.height and expected_y_min > 10){
                        overlap_grid.rowRange(cell_size_y * j , (j+1) * cell_size_y ).colRange(cell_size_x * i, (i+1) * cell_size_x ) = overlap_grid.rowRange(cell_size_y * j , (j+1) * cell_size_y ).colRange(cell_size_x * i,(i+1) * cell_size_x ) + 1;

                        overlap_grid2.at<double>(j,i) = overlap_grid2.at<double>(j,i) + 1;
                    }


                    //imgGrid[i][j].reserve(5);
                    //find the grids in c2 that overlap with c1.
                    // Epipolar line in image c2 = F * kp1
                    float a = grid_center_x * F.at<double>(0,0)+grid_center_y*F.at<double>(0,1)+F.at<double>(0,2);
                    float b = grid_center_x * F.at<double>(1,0)+grid_center_y*F.at<double>(1,1)+F.at<double>(1,2);
                    float c = grid_center_x * F.at<double>(2,0)+grid_center_y*F.at<double>(2,1)+F.at<double>(2,2);

                    float den = a*a+b*b;
                    den = den ? 1./std::sqrt(den) : 1.;
                    a *= den; b *= den; c *= den;
                    den = a*a + b*b;

                    cv::Scalar color_(
                            (double)std::rand() / RAND_MAX * 255,
                            (double)std::rand() / RAND_MAX * 255,
                            (double)std::rand() / RAND_MAX * 255
                    );
                    //check for each grid_cell_x , where the y is, for coordinate (x,y) see if it is within bounds and what grid cell it belongs to, save the grid cells
                    for(int c2_grid_cell_x=0; c2_grid_cell_x < grid_cols; c2_grid_cell_x++){
                        // for each grid cell column
                        double c2_grid_center_x = cell_size_x * c2_grid_cell_x + (cell_size_x/2.0);
                        double c2_grid_center_y = -(c + a * c2_grid_center_x)/b;

                        int c2_grid_cell_y = floor(c2_grid_center_y / cell_size_y);
                        double c2_actual_grid_center_y = cell_size_y * c2_grid_cell_y + (cell_size_y/2.0);

//                        if(c2_grid_center_y < camconfig_.im_size_.height and c2_grid_center_y > 10){
//                            cv::circle(outImg,Point2f(c2_grid_center_x,c2_grid_center_y) + Point2f(camconfig_.im_size_.width , 0), 5, color_, 2);
//                            cv::circle(outImg,Point2f(c2_grid_center_x,c2_actual_grid_center_y) + Point2f(camconfig_.im_size_.width , 0), 5, Scalar(255,25,255), 1);
//                        }

                    }
                    //imgGrid[i][j].push_back(Point2f(grid_col_c, grid_row_c));

//                    // draw the epipolar line for grids in other camera
//                    int x0 = 0;
//                    int x1 = camconfig_.im_size_.width - 1;
//                    int y0 = -c/b;
//                    int y1 = -(c + a * x1)/b;
//                    cv::line(outImg,Point2f(x0,y0)+Point2f(camconfig_.im_size_.width , 0), Point2f(x1,y1)+Point2f(camconfig_.im_size_.width , 0),color_, 1);
//                    cv::circle(outImg,Point2f(grid_center_x,grid_center_y), 5, color_, 2);
//
//                    cv::imshow("epipolar line for camera:"+to_string(c1), outImg);
//                    cv::waitKey(0);

                }
            }

        }
        float no_overlap_cells = (grid_rows * grid_cols) - countNonZero(overlap_grid2);
        non_overlap_fraction.push_back(no_overlap_cells);
        total_non_overlap_cells += no_overlap_cells;

        overlap_grid = overlap_grid * 50;
        overlap_grid.convertTo(overlap_grid, CV_8U);

        overlap_masks.push_back(overlap_grid.clone());
        Mat overlap_grid_c, imgg;
        cvtColor(overlap_grid, overlap_grid_c, cv::COLOR_GRAY2BGR);

        cvtColor(currentFrame->imgs[c1], imgg, cv::COLOR_GRAY2BGR);
        overlap_grid_c = overlap_grid_c + imgg;
        Mat normed_img;
        cv::normalize(overlap_grid_c, normed_img, 0,255, cv::NORM_MINMAX);
       //cv::resize(overlap_grid, overlap_grid, camconfig_.im_size_);
       //cv::imshow("overlap for camera:"+to_string(c1), overlap_grid_c);
       //cv::waitKey(0);

    }
    for(int i =0; i <camconfig_.num_cams_;i++)
    {
       // cout<<"number of no overlap cells in cam"<<i<<" :"<<non_overlap_fraction.at(i)<<endl;
        non_overlap_fraction.at(i) =  non_overlap_fraction.at(i)/total_non_overlap_cells;
       // cout<<"fraction of no overlap cells in cam"<<i<<" :"<<non_overlap_fraction.at(i)<<endl;
    }

}

void FrontEnd::obtainLfFeatures(std::vector<IntraMatch>& matches_map, MultiCameraFrame* lf_frame,
                                std::vector<IntraMatch>& filtered_intra_matches,
                                vector<DBoW2::NodeId >& words_, set<DBoW2::NodeId >& filtered_words){

    std::vector<IntraMatch>::iterator matches_iter;
    int ind=0;
    set<DBoW2::NodeId > words_fil;
    vector<Mat> prjMats;
    vector<vector<bool>> keypoint_mask;
    std::map<int, int> intraMatchViews;
    for (int i = 0; i < lf_frame->num_cams_; ++i){
        keypoint_mask.emplace_back(lf_frame->image_kps_undist[i].size(), true);
        Mat P_11 = build_Rt(camconfig_.R_mats_[i], camconfig_.t_mats_[i]);
        prjMats.push_back(P_11);
        intraMatchViews[i+1] = 0;
    }

    std::vector<cv::Mat> vecDescs;

    vector<IntraMatch> mono_keypoints;
    vector<float> responses;
    vector<cv::Mat> mono_descs;
    float avg_depth_intrmatches = 0.0;

    lf_frame->intramatch_size =0;
    lf_frame->lIds.clear();
    lf_frame->mono_size = 0;
    currentFrame->intraMatches.clear();
    currentFrame->intraMatches.reserve(3000);
    //filtered_intra_matches.reserve(3000);
    vecDescs.reserve(3000);
    mono_keypoints.reserve(3000);
    responses.reserve(3000);
    mono_descs.reserve(3000);

    /// Iterate over intramatches (matches_map)
    for(matches_iter = matches_map.begin() ;  matches_iter!= matches_map.end(); ++matches_iter, ++ind ){
        IntraMatch temp = *matches_iter;
        DBoW2::NodeId word_cur = words_.at(ind);

        int first=-1, last=-1; // initialize this with ref camera, since that is the index which we will be avoiding
        vector<Mat> descs;
        vector<int> view_inds;
        int num_views=0;

        ///
        for(auto i = 0; i < temp.matchIndex.size(); ++i)
        {
            //print the intra matches so that we know.
            int feat_ind = temp.matchIndex[i];
            // if it is a valid index
            if(feat_ind != -1) {
                Point2f p = lf_frame->image_kps[i][feat_ind].pt;
                if (lf_frame->segMasks[i].at<float>(p.y, p.x) < 0.7) {

                    descs.push_back(lf_frame->image_descriptors[i][feat_ind]);
                    view_inds.push_back(i);
                    num_views++;
                } else {
                    temp.matchIndex[i] = -1;
                }
            }
        }
        //cout<<endl;
        //get the disparity of those points WRT to the biggest baseline it is visible in
        if(num_views>1) {
            //cout<<"Number of valid intra matches : "<<num_views<<endl;
            //create the DS to hold the 2D points corresponding to this intra match
            //VLOG(2)<<"Came here"<<endl;
            ///// Traingulation using  opencv sfm and normalized coordinates///////////
            vector<Mat> PJs;
            std::vector<Mat_<double> > xx;
            vector<int> octs;
            vector<Point2f> kps;
            for (int ii = 0; ii < num_views; ii++) {
                int cur_view_ind = view_inds[ii];
                //Mat Rt = build_Rt(camconfig_.R_mats_[cur_view_ind], camconfig_.t_mats_[cur_view_ind]);
               // Mat P1 = cv::Mat_<double>(Rt);
                PJs.push_back(prjMats[cur_view_ind]);
                Mat_<double> x1(2, 1);
                x1(0, 0) = (lf_frame->image_kps[cur_view_ind][temp.matchIndex[cur_view_ind]].pt.x -
                            camconfig_.K_mats_[cur_view_ind].at<double>(0, 2)) /
                           camconfig_.K_mats_[cur_view_ind].at<double>(0, 0);
                x1(1, 0) = (lf_frame->image_kps[cur_view_ind][temp.matchIndex[cur_view_ind]].pt.y -
                            camconfig_.K_mats_[cur_view_ind].at<double>(1, 2)) /
                           camconfig_.K_mats_[cur_view_ind].at<double>(1, 1);
                octs.push_back(lf_frame->image_kps[cur_view_ind][temp.matchIndex[cur_view_ind]].octave);
                kps.push_back( lf_frame->image_kps_undist[cur_view_ind][temp.matchIndex[cur_view_ind]].pt);
                xx.push_back(x1.clone());
            }

            cv::Mat pt3d_sfm;
            Eigen::Vector3d pt;
            mcslam::geom::triangulatePoints(xx, PJs, pt3d_sfm);

            if(pt3d_sfm.at<double>(2, 0) < 40 and pt3d_sfm.at<double>(2, 0) > 0.5 ){

                ///////////////////////////CHECK THE TRIANGULATION/////////////////////////////////////////////////

                // check the reprojection errors of the points and the parallalx angle
                int jj=0;
                bool accept_intramatch = true;
//                if((currentFrame->frameId+1) % 100 ==0){   //// DO THIS ONLY During online calib
//                    for (auto P : PJs){
//                        int cam_ind = view_inds[jj];
//                        Mat p3d_conv = P(cv::Range(0, 3), cv::Range(0, 3)) * pt3d_sfm +  P(cv::Range(0, 3), cv::Range(3, 4));
//                        Mat projected = camconfig_.K_mats_[cam_ind] * p3d_conv;
//                        double expected_x = projected.at<double>(0,0) / projected.at<double>(2,0);
//                        double expected_y = projected.at<double>(1,0) / projected.at<double>(2,0);
//                        double err = (expected_x - kps[jj].x)*(expected_x - kps[jj].x)+(expected_y - kps[jj].y)*(expected_y - kps[jj].y);
//                        err = err*orBextractor->GetInverseScaleSigmaSquares()[ octs[jj]];
//                        jj++;
//                        if(err>5.991){
//                            accept_intramatch = false;
//                            // VLOG(2)<<"Bad Intramatch : ";
//                            break;
//                        }
//                    }
//                }

                if(accept_intramatch){
                    intraMatchViews[num_views]++;
                    Mat projected = camconfig_.K_mats_[0] * pt3d_sfm;
                    double expected_x = projected.at<double>(0, 0) / projected.at<double>(2, 0);
                    double expected_y = projected.at<double>(1, 0) / projected.at<double>(2, 0);
                   // double inv_depth = 1.0 / projected.at<double>(2, 0);
                    //from multi-cam elas. just for comparison
                   // double base_0 = -1 * prjMats[0].at<double>(0, 3);
                   // double f_0 = camconfig_.K_mats_[0].at<double>(0, 0);
                   // inv_depth = f_0 * base_0 / (double) pt3d_sfm.at<double>(2, 0);
                    //VLOG(3)<<"Intra Match Point: "<<pt3d_sfm.at<double>(0, 0)<<","<<pt3d_sfm.at<double>(1, 0)<<","<<pt3d_sfm.at<double>(2, 0);

                    words_fil.insert(word_cur);
                    lf_frame->lIds.push_back(-1);
                    //update the intraMatch descriptor
                    cv::Mat desc_out;
                    lf_frame->computeRepresentativeDesc(descs, desc_out);
                    temp.matchDesc = desc_out;
                    temp.point3D = pt3d_sfm;
                    temp.uv_ref = cv::Point2f(expected_x, expected_y);
                    temp.mono = false;
                    temp.n_rays = num_views;
                    //filtered_intra_matches.push_back(temp);
                    currentFrame->intraMatches.push_back(temp);
                    vecDescs.push_back(desc_out);
                    lf_frame->intramatch_size++;

                    avg_depth_intrmatches = avg_depth_intrmatches + pt3d_sfm.at<double>(2, 0);
                    for(auto a: view_inds){
                        keypoint_mask[a][temp.matchIndex[a]] = false;

                    }
                }

                //////////////////////////////////////////////////////////////////////////
            }

        }
        else if(num_views == 1){
            intraMatchViews[num_views]++;
            temp.matchDesc = descs[0];
            temp.uv_ref = lf_frame->image_kps_undist[view_inds[0]][temp.matchIndex[view_inds[0]]].pt;
            temp.n_rays = num_views;
            temp.mono  = true;
//            lf_frame->lIds.push_back(-1);
            /// We are not adding this feature to intramatches or lids yet. we store them in temp vectors
            /// for monofeatures and then pick based on the distribution strategy along with the rest of the mono features.
            mono_keypoints.push_back(temp);
            mono_descs.push_back(descs[0]);
            responses.push_back(lf_frame->image_kps_undist[view_inds[0]][temp.matchIndex[view_inds[0]]].response);
            keypoint_mask[view_inds[0]][temp.matchIndex[view_inds[0]]] = false;
        }

    }
    // VLOG(1)<<"Average Depth of the Intra Match features: "<<avg_depth_intrmatches / lf_frame->intramatch_size;
    VLOG(1)<<"Number of Intra Match features : "<<lf_frame->intramatch_size;
    // for(int i=0 ; i < lf_frame->num_cams_ ; i++)
    //     VLOG(1)<<"num of intra matches with "<<(i+1)<<" views: "<<intraMatchViews[i+1];
   //if((currentFrame->frameId+1) % 100 ==0)
   //     bundleAdjustIntraMatches(currentFrame->intraMatches);
    //     bundleAdjustIntraMatches(filtered_intra_matches);
    ///Tiling of  features for good distribution
    int total_feats = 3000;
    int num_mono_feats = total_feats - lf_frame->intramatch_size;
    int cells_hor = 4; //changed this for kitti
    int cells_vert = 4; //changed this for kitti
    int feats_per_cell = ceil(num_mono_feats /(cells_hor * cells_vert*5));
    bool tiling = false;
    bool refview= false;


//    assert (lf_frame->img_size.width % cells_hor == 0);
//    assert (lf_frame->img_size.height % cells_vert == 0);
    int cell_width = round(lf_frame->img_size.width / cells_hor);
    int cell_height = round(lf_frame->img_size.height / cells_vert);
    if(num_mono_feats <= 0)
        tiling= false;
    if(tiling){

        int ind_to_mono = mono_keypoints.size();
        for(int i = 0; i < lf_frame->num_cams_; ++i) {

            std::vector<std::size_t> grid[cells_hor][cells_vert];
            std::vector<float> gridResponses[cells_hor][cells_vert];
            for(unsigned int i1=0; i1<cells_hor;i1++){
                for (unsigned int j1=0; j1<cells_vert;j1++){
                    grid[i1][j1].reserve(feats_per_cell);
                    gridResponses[i1][j1].reserve(feats_per_cell);
                }
            }
            for (int j = 0; j < lf_frame->image_kps_undist[i].size(); ++j) {
                Point2f p = lf_frame->image_kps[i][j].pt;
                if (keypoint_mask[i][j] && lf_frame->segMasks[i].at<float>(p.y, p.x) < 0.7) {
                    IntraMatch intra_match;
                    intra_match.matchIndex[i] = j;
                    intra_match.matchDesc = lf_frame->image_descriptors[i][j];
                    intra_match.uv_ref = lf_frame->image_kps_undist[i][j].pt;
                    intra_match.n_rays = 1;
                    mono_keypoints.push_back(intra_match);
                    responses.push_back(lf_frame->image_kps_undist[i][j].response);
                    mono_descs.push_back(lf_frame->image_descriptors[i][j]);

                    int nGridPosX, nGridPosY;
                    int grid_x = round((p.x - 30) / cell_width);
                    int grid_y = round((p.y - 30) / cell_height);
                    if (grid_x >= 0 and grid_x < cells_hor and grid_y >= 0 and grid_y < cells_vert) {
                        grid[grid_x][grid_y].push_back(ind_to_mono);
                        gridResponses[grid_x][grid_y].push_back(lf_frame->image_kps_undist[i][j].response);
                    }
                    ind_to_mono++;
                }
            }
            //cout<<"number of mono keypoints upto view "<<i<<" : "<<ind_to_mono<<endl;
            // select the top feats_per_cell number of kps in each grid cell
            int selected=0;
            for (int i1 = 0; i1 < cells_hor; i1++) {
                for (int j1 = 0; j1 < cells_vert; j1++) {
                    vector<size_t> kp_inds = grid[i1][j1];
                    vector<int> sorted_kp_ind = argsorte(gridResponses[i1][j1], false);
                    for (int k = 0; k < sorted_kp_ind.size() && k < feats_per_cell; ++k) {
                        int mono_kp_ind = kp_inds[sorted_kp_ind[k]];
                        currentFrame->intraMatches.push_back(mono_keypoints[mono_kp_ind]);
                        //filtered_intra_matches.push_back(mono_keypoints[mono_kp_ind]);
                        vecDescs.push_back(mono_descs[mono_kp_ind]);
                        lf_frame->lIds.push_back(-1);
                        lf_frame->mono_size++;
                        selected++;
                    }
                }
            }
            //cout<<"Number of Kps selected in view "<<i<<": "<<selected<<endl;
        }
    }
    else if (refview){
        for(int j = 0; j < lf_frame->image_kps_undist[0].size(); ++j){
            Point2f p = lf_frame->image_kps_undist[0][j].pt;
            if(keypoint_mask[0][j] && lf_frame->segMasks[0].at<float>(p.y, p.x) < 0.7){
                IntraMatch intra_match;
                intra_match.matchIndex[0] = j;
                intra_match.matchDesc = lf_frame->image_descriptors[0][j];
                intra_match.uv_ref = lf_frame->image_kps_undist[0][j].pt;
                intra_match.n_rays = 1;
                //mono_keypoints.push_back(intra_match);
                currentFrame->intraMatches.push_back(intra_match);
                //filtered_intra_matches.push_back(intra_match);
                vecDescs.push_back(lf_frame->image_descriptors[0][j]);
                lf_frame->lIds.push_back(-1);
                lf_frame->mono_size++;
            }
        }
    }
    else{
        int kp_mask_cnt=0;
        for(int i = 0; i < lf_frame->num_cams_; ++i){
            VLOG(2)<<"Number of keypoints in cam "<<i<<": "<<lf_frame->image_kps_undist[i].size();
            for(int j = 0; j < lf_frame->image_kps_undist[i].size(); ++j){
                Point2f p = lf_frame->image_kps_undist[i][j].pt;
                if(keypoint_mask[i][j]){// && lf_frame->segMasks[i].at<float>(p.y, p.x) < 0.7){
                    uint8_t ov_val = overlap_masks[i].at<uint8_t>(p);
                    //if(overlap_masks[i].at<uint8_t>(p)==0){
                        IntraMatch intra_match;
                        intra_match.matchIndex[i] = j;
                        intra_match.matchDesc = lf_frame->image_descriptors[i][j];
                        intra_match.uv_ref = lf_frame->image_kps_undist[i][j].pt;
                        intra_match.n_rays = 1;
                        intra_match.mono = true;
                        mono_keypoints.push_back(intra_match);
                        responses.push_back(lf_frame->image_kps_undist[i][j].response);
                        mono_descs.push_back(lf_frame->image_descriptors[i][j]);
                        keypoint_mask[i][j] = false;
                   // }
                }
                else{
                    kp_mask_cnt++;

                }
            }
        }
        VLOG(2)<<"KP mask cnt: "<<kp_mask_cnt;
        VLOG(2)<<"Overall number of mono features across all camera:" << mono_keypoints.size() << endl;
        VLOG(2)<<"Overall number of intramatches so far :" << lf_frame->intramatch_size << endl;
        vector<int> sorted_kp_ind = argsorte(responses, false);
        for(int i = 0; i < sorted_kp_ind.size() && i < (3000 - lf_frame->intramatch_size); ++i){
            //filtered_intra_matches.push_back(mono_keypoints[sorted_kp_ind[i]]);
            currentFrame->intraMatches.push_back(mono_keypoints[sorted_kp_ind[i]]);
            vecDescs.push_back(mono_descs[sorted_kp_ind[i]]);
            lf_frame->lIds.push_back(-1);
            lf_frame->mono_size++;
        }
        VLOG(2)<<"Overall number of mono features after response baed filtering:"<<mono_keypoints.size()<<endl;
    }
//    VLOG(0) << "Descriptor of image - " << currentFrame->image_descriptors[0][0] << endl;
    orb_vocabulary->transform(vecDescs, lf_frame->lfBoW, lf_frame->lfFeatVec, 4);

    ///////draw the intramatches
//    Mat all;
//    all.create(lf_frame->imgs[0].rows, lf_frame->imgs[0].cols * lf_frame->num_cams_, CV_8UC3);
//    for(int i=0; i < camconfig_.num_cams_ ; i++){
//        Mat imgBGR;
//        cvtColor(lf_frame->imgs[i],imgBGR , COLOR_GRAY2BGR);
//        imgBGR.copyTo(all.colRange(camconfig_.im_size_.width*i, camconfig_.im_size_.width*(i+1)));
//
//    }
//    //Show the support matches
//    cv::Mat dMap(Size(1,lf_frame->intramatch_size),CV_8UC1);
//    int ind_Dmap=0;
//    //for(IntraMatch im : filtered_intra_matches)
//    for(IntraMatch im : currentFrame->intraMatches){
//        if(!im.mono) {
//            dMap.at<uint8_t>(ind_Dmap, 0) = (uint8_t) im.point3D.at<double>(2, 0);
//            ind_Dmap++;
//        }
//
//    }
//
//    ///colors  mono-white, 2 views - r, 3 views -g, 4-views-b, 5-views - mix
//    Mat out;
//    Mat img_color;
//    cv::normalize(dMap, dMap, 0, 220, cv::NORM_MINMAX, CV_8U);
//    cv::applyColorMap(dMap, out, cv::COLORMAP_RAINBOW);
//    ind_Dmap=0;
//    for(IntraMatch im : currentFrame->intraMatches){
//        if(!im.mono){
//             Vec3b col1 = out.at<Vec3b>(ind_Dmap,0);
//             //cv::circle(all, im.uv_ref , 3, Scalar(col1[0],col1[1],col1[2]), 2,8,0 );
//             Scalar ccc;
//             if(im.n_rays == 2)
//                 ccc = Scalar(0,0,255);
//             else if(im.n_rays == 3)
//                 ccc = Scalar(0,255,0);
//             else if(im.n_rays == 4)
//                 ccc = Scalar(255,0,0);
//             else if(im.n_rays == 5)
//                 ccc = Scalar(0,0,255);
//             else
//                 cout<<"invalid number of rays:"<<im.n_rays<<endl;
//
//            for (int i = 0; i < im.matchIndex.size(); ++i){
//                if(im.matchIndex[i] != -1)
//                    cv::circle(all, lf_frame->image_kps_undist[i][im.matchIndex[i]].pt + Point2f(i * lf_frame->imgs[0].cols , 0),  5, Scalar(col1[0],col1[1],col1[2]), 1 );
//            }
//
//            //cv::circle(all, im.uv_ref , 5, ccc, 2,8,0 );
//             ind_Dmap++;
//        }
//        else{
//            for (int i = 0; i < im.matchIndex.size(); ++i){
//                if(im.matchIndex[i] != -1)
//                    cv::circle(all, lf_frame->image_kps_undist[i][im.matchIndex[i]].pt + Point2f(i * lf_frame->imgs[0].cols , 0),  3, Scalar(255,255,255), 2 );
//            }
//        }
//    }
//
//    cv::resize(all, all, Size(lf_frame->imgs[0].cols * lf_frame->num_cams_/2 , lf_frame->imgs[0].rows/2));
//    imshow("img with keypoints ", all);
//    waitKey(5);


//    filtered_words = words_fil;

}

//void FrontEnd::BruteForceMatching(Mat img1, Mat img2, vector<Mat> descs1, vector<Mat> descs2, vector<KeyPoint> kps1, vector<KeyPoint> kps2){
//    vector<vector<DMatch>> matches_mono;
//    vector<DMatch> good_matches;
//    vector<KeyPoint> kp_pts1_mono, kp_pts2_mono;
//    Mat mask_mono;
//
//    Mat descs1_mono = Mat(descs1.size(), descs1[0].cols, CV_8U );
//    Mat descs2_mono = Mat(descs2.size(), descs2[0].cols, CV_8U );
//    int ind=0;
//    for (auto& d : descs1){
//        d.copyTo(descs1_mono.row(ind));
//        ind++;
//    }
//    ind=0;
//    for (auto& d : descs2){
//        d.copyTo(descs2_mono.row(ind));
//        ind++;
//    }
//
//    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
//    auto start_intramatch = high_resolution_clock::now();
//    matcher->knnMatch(descs1_mono, descs2_mono, matches_mono, 2);
//    auto stop_intramatch = high_resolution_clock::now();
//    auto duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
//    VLOG(1)<<"time taken for matching brute force "<<duration.count()<<endl;
//
//    for(auto &m : matches_mono){
//        if(m[0].distance < 0.7*m[1].distance) {
//            if(m[0].distance > 50)
//                continue;
//            KeyPoint p_prev = kps1[m[0].queryIdx];
//            KeyPoint p_cur = kps2[m[0].trainIdx];
//            //make sure that the points belong to static areas based on the segmasks
//           // if (currentFrame->segMasks[0].at<float>(p_prev.pt.y, p_prev.pt.x) < 0.7 and currentFrame->segMasks[1].at<float>(p_cur.pt.y, p_cur.pt.x) < 0.7){
//                kp_pts1_mono.push_back(p_prev);
//                kp_pts2_mono.push_back(p_cur);
//                good_matches.push_back(m[0]);
//           // }
//
//        }
//
//    }
//    Mat matches_img;
//    drawMatches( img1, kps1,img2, kps2, good_matches, matches_img, Scalar::all(-1),
//                 Scalar::all(-1) );
//    imshow("Good Matches:"+to_string(0)+"--"+to_string(1), matches_img );
//    waitKey(5);
//
//    cout<<"BRUTEFORCE Total Number of matches between cam0 and cam1: "<<good_matches.size()<<endl;
//}

void FrontEnd::processFrame() {
    auto start_intramatch = high_resolution_clock::now();
    // get the images and extract key points
    ///////////////////////////////////////
    ///extract the ORB  features
    //////////////////////////////////////
    VLOG(1) << " ------------NEW FRAME ----------- " << endl;

    currentFrame->extractFeaturesParallel();

    auto stop_intramatch = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
    VLOG(1) << "Time taken for extracting ORB features: " << duration.count() << endl;

    start_intramatch = high_resolution_clock::now();
    std::vector<IntraMatch> matches_map;
      vector<DBoW2::NodeId> words_;
      set<DBoW2::NodeId> filtered_words;
    currentFrame->computeIntraMatches(matches_map, words_);
    //currentFrame->computeIntraMatches(matches_map, false);
    words_ = vector<DBoW2::NodeId>(matches_map.size(), 1);
    // VLOG(1) << "Total Number of intra matches found before filtering :" << matches_map.size();
    stop_intramatch = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
    // VLOG(1) << "time taken for Matching All Cams BoW " << duration.count() << endl;

    // Project intra-matches on to reference camera and visualize the disparity prior
    // Find out all the matches which do not have a correspondence in the reference frame
    // get the rays going through
    std::vector<IntraMatch> filtered_intra_matches;
    assert(words_.size() == matches_map.size());

    start_intramatch = high_resolution_clock::now();
    obtainLfFeatures(matches_map, currentFrame, filtered_intra_matches, words_, filtered_words);
    stop_intramatch = high_resolution_clock::now();
    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
    VLOG(1) << "Time taken to filter and insert intra matches: " << duration.count() << endl;
    VLOG(1) << "Total number of LF features : " << currentFrame->intraMatches.size() << endl;

    if (relocal) {
        for (int i = 0; i < currentFrame->num_cams_; i++) {
            cv::Mat kps = cv::Mat::zeros(currentFrame->image_kps[i].size(), 2, CV_64F);
            VLOG(0) << "Generating KD tree for camera " << i << " with " <<
                    currentFrame->image_kps[i].size() << " keypoints" << endl;
            for (int j = 0; j < currentFrame->image_kps[i].size(); j++) {
                kps.at<double>(j, 0) = currentFrame->image_kps[i][j].pt.x;
                kps.at<double>(j, 1) = currentFrame->image_kps[i][j].pt.y;
            }
            // create generic index
            cv::flann::GenericIndex<cv::flann::L2<double>> *index;
            index = new cv::flann::GenericIndex<cv::flann::L2<double>>(kps, cvflann::KDTreeIndexParams(4));
            // create a kd tree and push it to the vector
            cv::flann::GenericIndex<cv::flann::L2<double>> *imageTree;
            imageTree = new cv::flann::GenericIndex<cv::flann::L2<double>>(*index);
            currentFrame->image_kps_kdtree.push_back(imageTree);
        }
        tracer->imgCols = currentFrame->imgs[0].cols;
        tracer->imgRows = currentFrame->imgs[0].rows;
    }

    ////////////////////// Matching indiidual cameras for debugging///////////////////

//    vector<unsigned int> ind1, ind2;set<DBoW2::NodeId> words1;
//    start_intramatch = high_resolution_clock::now();
//    currentFrame->BowMatching(0,1,ind1, ind2,words1 );
//    stop_intramatch = high_resolution_clock::now();
//    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
//    VLOG(1)<<"time taken for Matching single Cam BoW "<<duration.count()<<endl;
//    vector<DMatch> good_matches_bow;
//    for(int i =0; i <ind1.size(); i++){
//        if(abs(currentFrame->image_kps_undist[0][ind1[i]].pt.y - currentFrame->image_kps_undist[1][ind2[i]].pt.y) < 50){
//            DMatch m = DMatch(ind1[i], ind2[i], 0);
//            good_matches_bow.push_back(m);
//        }
//    }
//    cout<<"BOW Total Number of matches between cam0 and cam1: "<<good_matches_bow.size()<<endl;
//    Mat matches_img_bow;
//    drawMatches( currentFrame->imgs[0], currentFrame->image_kps_undist[0], currentFrame->imgs[1], currentFrame->image_kps_undist[1], good_matches_bow, matches_img_bow, Scalar::all(-1),
//                 Scalar::all(-1) );
//    imshow("Good Matches Bow:"+to_string(0)+"--"+to_string(1), matches_img_bow );
//    waitKey(5);
//
//    BruteForceMatching(currentFrame->imgs[0], currentFrame->imgs[1], currentFrame->image_descriptors[0], currentFrame->image_descriptors[1],
//                       currentFrame->image_kps_undist[0], currentFrame->image_kps_undist[1]);
//
//    if(lfFrames.size()>1){
//        MultiCameraFrame* prevLF = lfFrames.back();
//        BruteForceMatching(prevLF->imgs[0], currentFrame->imgs[0], prevLF->image_descriptors[0], currentFrame->image_descriptors[0],
//                           prevLF->image_kps_undist[0], currentFrame->image_kps_undist[0]);
//    }

    ////////////////////// Matching indiidual cameras for debugging///////////////////


////////////////////COMPARISON ////////////////////////////////////
//    start_intramatch = high_resolution_clock::now();
//    std::vector<IntraMatch> matches_map_2;
//    currentFrame->computeIntraMatches( matches_map_2, true);
//    words_ = vector<DBoW2::NodeId>(matches_map_2.size(), 1);
//    VLOG(2) << "Total Number of intra matches found before filtering :" << matches_map_2.size();
//    stop_intramatch = high_resolution_clock::now();
//    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
//    VLOG(2) << "time taken for Matching All Cams BoW " << duration.count() << endl;
//    std::vector<IntraMatch> filtered_intra_matches_2;
//    start_intramatch = high_resolution_clock::now();
//    obtainLfFeatures(matches_map_2, currentFrame, filtered_intra_matches_2, words_, filtered_words);
//    stop_intramatch = high_resolution_clock::now();
//
//    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
//    VLOG(2) << "time taken for filter and insert intra matches: " << duration.count() << endl;
//    VLOG(2) << "Total number of LF features : " << currentFrame->intraMatches.size() << endl;
//    ////todo keep track of num mono and num intra
//    /// visualize intra matches across pairs - todo: maybe extract into a function ?

//    start_intramatch = high_resolution_clock::now();
//    std::vector<IntraMatch> matches_map_3;
//    currentFrame->computeIntraMatches_chained( matches_map_3, false);
//    words_ = vector<DBoW2::NodeId>(matches_map_3.size(), 1);
//    VLOG(2) << "Total Number of intra matches found before filtering :" << matches_map_3.size();
//    stop_intramatch = high_resolution_clock::now();
//    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
//    VLOG(2) << "time taken for Matching All Cams BoW " << duration.count() << endl;
//    std::vector<IntraMatch> filtered_intra_matches_3;
//    start_intramatch = high_resolution_clock::now();
//    obtainLfFeatures(matches_map_3, currentFrame, filtered_intra_matches_3, words_, filtered_words);
//    stop_intramatch = high_resolution_clock::now();
//
//    duration = duration_cast<milliseconds>(stop_intramatch - start_intramatch);
//    VLOG(2) << "time taken for filter and insert intra matches: " << duration.count() << endl;
//    VLOG(2) << "Total number of LF features : " << currentFrame->intraMatches.size() << endl;
//    ////todo keep track of num mono and num intra
//    /// visualize intra matches across pairs - todo: maybe extract into a function ?




    // construct the keypoint matches
    std::vector<KeyPoint> keypoints1, keypoints2;
    vector<Point2f> kp_pts1, kp_pts2;
    vector<DMatch> good_matches;
    int matching_cam = 0;
    std::vector<IntraMatch>::iterator matches_iter;
   /* for (int matching_cam2 = matching_cam + 1; matching_cam2 < currentFrame->num_cams_; matching_cam2++) {
        int ind = 0;
        for (matches_iter = filtered_intra_matches.begin();
             matches_iter != filtered_intra_matches.end(); ++matches_iter) {
            array<int, 5> temp = matches_iter->matchIndex;
            if (temp[matching_cam] != -1 and temp[matching_cam2] != -1) {
                keypoints1.push_back(currentFrame->image_kps[matching_cam][temp[matching_cam]]);
                keypoints2.push_back(currentFrame->image_kps[matching_cam2][temp[matching_cam2]]);
                kp_pts1.push_back(currentFrame->image_kps[matching_cam][temp[matching_cam]].pt);
                kp_pts2.push_back(currentFrame->image_kps[matching_cam2][temp[matching_cam2]].pt);
                good_matches.push_back(DMatch(ind, ind, 0));
                ind++;
            }
        }
        Mat img_matches_mono1;
        cv::drawMatches(currentFrame->imgs[matching_cam], keypoints1, currentFrame->imgs[matching_cam2], keypoints2, good_matches, img_matches_mono1, Scalar::all(-1),
                        Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        ///draw the inatrmatch 2D points as well
        ////-- Show detected matches
        imshow("Inter Matches 1"+to_string(matching_cam)+to_string(matching_cam2), img_matches_mono1);
        waitKey(0);
        Mat inliers;
        cv::findFundamentalMat(kp_pts1, kp_pts2, cv::FM_RANSAC, 2, 0.99,inliers);

        Mat img_matches_mono;
        cv::drawMatches(currentFrame->imgs[matching_cam], keypoints1, currentFrame->imgs[matching_cam2], keypoints2, good_matches, img_matches_mono, Scalar::all(-1),
                        Scalar::all(-1),inliers, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        ///draw the inatrmatch 2D points as well
        ////-- Show detected matches
        imshow("Inter Matches"+to_string(matching_cam)+to_string(matching_cam2), img_matches_mono);
        waitKey(0);

        Mat img_matches_mono2;
        cv::bitwise_not(inliers,inliers);
        cv::drawMatches(currentFrame->imgs[matching_cam], keypoints1, currentFrame->imgs[matching_cam2], keypoints2, good_matches, img_matches_mono2, Scalar::all(-1),
                        Scalar::all(-1),inliers, DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

        ///draw the inatrmatch 2D points as well
        ////-- Show detected matches
        imshow("Inter Matches outliers"+to_string(matching_cam)+to_string(matching_cam2), img_matches_mono2);
        waitKey(0);
    } */
}


/// Method which returns the latest estimated pose
/// \return
cv::Mat FrontEnd::getPose(){
    unique_lock<mutex> lock(mMutexPose);
    return currentFramePose;
}
////////////// this is just for visualizing which poss estimation is better////////////
cv::Mat FrontEnd::getPose_gp3p(){
    unique_lock<mutex> lock(mMutexPose);
    return currentFramePose2;
}




/*cv::Mat FrontEnd::getPose(){
    unique_lock<mutex> lock(mMutexPose);
    cv::Mat pose;
    if(currentFramePose.isZero()) {
        pose = cv::Mat::eye(3, 4, CV_32FC1);
    }
    else
        cv::eigen2cv(currentFramePose, pose);
    return pose;
}*/

vector<cv::Mat> FrontEnd::getAllPoses(){
    unique_lock<mutex> lock(mMutexPose);
    return allPoses;
}

vector<cv::Mat> FrontEnd::getAllPosesRelocalization() {
    unique_lock<mutex> lock(mMutexPose);
    vector<cv::Mat> relocalizationPoses;
    for (int i = 0; i < allPoses.size(); i++) {
        relocalizationPoses.push_back(allPoses[i] * Tbc_cv);
    }
    return relocalizationPoses;
}

////////////??EXTRA for pose vcisualizatrion
// Absolute pose visualization for loop closure
vector<cv::Mat> FrontEnd::getAllPoses1(){
    unique_lock<mutex> lock(mMutexPose);
    return allPoses1;
}

// Relative Pose.
vector<cv::Mat> FrontEnd::getAllPoses_gp3p(){
    unique_lock<mutex> lock(mMutexPose);
    return allPoses2;
}

vector<cv::Mat> FrontEnd::getAllPoses_dummy(){
    unique_lock<mutex> lock(mMutexPose);
//    VLOG(0) << " all poses dummy returned";
    return allPoses_dummy;
}

/// Method which clears all the vectors after
/// processing at time step t
void FrontEnd::reset() {
    //clear the data structures which are no longer needed to store
    if(lfFrames.size() > 3){
        //cleanup unncessary data before inserting
        int ref_cam=0;
        MultiCameraFrame* f = lfFrames[lfFrames.size() - 3 - 1];

        f->imgs.clear();
        f->imgs_original.clear();
        f->depthMap.deallocate();
        f->segMasks.clear();
        f->image_descriptors.clear();
        f->image_kps.clear();
        f->image_kps_kdtree.clear();
    }
    //currentFrame->imgs.clear();
    //currentFrame->segMasks.clear();
    //currentFrame->image_descriptors.clear();
   // currentFrame->image_kps.clear();
   // currentFrame->image_kps_undist.clear();

}

// Use the saveVocab with new datasets.
void FrontEnd::saveORBDatabase() const
{
    looper->saveDatabase(dbFile);
    // Save the database.
    serializeJSONObject();
}

void FrontEnd::writeTrajectoryToFile(const string &filename, bool include_dummy)
{
    VLOG(0) << "Saving camera trajectory to " << filename << " ..." << endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;
    int ind =0;
    vector<Mat> posesTowrite;
    vector<double> times;
    if(include_dummy == false){
        posesTowrite = allPoses;
        times = poseTimeStamps;
    }
    else{

        VLOG(0) <<" all poses size "<< allPoses.size();
        VLOG(0) <<" all poses dummy size "<< allPoses_dummy.size();
        VLOG(0) <<" combined size "<< combined_poses.size();
        VLOG(0) <<" combined_tStamps size " << combined_tStamps.size();

        posesTowrite = combined_poses;
        times = combined_tStamps;
    }

    for(auto p : posesTowrite){
        Mat Rwc = p.colRange(0,3);
        Mat twc = p.colRange(3,4);
        //cv::Mat C = (cv::Mat_<double>(3,3) << 0, 1, 0, 0, 0, -1, -1, 0, 0); //above 25_08_2020
        //cv::Mat C = (cv::Mat_<double>(3,3) << -1, 0, 0, 0, 0, 1, 0, 1, 0);
        //Rwc = Rwc * C ;
        vector<float> q = RotMatToQuat(Rwc);
        // VLOG(2)<< setprecision(6) << times.at(ind) << " " <<  setprecision(9) << twc.at<double>(0) << " " << twc.at<double>(1) << " " << twc.at<double>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        f << setprecision(6) << times.at(ind) << " " <<  setprecision(9) << twc.at<double>(0) << " " << twc.at<double>(1) << " " << twc.at<double>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        ind++;
    }
    f.close();
    VLOG(0) << "trajectory saved!" << endl;
}



void FrontEnd::insertKeyFrame(gtsam::NavState prevState_, gtsam::imuBias::ConstantBias prevBias_, double timestamp){

    VLOG(0) << " Inserting GPS KF at " << currentKFID;
    int currKFID = currentKFID;
    int prevKFID = lfFrames.back()->kfID;
//    VLOG(0) << "prevkfid = " << prevKFID;
//
    VLOG(0) << " prev state before this prediction " << prevState_;
    gtsam::NavState next_state = imu_integrator_comb->predict(prevState_, prevBias_);
//
    gtsam::Pose3 currKfPose = next_state.pose();
    VLOG(0) <<" Predicted pose for the keyframe (imu+gps)" << currKfPose;
//
    cv::Mat Pose = convertPose3_GTSAM2CV(currKfPose);

    currentFrameDummy = new MultiCameraFrame(true, timestamp, camconfig_);
    currentFrameDummy->kfID = currentKFID;
    currentFrameDummy->pose = Pose;
    currentFrameDummy->timeStamp = timestamp;

    cv::Mat PoseDummy_wc = currentFrameDummy->pose * Tbc_cv;


    assert(currentFrameDummy->dummy_kf == true);

    lfFrames.push_back(currentFrameDummy);
//    allPoses.push_back(PoseDummy_wc(cv::Range(0,3), cv::Range(0,4)).clone());
    allPoses_dummy.push_back(PoseDummy_wc(cv::Range(0,3), cv::Range(0,4)).clone());
    combined_poses.push_back(PoseDummy_wc(cv::Range(0,3), cv::Range(0,4)).clone());
    combined_tStamps.push_back(currentFrameDummy->timeStamp);


}

bool FrontEnd::checkGlobalRelocalization()
{
    // Caller function for relocalization check.
    bool relocalized = false;
    if (navability) {
        relocalized = relook->checkRelocalizationNavability(currentFrame);
    }
    else {
        relocalized = relook->checkRelocalization(currentFrame);
    }
    if(relocalized) {
        // set the pose of the current frame to be the pose found from relocalization
        gtsam::Pose3 w_T_c = relook->result.relative_pose;
        VLOG(0) << "Pose from global relocalization - \n" << w_T_c << std::endl;
//        // for debugging
//        currentFrame->pose = convertPose3_GTSAM2CV(w_T_c);
//        // insert the keyframe
//        insertKeyFrame();
        // convert Tbc_Relocalization to gtsam Pose3
        gtsam::Pose3 b_T_c = convertPose3_CV2GTSAM(Tbc_cv);
        // w_T_b = w_T_c * c_T_b
        gtsam::Pose3 w_T_b = w_T_c * b_T_c.inverse();
        // set the pose of the current frame to the relocalized pose
        currentFrame->pose = convertPose3_GTSAM2CV(w_T_b);
        VLOG(0) << "body pose after relocalization - \n" << w_T_b << std::endl;
//
        // set the camera rig pose to the relocalized pose
        tracer->cameraRig.c0_T_w = w_T_c.inverse();

        // insert the keyframe
        insertKeyFrame();

        // visualize the relocalized image and pose
        Mat tmpImg = currentFrame->imgs[0].clone();
//        cv::cvtColor(tmpImg, tmpImg, cv::COLOR_GRAY2BGR);
//        cv::Mat relocalizedPose = currentFrame->pose.clone();
//        cv::Mat relocalizedPose_wc = relocalizedPose * Tbc_cv;
//        cv::Mat relocalizedPose_wc_ = relocalizedPose_wc(cv::Range(0,3), cv::Range(0,4)).clone();
        cv::imshow("relocalized image", tmpImg);
        cv::waitKey(0);

        // temporary visualization
//        tracer->visualizeMatches(currentFrame);
    }
    return relocalized;
}

/* INCOMPLETE : This function is under development. This function should be used for tracking with only vision */
/* COMPLETE : This function is used for tracking with vision and IMU */
bool FrontEnd::startTrackingModule(gtsam::NavState state)
{
    cv::Mat result;
    cv::Mat dist;
//    gtsam::Pose3 relocalizationPoint = relook->result.relative_pose;
    gtsam::Pose3 w_T_bi = state.pose();
    VLOG(0) << "Pose of the current frame before refinement - \n" << w_T_bi << std::endl;
    // transform the pose from the car's base frame, which is IMU frame using the camera to IMU extrinsics
    // get the pose of the camera rig in the IMU frame convert Tbc_cv to gtsam Pose3
    gtsam::Pose3 b_T_c = convertPose3_CV2GTSAM(Tbc_cv);

    gtsam::Pose3 w_T_ci = w_T_bi * b_T_c;
    // Create a queryPoint of size 1 row, 3 cols
    cv::Mat queryPoint = cv::Mat::zeros(1, 3, CV_64F);
    queryPoint.at<double>(0, 0) = w_T_ci.translation().x();
    queryPoint.at<double>(0, 1) = w_T_ci.translation().y();
    queryPoint.at<double>(0, 2) = w_T_ci.translation().z();
    assert(queryPoint.size() == cv::Size(3, 1));

    auto start_T = high_resolution_clock::now();
    /// Get the match
    tracer->queryPoints(queryPoint, result, dist);
    // get all the poses from the json file using the getAllPoses() from tracer
    vector<Mat> allPosesFromMap = tracer->getAllPoses(Tbc_cv);
    // get the poses that were identified as neighbors from the json file
    vector<Mat> neighborPoses;
    for (int poseIt = 0; poseIt < result.cols; poseIt++) {
        int poseIndex = result.at<int>(0, poseIt);
        neighborPoses.push_back(allPosesFromMap[poseIndex]);
    }
    // set this variable as a member variable to visualize in openGLViewer
    neighborPoses_ = neighborPoses;
    auto stop_T = high_resolution_clock::now();
    auto duration_T = duration_cast<milliseconds >(stop_T - start_T);
    VLOG(0) << "Time taken for queryPoints - " << duration_T.count() << " m_s" << std::endl;
    std::map<int, cv::Mat> descriptors;
    std::unordered_map<int, cv::Mat> landmarksMap;
    auto start_T100 = high_resolution_clock::now();
    for (int i = 0; i < 5; i++) {
        tracer->processLandmarks(result.at<int>(0,i), landmarksMap, descriptors);
    }
//    tracer->retrieveLandmarks(result, descriptors, landmarksMap);
    auto stop_T100 = high_resolution_clock::now();
    auto duration_T100 = duration_cast<milliseconds >(stop_T100 - start_T100);
    VLOG(0) << "Time taken for retrieveLandmarks - " << duration_T100.count() << " m_s" << std::endl;
//    VLOG(0) << "Size of the landmarks map - " << landmarksMap.size() << std::endl;
    /// Set the tracer->cameraRig.POSE to the relocalizationPoint.
    gtsam::Pose3 ci_T_w = w_T_ci.inverse();
//    gtsam::Pose3 b_T_w = w_T_ci;
    tracer->cameraRig.c0_T_w = ci_T_w;
    VLOG(0) << "Pose of the camera rig - \n" << tracer->cameraRig.c0_T_w << std::endl;
    tracer->cameraRig.timestamp = currentFrame->timeStamp;

    // set the current frame pose to the relocalization point
    cv::Mat w_T_bi_cv = convertPose3_GTSAM2CV(w_T_bi);
    currentFrame->pose = w_T_bi_cv;

    /// Project these landmarks to get the keypoints.
    std::map<unsigned int, std::vector<cv::KeyPoint>> keypoints;

    std::map<unsigned int, std::vector<cv::Mat>> descriptorsMap;
    auto start_T2 = high_resolution_clock::now();
    std::map<int, std::vector<int>> projectedLandmarkIds;
    std::map<int, std::vector<cv::Mat>> projectedLandmarks = tracer->projectLandmarks(landmarksMap, keypoints,
                                                                                      descriptors, descriptorsMap,
                                                                                      projectedLandmarkIds);
    auto stop_T2 = high_resolution_clock::now();
    auto duration_T2 = duration_cast<milliseconds >(stop_T2 - start_T2);
    VLOG(0) << "Time taken for projectLandmakrs - " << duration_T2.count() << " m_s" << std::endl;

    // if number of successfully projected keypoints is too low, lost tracking. set globalRelocalization to false
    if(keypoints[0].size() < 10)
    {
        VLOG(0) << "Lost tracking. Relocalization failed." << std::endl;
        insertKeyFrame();
        return false;
    }
    else
    {
        VLOG(0) << "Relocalization successful. Number of keypoints - " << keypoints[0].size() << std::endl;
        std::map<int, std::vector<cv::KeyPoint>> bestMatches = tracer->queryCurrentFrame(keypoints, descriptors,
                                                                                         descriptorsMap, projectedLandmarks,
                                                                                         projectedLandmarkIds,
                                                                                         currentFrame, orBextractor);
        // store the bestMatches and projectedLandmarks in the member variables
        reprojectedKeypoints = bestMatches;
        reprojectedLandmarks = projectedLandmarks;
        reprojectedLandmarkIds = projectedLandmarkIds;

        // for debugging visualize the the best matches on the current frame in green circles and corresponding
        // projected landmarks in red circles for this image and exit upon esc key press
        cv::Mat allImagesTogether = cv::Mat::zeros(Size(currentFrame->num_cams_ * currentFrame->img_size.width,
                                                       currentFrame->img_size.height), CV_8UC3);
        float reprojectionError = 0;
        for (int camera =0; camera < currentFrame->num_cams_ ; camera++) {
            cv::Mat img = currentFrame->imgs[camera];
            // convert grayscale image to CV_8UC3
            cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
            for (int kps = 0; kps < reprojectedKeypoints[camera].size(); kps++) {
                cv::circle(img, reprojectedKeypoints[camera][kps].pt, 3, cv::Scalar(0, 255, 0), 2);
                gtsam::Point3 lm = gtsam::Point3(reprojectedLandmarks[camera][kps].at<double>(0),
                                                 reprojectedLandmarks[camera][kps].at<double>(1),
                                                 reprojectedLandmarks[camera][kps].at<double>(2));
                lm = tracer->cameraRig.c0_T_w.transformFrom(lm);
                std::vector<Point2, Eigen::aligned_allocator<Point2>> measurement = tracer->multi_cam_rig->project2(lm);
                cv::circle(img, cv::Point2f(measurement[camera].x(), measurement[camera].y()), 3, cv::Scalar(0, 0, 255), 2);
                // draw thin line between the reprojected landmark and the keypoint in the image
                cv::line(img, bestMatches[camera][kps].pt, cv::Point2f(measurement[camera].x(), measurement[camera].y()), cv::Scalar(0, 255, 0), 1);
                reprojectionError += sqrt(pow(measurement[camera].x() - bestMatches[camera][kps].pt.x, 2) + pow(measurement[camera].y() - bestMatches[camera][kps].pt.y, 2));
            }
            img.copyTo(allImagesTogether.rowRange(0, currentFrame->img_size.height).colRange(camera * currentFrame->img_size.width, (camera + 1) * currentFrame->img_size.width));
        }
        cv::resize(allImagesTogether, allImagesTogether, Size(currentFrame->num_cams_ * currentFrame->img_size.width / 2,
                                                             currentFrame->img_size.height / 2));
        cv::imshow("Best matches", allImagesTogether);
        cv::waitKey(3);
        insertKeyFrame();
    }
    return true;
}

void FrontEnd::refinePose() {
    // refine the current estimated pose using the reprojected keypoints and landmarks using GP3P
    //create a non-central absolute adapter
    bearingVectors_t bearingVectors;
    points_t points;
    std::vector<int> camCorrespondences;
    // fill the openGL vectors
    for (int i = 0; i < camconfig_.num_cams_; i++) {
        for (int j = 0; j < reprojectedLandmarks[i].size(); j++) {
            cv::Mat point = reprojectedLandmarks[i][j];
            Eigen::Vector3d point_eigen;
            for (int k = 0; k < 3; k++) {
                point_eigen(k) = point.at<double>(k);
            }
            points.push_back(point_eigen);
            Point2f measurement = reprojectedKeypoints[i][j].pt;
            double normX = (measurement.x - camconfig_.K_mats_[i].at<double>(0, 2)) / camconfig_.K_mats_[i].at<double>(0, 0);
            double normY = (measurement.y - camconfig_.K_mats_[i].at<double>(1, 2)) / camconfig_.K_mats_[i].at<double>(1, 1);
            Eigen::Vector3d pt_n;
            pt_n << normX, normY, 1;
            bearingVectors.push_back(pt_n/pt_n.norm());
            camCorrespondences.push_back(i);
        }
    }
    absolute_pose::NoncentralAbsoluteAdapter adapter(
            bearingVectors,
            camCorrespondences,
            points,
            currentFrame->translations,
            *currentFrame->rotations_ptr);
    //Create a AbsolutePoseSacProblem and Ransac
    //The method is set to GP3P
    sac::Ransac<
            sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
            new sac_problems::absolute_pose::AbsolutePoseSacProblem( adapter,
                                                                     sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/camconfig_.K_mats_[0].at<double>(0,2)));
    ransac.max_iterations_ = 100;

    vector<bool> inliers = vector<bool>(bearingVectors.size(), false);
    vector<bool> ransac_inliers = vector<bool>(bearingVectors.size(),false);

    //Run the experiment
    struct timeval tic;
    struct timeval toc;
    gettimeofday( &tic, 0 );
    // VLOG(1) << " RANSAC in  absolutePoseFromGP3P" << std::endl;
    ransac.computeModel();
    gettimeofday( &toc, 0 );
    double ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));

    for(size_t i = 0; i < ransac.inliers_.size(); i++){
        ransac_inliers[ransac.inliers_[i]] = true;
        inliers[ransac.inliers_[i]] = true;
    }

    transformation_t nonlinear_transformation = ransac.model_coefficients_;
    Mat curPos;
    cv::eigen2cv(nonlinear_transformation, curPos);
    cv::Mat T = Mat::eye(4, 4, CV_64F);
    curPos.copyTo(T(cv::Range(0, 3), cv::Range(0, 4)));
    VLOG(2)<<"Ransac Done";

    // count the number of inliers and outliers
    int numInliers = 0;
    int numOutliers = 0;
    for (int i = 0; i < inliers.size(); i++) {
        if (inliers[i]) {
            numInliers++;
        } else {
            numOutliers++;
        }
    }
    VLOG(0) << "Number of inliers - " << numInliers << std::endl;
    VLOG(0) << "Number of outliers - " << numOutliers << std::endl;
    VLOG(0) << "Ransac time - " << ransac_time << std::endl;
    VLOG(0) << "Pose after RANSAC - \n" << T << std::endl;

    // calculate the inliers to total ratio
    double inlierRatio = (double) numInliers / (double) (numInliers + numOutliers);
    VLOG(0) << "Inlier ratio - " << inlierRatio << std::endl;

    // set the current frame pose to the w_T_b = w_T_c * c_T_b
    if (inlierRatio > 0.2)
    {
        // print the pose after refinement
        currentFrame->pose = T * Tbc_cv.inv();
        Mat refinedPose = T * Tbc_cv.inv();
        // print the pose after refinement
        VLOG(0) << "Pose after refinement - \n" << refinedPose << std::endl;
    }
    // insert the current frame as a keyframe
    insertKeyFrame();
}

void  FrontEnd::initializeLoopClosure()
{
    // Create Loop closer objects changes the database to the new one.
//    looper = new LoopCloser(database_vocab, orBextractor);
    looper = new LoopCloser(orb_vocabulary, orBextractor);
    //TODO: Set from a config file. ----
    looper->K_mat = camconfig_.K_mats_;
    looper->dloop_param.image_cols = 720;   // Not used anymore.
    looper->dloop_param.image_rows = 5; //40;   // Not used anymore.
    looper->dloop_param.di_levels = 2;
    looper->dloop_param.k = 2;
    looper->dloop_param.alpha = 0.2;
    looper->dloop_param.min_Fpoints = 7;

    looper->setParams(looper->dloop_param);
    looper->type = LoopCloser::ABSOLUTE_POSE;
    result.status = DLoopDetector::NO_GROUPS;
}


void FrontEnd::insertKeyFrame(){


    VLOG(0)<<"INSERTING KEYFRAME at " << currentKFID  << " for image no. " << currentFrame->frameId << endl;
    VLOG(1)<< std::setprecision(15) << "time of keyframe insertion =  " <<currentFrame->timeStamp << endl;
    unique_lock<mutex> lock(mMutexPose);


    currentFrame->kfID = currentKFID;
    currentKFID++;
    currentFramePose = currentFrame->pose(cv::Range(0,3), cv::Range(0,4)).clone();

    VLOG(0) << " Keyframe pose is " << currentFramePose;
    lfFrames.push_back(currentFrame);
    allPoses.push_back(currentFrame->pose(cv::Range(0,3), cv::Range(0,4)).clone());
    combined_poses.push_back(currentFrame->pose(cv::Range(0,3), cv::Range(0,4)).clone());
    combined_tStamps.push_back(currentFrame->timeStamp);
    //allPoses1.push_back(currentFrame->pose(cv::Range(0,3), cv::Range(0,4)).clone());
    poseTimeStamps.push_back(currentFrame->timeStamp);
    ///insert also the sparse map for visualization
    vector<Point3f> points;
    vector<Point3f> colors;
    points.reserve(1000);

    if(initialized_ == INITIALIZED)
    {
        /// Loop Detection check:
        // looper->callerDetectLoop(currentFrame, map, result);
        // if(result.detection())
        if(relocal)
        {
//            if (relook->checkRelocalization(currentFrame)) {
//                VLOG(0) << "++++++++++++++++++++++++++++++++++++++++" << std::endl;
//                VLOG(0) << "Completed Relocalization!" << std::endl;
//                VLOG(0) << "++++++++++++++++++++++++++++++++++++++++" << std::endl;
//            }
        }
        else
        {
            /// Loop Detection check:
            VLOG(0) << "Calling loop detection" << std::endl;
            looper->callerDetectLoop(currentFrame, map, lfFrames, result);
            if(result.detection())
            {
                VLOG(0) << "++++++++++++++++++++++++++++++++++++++++" << std::endl;
                VLOG(0) << "Found Loop!" << std::endl;
                VLOG(0) << "Matched at - " << result.match << std::endl;
                VLOG(0) << "++++++++++++++++++++++++++++++++++++++++" << std::endl;
                currentFramePose2 = result.pose_to_check.first;
                allPoses1.push_back(result.pose_to_check.first(cv::Range(0,3), cv::Range(0,4)).clone());
                allPoses2.push_back(result.pose_to_check.second(cv::Range(0,3), cv::Range(0,4)).clone());
            }
        }

    }


  ///////////// Plot point from stereo reconstruction
//    for (int i = 0; i < currentFrame->depthMap.cols; i=i+12) {
//        for (int j = 0; j < currentFrame->depthMap.rows; j=j+12) {
//            Vec3f pt= currentFrame->depthMap.at<Vec3f>(j,i);
//            Vec3b cl= currentFrame->imgs_original[0].at<Vec3b>(j, i);
//            points.push_back(Point3f(pt[0], pt[1],pt[2]));
//            colors.push_back(Point3f(cl[2]/255.0, cl[1]/255.0, cl[0]/255.0));
//        }
//    }
    ///////////// Plot point from intramatch reconstruction
//    for (const IntraMatch& im : currentFrame->intraMatches){
//        if(!im.mono){
//            Mat l = currentFrame->pose.rowRange(0,3).colRange(0,3)*im.point3D + currentFrame->pose.rowRange(0,3).colRange(3,4);
//            points.emplace_back( l.at<double>(0,0) , l.at<double>(1,0) , l.at<double>(2,0));
//            float b = currentFrame->imgs_original[0].at<Vec3b>(im.uv_ref.x,im.uv_ref.y)[0]/255.0;
//            float g = currentFrame->imgs_original[0].at<Vec3b>(im.uv_ref.x,im.uv_ref.y)[1]/255.0;
//            float r = currentFrame->imgs_original[0].at<Vec3b>(im.uv_ref.x,im.uv_ref.y)[2]/255.0;
//            colors.emplace_back(r,g,b);
//        }
//    }
    sparse_recon.push_back(points);
    sparse_recon_color.push_back(colors);
}

void FrontEnd::deleteCurrentFrame(){
    delete currentFrame;
}

///INITIALIZATION AND POSE ESTIMATION and TRIANGULATE IN MONO///
///////////////////////////////////////////////////////////////

//////// TAKEN FROM ORBSLAM ///////////////////////////////////////
//void FrontEnd::triangulatePointTwoLFs(MultiCameraFrame* lf1, MultiCameraFrame* lf2, int featInd1, int featInd2, Mat pose1, Mat pose2,){
//}

void
FrontEnd::get3D_2DCorrs(MultiCameraFrame *lf, int featInd, Mat pose, vector<Mat> &projmats, vector<Mat> &PJs, std::vector<Mat_<double>> &xs,
                        vector<int> &view_inds, vector<Point2f> &kps, vector<int> &octaves) {
    IntraMatch m = lf->intraMatches[featInd];
    for (int i1 = 0 ; i1 < lf->num_cams_ ; i1++){
        int ind = m.matchIndex[i1];
        if (ind != -1){
            /// for the LF frame we need to build projection matrices in world frame
//            Mat ref_R_cur = camconfig_.R_mats_[i1].t();
//            Mat ref_t_cur = -ref_R_cur*camconfig_.t_mats_[i1];
//            Mat ref_T_cur = Mat::eye(4, 4, CV_64F);
//            ref_R_cur.copyTo(ref_T_cur(cv::Range(0, 3), cv::Range(0, 3)));
//            ref_t_cur.copyTo(ref_T_cur(cv::Range(0, 3), cv::Range(3, 4)));
//            Mat W_T_cur = pose * ref_T_cur;
//            Mat cur_T_W = W_T_cur.inv();
//            Mat P1 = cv::Mat_<double>(cur_T_W(cv::Range(0,3), cv::Range(0,4)));
            PJs.push_back(projmats[i1](cv::Range(0,3), cv::Range(0,4)));
            Mat_<double> x1(2, 1);
            x1(0, 0) = (lf->image_kps_undist[i1][ind].pt.x - camconfig_.K_mats_[i1].at<double>(0,2))/camconfig_.K_mats_[i1].at<double>(0,0);
            x1(1, 0) = (lf->image_kps_undist[i1][ind].pt.y - camconfig_.K_mats_[i1].at<double>(1,2)) / camconfig_.K_mats_[i1].at<double>(1,1);
            xs.push_back(x1.clone());
            view_inds.push_back(i1);
            kps.push_back(lf->image_kps_undist[i1][ind].pt);
            octaves.push_back(lf->image_kps_undist[i1][ind].octave);
        }
    }
}
///////////////////////////////////////////////////////////////
bool FrontEnd::initialization(){
    // if this is the first frame then insert it as a KF
    // todo: May be check if there are enough intra matches for no
    VLOG(0)<<"Initializing ......."<<endl;
    if (init_cond_ == MIN_FEATS){
        if(currentFrame->intraMatches.size() > 150){
           // currentFramePose = cv::Mat::eye(3, 4, CV_64F);

            currentFrame->pose = cv::Mat::eye(4, 4, CV_64F);
            insertKeyFrame();
            initialized_ = INITIALIZED;
            VLOG(2)<<"Initialization DONE"<<endl;
            return true;
        }
    } else {
        auto frame_test_id = currentFrame->kfID;

        if((lfFrames.size() == 0 or initialized_ == REINITIALIZING) and   initializationTrials == 0){

            // Adding IMU pose
            // We have Tbw and Tbc
            // Twc = inv(Tbw) * Tbc
            // T body to world:

            // as of now there is no dead reckoning. So, we are using the first
            // as of now there is no dead reckoning. So, we are using the first
            // imu message's orientation as the initial orientation
            initializationTrials++;
            if (useIMU) {
                Eigen::Matrix4d Tbw = Eigen::Matrix4d::Identity();
                if (initialized_ == REINITIALIZING) {
                    VLOG(0) << "start Reinitialization of vision" << endl;

                    // perform IMU  pre-integration upto this point to get the IMU pose wrt world
                    imu_preintegration(currentFrame->timeStamp);
                    gtsam::NavState cur_imu_state = imu_integrator_comb->predict(lastState_, lastBias_);
                    Tbw = cur_imu_state.pose().matrix().inverse();
                } else {
                    VLOG(0) << "start initialization at 0th frame" << endl;
                    Tbw.block<3, 3>(0, 0) = world_imu_rotation;
                }

                // T camera to world:
                Eigen::Matrix4d Twc = Tbw.inverse() * Tbc;
                cv::Mat cvMat(4, 4, CV_64F);
                cv::eigen2cv(Twc, cvMat);
                currentFrame->pose = cvMat;

                for (auto &im: currentFrame->intraMatches) {
                    if (!im.mono) {
                        Mat tmp_3d = cvMat(cv::Range(0, 3), cv::Range(0, 3)) * im.point3D +
                                     cvMat(cv::Range(0, 3), cv::Range(3, 4));
                        im.point3D = tmp_3d.clone();
                    }

                }

            } else {

                currentFrame->pose = cv::Mat::eye(4, 4, CV_64F);
            }

            insertKeyFrame();
            return false;
        }
        else{
//            assert(lfFrames.size() == 1);
            initializationTrials++;
            /// match the features between current frame and the prev frame
            MultiCameraFrame* lf_prev;
            for (int i = lfFrames.size() - 1; i >= 0; i --) {
                lf_prev = lfFrames[i];
                //the prev frame could be a dummy kf
                if(lf_prev->dummy_kf){
                    continue;
                }
                else{
                    break;
                }
            }


            /// Find inter matches, and get the rotation and translation from last keyframe
            std::vector<DMatch> inter_matches, mono_matches;
            opengv::points_t interMatch_3d_1, interMatch_3d_2;
            switch (interMatch_) {
                case BF_MATCH:
                    findInterMatches(lf_prev, currentFrame, inter_matches, mono_matches, false);
                    break;
                case BoW_MATCH:
                    findInterMatchesBow(lf_prev, currentFrame, inter_matches, mono_matches, false);
                    break;
            }
            VLOG(2) << "Intramatch Intermatches size: " << inter_matches.size() << endl;
            VLOG(2) << "Monomatch Intermatches size: " << mono_matches.size() << endl;
            // if there are min number of matches
            vector<bool> inliers;
            vector<bool> mono_inliers;
            vector<Mat> triangulatedPts = vector<Mat>(inter_matches.size() + mono_matches.size());
//            vector<Mat> triangulatedPtNormals = vector<Mat>(inter_matches.size() + mono_matches.size());
            Mat T = Mat::eye(4, 4, CV_64F);
            Mat T_17 = Mat::eye(4, 4, CV_64F);
            int num_mono_inliers = poseFromSeventeenPt(lf_prev, currentFrame, mono_matches, mono_inliers, T_17, false);

            if(inter_matches.size() > 50) {
                // estimate the pose between the two LF frames and
                //get the inlier features
                int num_inliers = estimatePoseLF(lf_prev, currentFrame, inter_matches, inliers, T, false);

            }
            else if(currentFrame->num_cams_ == 1){ //// with the current status, we canot run mono with IMU. We haveto fix it
                vector<Point2f> kp_pts1_mono, kp_pts2_mono;
                Mat mask_mono;
                cv::Mat rotation_t = Mat::zeros(3, 3, CV_32F), translation_t = Mat::zeros(3, 1, CV_32F);
                for(auto &m : mono_matches){
                    Point2f p_prev = lf_prev->intraMatches[m.queryIdx].uv_ref;
                    Point2f p_cur = currentFrame->intraMatches[m.trainIdx].uv_ref;
                    kp_pts1_mono.push_back(p_prev);
                    kp_pts2_mono.push_back(p_cur);
                }
                ///ESSENIAL MATRIX
                cv::Mat E_mat;
                //find the essential matrix
                E_mat = findEssentialMat(kp_pts1_mono, kp_pts2_mono , camconfig_.K_mats_[0],cv::RANSAC,0.97, 1.0, mask_mono);
                //recover pose
                recoverPose(E_mat, kp_pts1_mono, kp_pts2_mono, camconfig_.K_mats_[0], rotation_t, translation_t, mask_mono);
                //triangulate the points
                mono_inliers.clear();
                int i=0;
                for(auto &m : mono_matches) {
                    if (mask_mono.at<uchar>(i)){
                        mono_inliers.push_back(true);
                        //num_inliers++;
                    }
                    else
                        mono_inliers.push_back(false);
                    i++;
                }

                Mat rotation = rotation_t.t();
                Mat translation = -rotation_t.t() * translation_t;

                T = Mat::eye(4, 4, CV_32F);
                rotation.copyTo(T(cv::Range(0, 3), cv::Range(0, 3)));
                translation.copyTo(T(cv::Range(0, 3), cv::Range(3, 4)));
                T.convertTo(T, CV_64F);

            }
            else{
                T = T_17.clone();
            }

            if(norm(T.colRange(3,4).rowRange(0,3)) > kf_translation_threshold*0.5 ){
                //if the inliers are greater than a threshold (100)

                inter_matches.insert(inter_matches.end(), mono_matches.begin(), mono_matches.end());
                inliers.insert(inliers.end(), mono_inliers.begin(), mono_inliers.end());

                vector<Mat> ProjMats1, ProjMats2;
                for (int c_ind = 0; c_ind < currentFrame->num_cams_; c_ind++) {
                    Mat prev_T_ref = Mat::eye(4, 4, CV_64F);
                    camconfig_.R_mats_[c_ind].copyTo(prev_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
                    camconfig_.t_mats_[c_ind].copyTo(prev_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
                    // T_c_w  = T_c_b * T_b_w
                    Mat cur_T_W = prev_T_ref * lf_prev->pose.inv();
                    Mat P1 = cv::Mat_<double>(cur_T_W(cv::Range(0, 3), cv::Range(0, 4)));
                    ProjMats1.push_back(P1.clone());
                }
                for (int c_ind = 0; c_ind < currentFrame->num_cams_; c_ind++) {
                    Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
                    camconfig_.R_mats_[c_ind].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
                    camconfig_.t_mats_[c_ind].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
                    Mat cur_T_W = cur_T_ref * T.inv();
                    Mat P1 = cv::Mat_<double>(cur_T_W(cv::Range(0, 3), cv::Range(0, 4)));
                    ProjMats2.push_back(P1.clone());
                }


                int num_triangulated = 0;
                int num_initial_inliers = 0;
                vector<double> parllaxVec,depthVec;
                //Do a triangulation with non linear refinement of the inliers using both the LF frames
                int numt1=0, numt2=0, numt3=0;
                double avg_depth=0.0, median_scene_depth = 0.0;
                for (int i = 0 ; i < inter_matches.size(); i++){
                    /// if the current match is an outlier move on to next
                    if (!inliers[i])
                        continue;
                    num_initial_inliers++;
                    ///Get the intramatches and their corresponding 2D observations
                    DMatch m = inter_matches[i];
                    vector<Mat> PJs, centres;
                    vector<int> view_inds;
                    vector<Point2f> kps;
                    int num_views = 0;
                    std::vector<Mat_<double> > xs;
                    vector<int> kp_octaves;
                    Mat o1, o2;
                    //extract the 3D-2D correspondences in world coordintes
                    get3D_2DCorrs(lf_prev, m.queryIdx, Mat::eye(4, 4, CV_64F), ProjMats1, PJs, xs, view_inds, kps,
                                  kp_octaves);
                    o1 = -1 * PJs[0](cv::Range(0, 3), cv::Range(0, 3)).t() * PJs[0](cv::Range(0, 3), cv::Range(3, 4));
                    int nextI = PJs.size();
                    get3D_2DCorrs(currentFrame, m.trainIdx, T, ProjMats2, PJs, xs, view_inds, kps, kp_octaves);
                    o2 = -1 * PJs[nextI](cv::Range(0, 3), cv::Range(0, 3)).t() *
                         PJs[nextI](cv::Range(0, 3), cv::Range(3, 4));

                    //todo: Triangulation methods implementation
                    ///// Triangulation using  opencv sfm and normalized coordinates///////////
                    //triangulate and get the 3D point
                    cv::Mat pt3d;
                    mcslam::geom::triangulatePoints(xs, PJs, pt3d);

                    //TODO: check if this is correct - it is in the for loop below. but why?
                //    if(pt3d.at<double>(2,0) < 0){
                //        inliers[i] = false;
                //        continue;
                //    }

                    // check the reprojection errors of the points and the parallalx angle
                    int jj = 0;
                    for (auto P: PJs) {

                        int cam_ind = view_inds[jj];
                        //cout<<"landmark camrea view: "<<cam_ind<<endl;
                        // Convert the 3D point from world into camera frame. CHeck if it is behind the camera.
                        Mat p3d_conv = P(cv::Range(0, 3), cv::Range(0, 3)) * pt3d +  P(cv::Range(0, 3), cv::Range(3, 4));
                        if(p3d_conv.at<double>(2,0) < 0){
                            inliers[i] = false;
                            break;
                        }
                        Mat projected = camconfig_.K_mats_[cam_ind] * p3d_conv;
                        double expected_x = projected.at<double>(0, 0) / projected.at<double>(2, 0);
                        double expected_y = projected.at<double>(1, 0) / projected.at<double>(2, 0);
                        double err = (expected_x - kps[jj].x) * (expected_x - kps[jj].x) +
                                     (expected_y - kps[jj].y) * (expected_y - kps[jj].y);
                        err = err * orBextractor->GetInverseScaleSigmaSquares()[kp_octaves[jj]];
                        jj++;
                        if (err > 5.991) {
                            inliers[i] = false;
                            break;
                        }

                    }
                    if (inliers[i]) {
                        // Check parallax
                        cv::Mat normal1 = pt3d - o1;
                        double dist1 = cv::norm(normal1);

                        cv::Mat normal2 = pt3d - o2;
                        double dist2 = cv::norm(normal2);

                        double cosParallax = normal1.dot(normal2) / (dist1 * dist2);

                        VLOG(2) << "cosParallax: " << cosParallax;


                        parllaxVec.push_back(cosParallax);
                        if(cosParallax < 0.99998){
                            //Eigen::Vector3d pt;
                            //for(int j=0; j<3; ++j)
                            //    pt[j] = pt3d.at<double>(j,0);
                            triangulatedPts[i] = pt3d;
//                            //compute normal
//                            Mat normal = (normal1/dist1) + (normal2/dist2);
//                            triangulatedPtNormals[i] = normal/2.0;
                            num_triangulated++;
                            avg_depth += dist2;
                            depthVec.push_back(dist2);
                        }
                        else{
                            inliers[i] = false;
                        }

                    }

                }

                // CHekc if we have enough parallax, only then initialize

                double parallax = 0.0;
                if (parllaxVec.size() > 0 and depthVec.size()>0){
                    sort(parllaxVec.begin(),parllaxVec.end());
                    //size_t idx = min(50,int(parllaxVec.size()-1));
                   // parallax = acos(parllaxVec[idx])*180/CV_PI;
                    size_t idx = int(parllaxVec.size()/2);
                    parallax = parllaxVec[idx];

                    sort(depthVec.begin(),depthVec.end());
                    idx = int(depthVec.size()/2);
                    median_scene_depth = depthVec[idx];
                }
                VLOG(1) << "parallax" << parallax;


                if(parllaxVec.size() > 0 && parallax < 0.99998){
                    VLOG(2)<<"Baseline: "<<norm(T.colRange(3,4).rowRange(0,3));
                    VLOG(2)<<"Pose: "<<T;
                    VLOG(2)<<"Average cos parallax: "<<parallax;
                    VLOG(2)<<"Average Scene Depth: " <<avg_depth/num_triangulated;
                    VLOG(2)<<"Median Scene Depth: " <<median_scene_depth;
                    /// enough initial map points
                    if(num_triangulated > 50)
                    {
                        if(camconfig_.num_cams_ == 1 or inter_matches.size() < 50){
                             ///scale the translation to the scene depth
                            T.colRange(3,4).rowRange(0,3) = T.colRange(3,4).rowRange(0,3)/median_scene_depth;
                            VLOG(2)<<"Scaled baseline: "<<norm(T.colRange(3,4).rowRange(0,3));
                            VLOG(2)<<"Pose: "<<T;
                        }

                        VLOG(1)<<"Initialization DONE"<<endl;
                        VLOG(1)<<"Initial Map with landmarks :"<<num_triangulated<<endl;
                        VLOG(2)<<"fraction of Triangulation/number of matches"<<(float)num_triangulated/inter_matches.size();
                        VLOG(2)<<"fraction of Triangulation/pose inliers"<<(float)num_triangulated/num_initial_inliers;
                        //insert the two LF frames as keyframes
                        currentFramePose = T(cv::Range(0,3),cv::Range(0,4)).clone();
                        currentFrame->pose = T.clone();
                        //currentFramePose1 = T(Range(0,3),Range(0,4)).clone();
                        // currentFramePose2 = T(Range(0,3),Range(0,4)).clone();

                        lf_prev->numTrackedLMs = num_triangulated;
                        currentFrame->numTrackedLMs = num_triangulated;
                        insertKeyFrame();
                        //create Landmarks for each inlier and insert them into global map and well as
                        // the keyframes.
                        for (int i =0; i < inter_matches.size(); i++) {
                            DMatch m = inter_matches[i];
                            if(inliers[i])
                            {
                                // cout<<"3D point from Intra match triangulation"<<endl;
                                // cout<<lf_prev->points_3D[ m.queryIdx];
                                // cout<<"3D point form relative pose triangulation"<<endl;
                                // cout<<triangulatedPts[i];
                                // cout<<"descriptor distance : "<<m.distance<<endl;
                                if(camconfig_.num_cams_ == 1 or inter_matches.size() < 50){
                                    triangulatedPts[i] = triangulatedPts[i]/ median_scene_depth;
                                }
                                int l_id = map->insertLandmark( triangulatedPts[i], lf_prev, m.queryIdx, lf_prev->intraMatches[m.queryIdx].uv_ref);
                                Landmark* lm = map->getLandmark(l_id);
                                lm->addLfFrame(currentFrame, m.trainIdx, currentFrame->intraMatches[m.trainIdx].uv_ref);
//                                VLOG(2)<<"lm normal: " << lm->normal;
//                                VLOG(2)<<"Computed normal:"<<triangulatedPtNormals[i]; //added the normal to landmark when it is created
                                lf_prev->lIds[m.queryIdx] = l_id;
                                currentFrame->lIds[m.trainIdx] = l_id;

                                VLOG(3)<<"Inserted Landmark at"<<i<<"lid:"<<l_id<<": "<<triangulatedPts[i]<<endl;

                                //cout<<"prev_ind : "<<m.queryIdx<<"cur Ind: "<<m.trainIdx<<"Lid: "<<l_id<<endl;
                            }

                        }
                        //make the initialized flag as true
                        initialized_ = INITIALIZED;
                        first_keyframe = true;
                        initializationTrials = 0;
                        return true;
                    }
                }

            }
            // Not enough parallax
            if (initializationTrials > 20){
                initializationTrials = 0;
                // the initialization was not successful
                // delete the frame from lfFrames and insert the currentframe.
                lfFrames.pop_back();
                allPoses.pop_back();
                combined_poses.pop_back();
                combined_tStamps.pop_back();
                // allPoses1.pop_back();
//                allPoses1.pop_back();
                //allPoses2.clear();
                poseTimeStamps.pop_back();
            }
            return false;

        }
    }
}

void FrontEnd::findInterMatches(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                                std::vector<DMatch>& matches_z_filtered, std::vector<DMatch>& matches_mono_z_filtered,
                                bool viz){

    ///go through all the descriptors and support points
    /// put the descs into a Mat and create a keypoint for each support point.
    /// by default support point is in the reference frame

    // initialize a BF matcher
    Mat descs1 = Mat(lf_prev->intraMatches.size(), lf_prev->intraMatches[0].matchDesc.cols, CV_8U);
    Mat descs2 = Mat(lf_cur->intraMatches.size(), lf_cur->intraMatches[0].matchDesc.cols, CV_8U);
    vector<cv::KeyPoint> kps1, kps2;

    /// todo: mono uv_ref not in ref cam. fix
    /// todo: use vecDesc instead of recreating desc vector
    int ind = 0;
    for (auto &d : lf_prev->intraMatches) {
        d.matchDesc.copyTo(descs1.row(ind));
        kps1.push_back(KeyPoint(d.uv_ref, 3.0));
        ind++;
    }
    ind = 0;
    for (auto &d : lf_cur->intraMatches) {
        d.matchDesc.copyTo(descs2.row(ind));
        kps2.push_back(KeyPoint(d.uv_ref, 3.0));
        ind++;
    }

    std::vector<DMatch> good_matches;
    std::vector<DMatch> matches_z_outliers;
    vector<vector<vector<DMatch>>> vizMatches(lf_prev->num_cams_,vector<vector<DMatch>>(lf_cur->num_cams_, vector<DMatch>()));
    matches_z_filtered.clear();

    std::vector<int> inds1, inds2;
    std::vector<int> mono_inds1, mono_inds2;
    vector<int>::const_iterator inds1_it, inds2_it;
    vector<int>::const_iterator mono_inds1_it, mono_inds2_it;

    /// create a feature matcher and match the features
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    vector<vector<DMatch>> matches;
    matcher->knnMatch(descs1, descs2, matches, 2);

    ///filter the 3D points to get the matches
    vector<Point2f> ref_pts1, ref_pts2;
    int z_filtered_matches_cnt=0;
    vector<uint8_t> lm_flags;
    int num_lm_matches=0;
    int num_lm_matches_unique=0;
    int num_lm_matches_zfil = 0;

    /// go over the matches and get the unique matches
    for (auto &m : matches) {

        // if the match is a landmark, use it as it is even if it is not unique
        if(lf_prev->lIds[m[0].queryIdx] != -1){
            num_lm_matches++;
        }
        else{
            // if the match fails the ratio test move on
            if (m[0].distance > 0.7 * m[1].distance)
                continue;
        }
        good_matches.push_back(m[0]);

        /// if both the features are intramatches, check the difference in depth
        if(!lf_prev->intraMatches[m[0].queryIdx].mono and !lf_cur->intraMatches[m[0].trainIdx].mono) {
            cv::Mat  dif= lf_prev->intraMatches[m[0].queryIdx].point3D - lf_cur->intraMatches[m[0].trainIdx].point3D;
            float distance = cv::norm(dif);
            if(distance <= 2.0){   //
                z_filtered_matches_cnt++;
            }
            else{
                matches_z_outliers.push_back(m[0]);
                continue;
            }
        }

        ///find if the intermatch is already exising
        if(m[0].trainIdx < lf_cur->intramatch_size) {
            inds2_it = find(inds2.begin(), inds2.end(), m[0].trainIdx);
            mono_inds2_it = find(mono_inds2.begin(), mono_inds2.end(), m[0].trainIdx);
        }
        else{
            inds2_it = inds2.end();
            mono_inds2_it = find(mono_inds2.begin(), mono_inds2.end(), m[0].trainIdx);
        }
        ///if this is a new match
        if(inds2_it == inds2.end() && mono_inds2_it == mono_inds2.end())
        {
            int ind1 = m[0].queryIdx, ind2 = m[0].trainIdx;
            if(ind1 < lf_prev->intramatch_size && ind2 < lf_cur->intramatch_size) {
                inds2.push_back(ind2);
                inds1.push_back(ind1);
                matches_z_filtered.push_back(m[0]);
            }
            else {
                mono_inds2.push_back(ind2);
                mono_inds1.push_back(ind1);
                matches_mono_z_filtered.push_back((m[0]));
            }
            /// push matches into the viz matches structure
            IntraMatch im1 = lf_prev->intraMatches[m[0].queryIdx];
            IntraMatch im2 = lf_cur->intraMatches[m[0].trainIdx];
            int ii=0, ii2=0;
            if (im1.mono){
                ii=0;
                for(int featInd : im1.matchIndex){
                    if(featInd != -1)
                        break;
                    ii++;
                }
                assert(ii != lf_prev->num_cams_);

            }

            if(im2.mono){
                ii2=0;
                for(int featInd : im2.matchIndex){
                    if(featInd != -1)
                        break;
                    ii2++;
                }
                assert(ii2 != lf_cur->num_cams_);
            }

            vizMatches[ii][ii2].push_back(m[0]);

        }
        else if(mono_inds2_it == mono_inds2.end())/// the feature in second LF frame s already matched previouslly with an intramatch-intermatch
        {
            int idx_1 = inds1[ inds2_it - inds2.begin() ];
            double d = matches_z_filtered[inds2_it - inds2.begin()].distance; //norm( descs1.row(idx_1), descs2.row(m[0].trainIdx), NORM_HAMMING);
            if(m[0].distance < d)
            {
                inds1[  inds2_it - inds2.begin()] = m[0].queryIdx;
                matches_z_filtered[inds2_it - inds2.begin()] = m[0];
            }
        }
        else/// the feature in second LF frame s already matched previouslly with a mono-intermatch
        {
            int idx_1 = mono_inds1[ mono_inds2_it - mono_inds2.begin() ];
            double d = matches_mono_z_filtered[mono_inds2_it - mono_inds2.begin()].distance; //norm( descs1.row(idx_1), descs2.row(m[0].trainIdx), NORM_HAMMING);
            if(m[0].distance < d)
            {
                mono_inds1[  mono_inds2_it - mono_inds2.begin()] = m[0].queryIdx;
                matches_mono_z_filtered[mono_inds2_it - mono_inds2.begin()] = m[0];
            }
        }

    }


    for(int i =0; i >inds1.size(); i++){
        if(lf_prev->lIds[inds1[i]] != -1)
            num_lm_matches_zfil++;
        if(lf_prev->lIds[inds1[i]] != -1){
            lm_flags.push_back(255);
        }
        else{
            lm_flags.push_back(0);
        }
    }

    VLOG(1)<<"number of inter matches after ratio test : "<< good_matches.size()<<endl;
    VLOG(1) << "number of inter matches of Intramatch features after z test: " << z_filtered_matches_cnt << endl;
    //cout << "percentage of good points" << (z_filtered_matches_cnt / (float)good_matches.size()) * 100.0 << endl;
    VLOG(1)<<"Number of landmark matches"<<num_lm_matches_zfil<<endl;
    //cout<<"Number of landmark matches unique"<<num_lm_matches_unique<<endl;
    //VLOG(1)<<"Number of landmark matches z filtered"<<num_lm_matches_zfil<<endl;
    if( viz){
        for(int i=0; i <lf_prev->num_cams_; i++){
            for(int j=0; j < lf_cur->num_cams_; j++){
                Mat img_matches_mono;
                cv::drawMatches(lf_prev->imgs[i], kps1, lf_cur->imgs[j], kps2, vizMatches[i][j], img_matches_mono, Scalar::all(-1),
                                Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                ///draw the inatrmatch 2D points as well
                ////-- Show detected matches
                imshow("Inter Matches"+to_string(i)+to_string(j), img_matches_mono);
                waitKey(3);
            }
        }
    }
}


void FrontEnd::findInterMatches(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, std::vector<DMatch>& matches_z_filtered,  bool viz){

     ///go through all the descriptors and support points
    /// put the descs into a Mat and create a keypoint for each support point.
    /// by default support point is in the reference frame

    // initialize a BF matcher
    Mat descs1 = Mat(lf_prev->intraMatches.size(), lf_prev->intraMatches[0].matchDesc.cols, CV_8U);
    Mat descs2 = Mat(lf_cur->intraMatches.size(), lf_cur->intraMatches[0].matchDesc.cols, CV_8U);
    vector<cv::KeyPoint> kps1, kps2;

    /// todo: mono uv_ref not in ref cam. fix
    /// todo: use vecDesc instead of recreating desc vector
    int ind = 0;
    for (auto &d : lf_prev->intraMatches) {
        d.matchDesc.copyTo(descs1.row(ind));
        kps1.push_back(KeyPoint(d.uv_ref, 3.0));
        ind++;
    }
    ind = 0;
    for (auto &d : lf_cur->intraMatches) {
        d.matchDesc.copyTo(descs2.row(ind));
        kps2.push_back(KeyPoint(d.uv_ref, 3.0));
        ind++;
    }

    std::vector<DMatch> good_matches;
    std::vector<DMatch> matches_z_outliers;
    vector<vector<vector<DMatch>>> vizMatches(lf_prev->num_cams_,vector<vector<DMatch>>(lf_cur->num_cams_, vector<DMatch>()));
    matches_z_filtered.clear();

    std::vector<int> inds1, inds2;
    vector<int>::const_iterator inds1_it, inds2_it;

    /// create a feature matcher and match the features
     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
     vector<vector<DMatch>> matches;
     matcher->knnMatch(descs1, descs2, matches, 2);

    ///filter the 3D points to get the matches
    vector<Point2f> ref_pts1, ref_pts2;
    int z_filtered_matches_cnt=0;
    vector<uint8_t> lm_flags;
    int num_lm_matches=0;
    int num_lm_matches_unique=0;
    int num_lm_matches_zfil = 0;

    /// go over the matches and get the unique matches
    for (auto &m : matches) {

        // if the match is a landmark, use it as it is even if it is not unique
        if(lf_prev->lIds[m[0].queryIdx] != -1){
            num_lm_matches++;
        }
        else{
            // if the match fails the ratio test move on
            if (m[0].distance > 0.7 * m[1].distance)
                 continue;
        }
        good_matches.push_back(m[0]);

        /// if both the features are intramatches, check the difference in depth
        if(!lf_prev->intraMatches[m[0].queryIdx].mono and !lf_cur->intraMatches[m[0].trainIdx].mono) {
            cv::Mat  dif= lf_prev->intraMatches[m[0].queryIdx].point3D - lf_cur->intraMatches[m[0].trainIdx].point3D;
            float distance = cv::norm(dif);
            if(distance <= 2.0){   //
                z_filtered_matches_cnt++;
            }
            else{
                matches_z_outliers.push_back(m[0]);
                continue;
            }
        }

        ///find if the intermatch is already exising
        inds2_it = find(inds2.begin(), inds2.end(), m[0].trainIdx);
        ///if this is a new match
        if(inds2_it == inds2.end())
        {
            inds2.push_back(m[0].trainIdx);
            inds1.push_back(m[0].queryIdx);
            matches_z_filtered.push_back(m[0]);
            /// push matches into the viz matches structure
            IntraMatch im1 = lf_prev->intraMatches[m[0].queryIdx];
            IntraMatch im2 = lf_cur->intraMatches[m[0].trainIdx];
            int ii=0, ii2=0;
            if (im1.mono){
                ii=0;
                for(int featInd : im1.matchIndex){
                    if(featInd != -1)
                        break;
                    ii++;
                }
                assert(ii != lf_prev->num_cams_);

            }

            if(im2.mono){
                ii2=0;
                for(int featInd : im2.matchIndex){
                    if(featInd != -1)
                        break;
                    ii2++;
                }
                assert(ii2 != lf_cur->num_cams_);
            }

            vizMatches[ii][ii2].push_back(m[0]);

        }
        else /// the feature in second LF frame s already matched previouslly
        {
            int idx_1 = inds1[ inds2_it - inds2.begin() ];
            double d = matches_z_filtered[inds2_it - inds2.begin()].distance; //norm( descs1.row(idx_1), descs2.row(m[0].trainIdx), NORM_HAMMING);
            if(m[0].distance < d)
            {
                inds1[  inds2_it - inds2.begin()] = m[0].queryIdx;
                matches_z_filtered[inds2_it - inds2.begin()] = m[0];
            }
        }

    }


    for(int i =0; i >inds1.size(); i++){
        if(lf_prev->lIds[inds1[i]] != -1)
            num_lm_matches_zfil++;
        if(lf_prev->lIds[inds1[i]] != -1){
            lm_flags.push_back(255);
        }
        else{
            lm_flags.push_back(0);
        }
    }

    VLOG(1)<<"number of inter matches after ratio test : "<< good_matches.size()<<endl;
    VLOG(1) << "number of inter matches of Intramatch features after z test: " << z_filtered_matches_cnt << endl;
    //cout << "percentage of good points" << (z_filtered_matches_cnt / (float)good_matches.size()) * 100.0 << endl;
    VLOG(1)<<"Number of landmark matches"<<num_lm_matches_zfil<<endl;
    //cout<<"Number of landmark matches unique"<<num_lm_matches_unique<<endl;
    //VLOG(1)<<"Number of landmark matches z filtered"<<num_lm_matches_zfil<<endl;
    if( viz){
        for(int i=0; i <lf_prev->num_cams_; i++){
            for(int j=0; j < lf_cur->num_cams_; j++){
                Mat img_matches_mono;
                cv::drawMatches(lf_prev->imgs[i], kps1, lf_cur->imgs[j], kps2, vizMatches[i][j], img_matches_mono, Scalar::all(-1),
                                Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                ///draw the inatrmatch 2D points as well
                ////-- Show detected matches
                imshow("Inter Matches"+to_string(i)+to_string(j), img_matches_mono);
                waitKey(3);
            }
        }
    }
}


void FrontEnd::findInterMatchesBow(MultiCameraFrame *lf_prev, MultiCameraFrame *lf_cur, std::vector<DMatch> &matches,
                                   std::vector<DMatch> &mono_matches, bool viz) {

    vector<unsigned int> ind1, ind2;
    set<DBoW2::NodeId> words;
    InterMatchingBow(lf_prev, lf_cur, ind1, ind2, words);
    assert(ind1.size() == ind2.size());
    vector<vector<vector<DMatch>>> vizMatches(lf_prev->num_cams_, vector<vector<DMatch>>(lf_cur->num_cams_, vector<DMatch>()));

    for (int i = 0; i < ind1.size(); i++) {
        DMatch m;
        m.queryIdx = ind1[i];
        m.trainIdx = ind2[i];
        if (ind1[i] < lf_prev->intramatch_size) // && ind2[i] < lf_cur->intramatch_size)
            matches.push_back(m);
        else
            mono_matches.push_back(m);

        IntraMatch im1 = lf_prev->intraMatches[m.queryIdx];
        IntraMatch im2 = lf_cur->intraMatches[m.trainIdx];
        if (viz) {
            int ii = 0, ii2 = 0;
            if (im1.mono) {
                ii = 0;
                for (int featInd: im1.matchIndex) {
                    if (featInd != -1)
                        break;
                    ii++;
                }
                assert(ii != lf_prev->num_cams_);

            }

            if (im2.mono) {
                ii2 = 0;
                for (int featInd: im2.matchIndex) {
                    if (featInd != -1)
                        break;
                    ii2++;
                }
                assert(ii2 != lf_cur->num_cams_);
            }

            vizMatches[ii][ii2].push_back(m);
        }

    }

    // cout << "Total Number of Inter matches" << matches.size() << endl;

    if(false){

        Mat all;
        vector<cv::KeyPoint> kps1, kps2;
        int ind = 0;
        for (auto &d: lf_prev->intraMatches) {
            kps1.push_back(KeyPoint(d.uv_ref, 3.0));

            ind++;
        }
        ind = 0;
        for (auto &d: lf_cur->intraMatches) {
            kps2.push_back(KeyPoint(d.uv_ref, 3.0));
            ind++;
        }

        Mat img_matches_mono;
        cv::drawMatches(lf_prev->imgs[0], kps1, lf_cur->imgs[0], kps2, vizMatches[0][0], all,
                        Scalar(150,150,150),
                        Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);



        //Show the support matches
        cv::Mat dMap(Size(1,lf_prev->intramatch_size),CV_8UC3);
        int ind_Dmap=0;
        for(IntraMatch im : lf_prev->intraMatches){
            if(!im.mono) {
                dMap.at<uint8_t>(ind_Dmap, 0) = (uint8_t) im.point3D.at<double>(2, 0);
                ind_Dmap++;
            }

        }

        Mat out;
        //all.create(lf_prev->imgs[0].rows, lf_prev->imgs[0].cols * 2, CV_8UC3);
        cv::normalize(dMap, dMap, 0, 220, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(dMap, out, cv::COLORMAP_RAINBOW);
        ind_Dmap=0;
        for(IntraMatch im : lf_prev->intraMatches){
            if(!im.mono){
                Vec3b col1 = out.at<Vec3b>(ind_Dmap,0);
                cv::circle(all, im.uv_ref , 3, Scalar(col1[0],col1[1],col1[2]), 1,8,0 );
                ind_Dmap++;
            }
            else{

                if(im.matchIndex[0] != -1)
                    cv::circle(all, lf_prev->image_kps_undist[0][im.matchIndex[0]].pt ,  3, Scalar(255,255,255), 1 );

            }
        }

        //Show the support matches
        cv::Mat dMap2(Size(1,lf_cur->intramatch_size),CV_8UC1);
        int ind_Dmap2=0;
        for(IntraMatch im : lf_cur->intraMatches){
            if(!im.mono) {
                dMap2.at<uint8_t>(ind_Dmap2, 0) = (uint8_t) im.point3D.at<double>(2, 0);
                ind_Dmap2++;
            }

        }

        Mat out2;
        cv::normalize(dMap2, dMap2, 0, 220, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(dMap2, out2, cv::COLORMAP_RAINBOW);
        ind_Dmap2=0;
        for(IntraMatch im : lf_cur->intraMatches){
            if(!im.mono){
                Vec3b col1 = out2.at<Vec3b>(ind_Dmap2,0);
                cv::circle(all, im.uv_ref+ Point2f( lf_cur->imgs[0].cols , 0) , 3, Scalar(col1[0],col1[1],col1[2]), 1,8,0 );
                ind_Dmap2++;
            }
            else{
                if(im.matchIndex[0] != -1)
                    cv::circle(all, lf_cur->image_kps_undist[0][im.matchIndex[0]].pt+ Point2f( lf_cur->imgs[0].cols , 0),  3, Scalar(255,255,255), 1 );
            }
        }
        imshow("Inter Matches", all);
        waitKey(0);

    }


    if (viz) {

        vector<cv::KeyPoint> kps1, kps2;
        int ind = 0;
        for (auto &d: lf_prev->intraMatches) {
            kps1.push_back(KeyPoint(d.uv_ref, 3.0));

            ind++;
        }
        ind = 0;
        for (auto &d: lf_cur->intraMatches) {
            kps2.push_back(KeyPoint(d.uv_ref, 3.0));
            ind++;
        }

        Mat img_matches_mono;


        Mat mask = cv::Mat::zeros(matches.size(), 1, CV_8UC1);
        if(matches.size() == 0)
            mask = cv::Mat::zeros(mono_matches.size(), 1, CV_8UC1);
        mask = mask * 255;
        for (int i = 0; i < lf_prev->num_cams_; i++) {
            for (int j = 0; j < lf_cur->num_cams_; j++) {
                Mat img_matches_mono;
                cv::drawMatches(lf_prev->imgs[i], kps1, lf_cur->imgs[j], kps2, vizMatches[i][j], img_matches_mono,
                                Scalar::all(-1),
                                Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                ///draw the inatrmatch 2D points as well
                ////-- Show detected matches
                imshow("Inter Matches" + to_string(i) + to_string(j), img_matches_mono);
                waitKey(0);
            }
        }
    }

}

void FrontEnd::findInterMatchesBow(MultiCameraFrame *lf_prev, MultiCameraFrame *lf_cur, std::vector<DMatch> &matches,
                                   bool viz) {

    vector<unsigned int> ind1, ind2;
    set<DBoW2::NodeId> words;
    // perform intermatching using fbow rather than dbow for faster computations
//    cv::Mat img_desc1, img_desc2;
//    vector<Mat> img_desc1_vec, img_desc2_vec;
//    take the current frame and prev frames intramatches and convert them to descs
//    for(auto& im : lf_prev->intraMatches){
//        img_desc1.push_back(im.matchDesc);
//        img_desc1_vec.push_back(im.matchDesc);
//    }
//    for(auto& im : lf_cur->intraMatches){
//        img_desc2.push_back(im.matchDesc);
//        img_desc2_vec.push_back(im.matchDesc);
//    }
//    obtain the fbow bow and feature vectors
//    bowVector prevFrameBowVector;
//    bowFeatureVector prevFrameFeatVec;
//    orb_vocabulary_fbow->transform(img_desc1, 4, prevFrameBowVector, prevFrameFeatVec);
//    bowVector currentFrameBowVector;
//    bowFeatureVector currentFrameFeatVec;
//    orb_vocabulary_fbow->transform(img_desc2, 4, currentFrameBowVector, currentFrameFeatVec);
    // perform the intermatching
    auto start1 = std::chrono::high_resolution_clock::now();
    InterMatchingBow(lf_prev, lf_cur, ind1, ind2, words);
//    InterMatchingFBow(prevFrameFeatVec, currentFrameFeatVec, img_desc1_vec, img_desc2_vec, ind1, ind2);
    auto end1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1);
    VLOG(0) << "Time taken for intermatching using dbow: " << duration1.count() << " milliseconds" << endl;
//    ind1.clear();
//    ind2.clear();
//    InterMatchingBow(lf_prev->BoW_feats[0], lf_cur->BoW_feats[0], lf_prev->image_descriptors[0], lf_cur->image_descriptors[0], ind1, ind2);
//    VLOG(2)<<"Number of Bow Matches between prev left- cur left :"<<ind1.size();
//    ind1.clear();
//    ind2.clear();
//    InterMatchingBow(lf_prev->BoW_feats[0], lf_cur->BoW_feats[1], lf_prev->image_descriptors[0], lf_cur->image_descriptors[1], ind1, ind2);
//    VLOG(2)<<"Number of Bow Matches between prev left- cur right :"<<ind1.size();

    assert(ind1.size() == ind2.size());
    vector<vector<vector<DMatch>>> vizMatches(lf_prev->num_cams_,vector<vector<DMatch>>(lf_cur->num_cams_, vector<DMatch>()));

    for(int i=0; i<ind1.size(); i++){
        DMatch m;
        m.queryIdx = ind1[i];
        m.trainIdx = ind2[i];
        matches.push_back(m);

        IntraMatch im1 = lf_prev->intraMatches[m.queryIdx];
        IntraMatch im2 = lf_cur->intraMatches[m.trainIdx];
        if(viz){
            int ii=0, ii2=0;
            if (im1.mono){
                ii=0;
                for(int featInd : im1.matchIndex){
                    if(featInd != -1)
                        break;
                    ii++;
                }
                assert(ii != lf_prev->num_cams_);

            }

            if(im2.mono){
                ii2=0;
                for(int featInd : im2.matchIndex){
                    if(featInd != -1)
                        break;
                    ii2++;
                }
                assert(ii2 != lf_cur->num_cams_);
            }

            vizMatches[ii][ii2].push_back(m);
        }

    }
    VLOG(1)<<"Total Number of Inter matches "<<matches.size()<<endl;
    // cout<<"Total Number of Inter matches"<<matches.size()<<endl;
    if(viz){

        vector<cv::KeyPoint> kps1, kps2;
        int ind = 0;
        for (auto &d : lf_prev->intraMatches) {
            kps1.push_back(KeyPoint(d.uv_ref, 3.0));

            ind++;
        }
        ind = 0;
        for (auto &d : lf_cur->intraMatches) {
            kps2.push_back(KeyPoint(d.uv_ref, 3.0));
            ind++;
        }

        Mat img_matches_mono;
        Mat mask = cv::Mat::zeros(matches.size(),1,CV_8UC1);
        mask = mask *255;
        for(int i=0; i <lf_prev->num_cams_; i++){
            for(int j=0; j < lf_cur->num_cams_; j++){
                Mat img_matches_mono;
                cv::drawMatches(lf_prev->imgs[i], kps1, lf_cur->imgs[j], kps2, vizMatches[i][j], img_matches_mono, Scalar::all(-1),
                                Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

                ///draw the inatrmatch 2D points as well
                ////-- Show detected matches
                imshow("InterMatches"+to_string(i)+to_string(j), img_matches_mono);
                waitKey(0);
            }
        }
    }

}


void FrontEnd::InterMatchingBow( DBoW2::FeatureVector lfFeatVec1, DBoW2::FeatureVector lfFeatVec2, std::vector<cv::Mat> img_desc1, std::vector<cv::Mat> img_desc2, vector<unsigned int>& indices_1,
                                vector<unsigned int>& indices_2 ){
    indices_2.clear();
    indices_1.clear();

    int itr_cng_match=0;
    // iterators for each of the feature vectors
    DBoW2::FeatureVector::const_iterator featVec1_it, featVec2_it;
    featVec1_it = lfFeatVec1.begin();
    featVec2_it = lfFeatVec2.begin();

    const auto featVec1_end = lfFeatVec1.end(); // std::next(featVec1.begin(),200);    //
    const auto featVec2_end = lfFeatVec2.end(); //std::next(featVec2.begin(),200);   //
    // until both the feature vectors iterators reach the end



    while(featVec1_it != featVec1_end && featVec2_it != featVec2_end)
    {
        // check if the node ID of both the vectors is same.
        if(featVec1_it->first == featVec2_it->first)
        {
            // compute matches between the features corresponding to
            // the matching node ID. The decriptors indices are present in the
            //corresponding featVec[it]->second. The actual descriptors of the key points
            // in the images are present in image_descriptors[i/j]
            vector<unsigned int> i_ind_tmp, j_ind_tmp;
            DBoW2::NodeId w1= featVec1_it->first;
            DBoW2::NodeId w2= featVec2_it->first;

            orBextractor->getMatches_distRatio(img_desc1, featVec1_it->second,
                                         img_desc2, featVec2_it->second, i_ind_tmp, j_ind_tmp,
                                         itr_cng_match);

            indices_1.insert(indices_1.end(), i_ind_tmp.begin(), i_ind_tmp.end());
            indices_2.insert(indices_2.end(), j_ind_tmp.begin(), j_ind_tmp.end());

            // move featVec1_it and featVec2_it forward
            ++featVec1_it;
            ++featVec2_it;
        }
        else if(featVec1_it->first < featVec2_it->first)
        {
            // move old_it forward
            featVec1_it = lfFeatVec1.lower_bound(featVec2_it->first);
            // old_it = (first element >= cur_it.id)
        }
        else
        {
            // move cur_it forward
            featVec2_it = lfFeatVec2.lower_bound(featVec1_it->first);
            // cur_it = (first element >= old_it.id)
        }
    }
}

void FrontEnd::InterMatchingFBow( bowFeatureVector lfFeatVec1, bowFeatureVector lfFeatVec2, vector<Mat> img_desc1_vec, vector<Mat> img_desc2_vec, vector<unsigned int>& indices_1,
                                 vector<unsigned int>& indices_2 ){
    indices_2.clear();
    indices_1.clear();

    int itr_cng_match=0;
    // iterators for each of the feature vectors
    bowFeatureVector::const_iterator featVec1_it, featVec2_it;
    featVec1_it = lfFeatVec1.begin();
    featVec2_it = lfFeatVec2.begin();

    const auto featVec1_end = lfFeatVec1.end(); // std::next(featVec1.begin(),200);    //
    const auto featVec2_end = lfFeatVec2.end(); //std::next(featVec2.begin(),200);   //
    // until both the feature vectors iterators reach the end



    while(featVec1_it != featVec1_end && featVec2_it != featVec2_end)
    {
        // check if the node ID of both the vectors is same.
        if(featVec1_it->first == featVec2_it->first)
        {
            // compute matches between the features corresponding to
            // the matching node ID. The decriptors indices are present in the
            //corresponding featVec[it]->second. The actual descriptors of the key points
            // in the images are present in image_descriptors[i/j]
            vector<unsigned int> i_ind_tmp, j_ind_tmp;
            NodeId w1= featVec1_it->first;
            NodeId w2= featVec2_it->first;


            orBextractor->getMatches_distRatio(img_desc1_vec, featVec1_it->second,
                                               img_desc2_vec, featVec2_it->second, i_ind_tmp, j_ind_tmp,
                                               itr_cng_match);

            indices_1.insert(indices_1.end(), i_ind_tmp.begin(), i_ind_tmp.end());
            indices_2.insert(indices_2.end(), j_ind_tmp.begin(), j_ind_tmp.end());

            // move featVec1_it and featVec2_it forward
            ++featVec1_it;
            ++featVec2_it;
        }
        else if(featVec1_it->first < featVec2_it->first)
        {
            // move old_it forward
            featVec1_it = lfFeatVec1.lower_bound(featVec2_it->first);
            // old_it = (first element >= cur_it.id)
        }
        else
        {
            // move cur_it forward
            featVec2_it = lfFeatVec2.lower_bound(featVec1_it->first);
            // cur_it = (first element >= old_it.id)
        }
    }
}


void FrontEnd::InterMatchingBow(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur, vector<unsigned int>& indices_1,
                                vector<unsigned int>& indices_2,set<DBoW2::NodeId>& words ){
    indices_2.clear();
    indices_1.clear();

    int itr_cng_match=0;
    // iterators for each of the feature vectors
    DBoW2::FeatureVector::const_iterator featVec1_it, featVec2_it;
    featVec1_it = lf_prev->lfFeatVec.begin();
    featVec2_it = lf_cur->lfFeatVec.begin();

    const auto featVec1_end = lf_prev->lfFeatVec.end(); // std::next(featVec1.begin(),200);    //
    const auto featVec2_end = lf_cur->lfFeatVec.end(); //std::next(featVec2.begin(),200);   //
    // until both the feature vectors iterators reach the end

    std::vector<cv::Mat> img_desc1, img_desc2;

    for (auto &d : lf_prev->intraMatches)
        img_desc1.push_back(d.matchDesc);

    for (auto &d : lf_cur->intraMatches)
        img_desc2.push_back(d.matchDesc);



    while(featVec1_it != featVec1_end && featVec2_it != featVec2_end)
    {
        // check if the node ID of both the vectors is same.
        if(featVec1_it->first == featVec2_it->first)
        {
            // compute matches between the features corresponding to
            // the matching node ID. The decriptors indices are present in the
            //corresponding featVec[it]->second. The actual descriptors of the key points
            // in the images are present in image_descriptors[i/j]
            vector<unsigned int> i_ind_tmp, j_ind_tmp;
            DBoW2::NodeId w1= featVec1_it->first;
            DBoW2::NodeId w2= featVec2_it->first;

            orBextractor->getMatches_distRatio(img_desc1, featVec1_it->second,
                                 img_desc2, featVec2_it->second, i_ind_tmp, j_ind_tmp,
                                 itr_cng_match);

            indices_1.insert(indices_1.end(), i_ind_tmp.begin(), i_ind_tmp.end());
            indices_2.insert(indices_2.end(), j_ind_tmp.begin(), j_ind_tmp.end());

            if(i_ind_tmp.size()>= 1){
                //cout<<"WordID :"<< featVec1_it->first <<endl;
                words.insert(featVec1_it->first);
            }
            // move featVec1_it and featVec2_it forward
            ++featVec1_it;
            ++featVec2_it;
        }
        else if(featVec1_it->first < featVec2_it->first)
        {
            // move old_it forward
            featVec1_it = lf_prev->lfFeatVec.lower_bound(featVec2_it->first);
            // old_it = (first element >= cur_it.id)
        }
        else
        {
            // move cur_it forward
            featVec2_it = lf_cur->lfFeatVec.lower_bound(featVec1_it->first);
            // cur_it = (first element >= old_it.id)
        }
    }
}

//void FrontEnd::poseFromPCAlignment(opengv::points_t points1, opengv::points_t points2, Mat& T


//// Here matches have queryIDx indexing lids vector and TrainIDx indexing currentFrame's  Intramatches vector.
/// matches and inliers have same size. inliers is a mask on matches.
void FrontEnd::OptimizePose(vector<DMatch> matches, vector<int> lids, vector<bool>& inliers, transformation_t nonlinear_transformation, Mat& poseOut ){

    NonlinearFactorGraph graph;
    // Define the camera observation noise model
    auto measurementNoise =
            noiseModel::Isotropic::Sigma(2, 1.0);  // one pixel in u and v // it was 2.0 in imu_integrate
    noiseModel::Robust::shared_ptr huberModel = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(
            sqrt(5.991)), measurementNoise);

    vector<int> fac_to_match;
    int Idx =0;
    int factorInd =0;
    Mat Twb_34, Twb;
    cv::eigen2cv(nonlinear_transformation, Twb_34);
    Twb = Mat::eye(4, 4, CV_64F);
    Twb_34.copyTo(Twb(cv::Range(0, 3), cv::Range(0, 4)));
    //for(size_t i = 0; i < ransac.inliers_.size(); i++)
    for(int i =0; i < matches.size(); i++)
    {
        DMatch m = matches[i];
        Mat pt3d = map->getLandmark(lids[m.queryIdx])->pt3D;
        gtsam::Point3 P_ = gtsam::Point3(pt3d.at<double>(0,0), pt3d.at<double>(1,0), pt3d.at<double>(2,0));

        IntraMatch iMatch_cur = currentFrame->intraMatches[m.trainIdx];
        for (int camInd = 0; camInd <currentFrame->num_cams_ ; camInd++){
            if(iMatch_cur.matchIndex[camInd] != -1){

                gtsam::Pose3 bTc = RT_Mats[camInd];
                Cal3_S2::shared_ptr K(new Cal3_S2(camconfig_.K_mats_[camInd].at<double>(0,0),
                                                  camconfig_.K_mats_[camInd].at<double>(1,1), 0.0,
                                                  camconfig_.K_mats_[camInd].at<double>(0,2),
                                                  camconfig_.K_mats_[camInd].at<double>(1,2)));
                cv::KeyPoint kp = currentFrame->image_kps_undist[camInd][iMatch_cur.matchIndex[camInd]];
                gtsam::Point2 p_(kp.pt.x, kp.pt.y);
                double expected_x =0.0, expected_y=0.0;
                project3dPoint(pt3d, camInd, Twb, expected_x,expected_y);
                double err = (expected_x - kp.pt.x)*(expected_x - kp.pt.x)+(expected_y - kp.pt.y)*(expected_y - kp.pt.y);
                if(err > 100 and inliers[i]){
                    //Visualize this match with last keyframe to see if it is an outlier
                    MultiCameraFrame* prev_lf = map->getLandmark(lids[m.queryIdx])->KFs.back();
                    int imatch_prev_ind = map->getLandmark(lids[m.queryIdx])->featInds.back();

                    IntraMatch iMatch_prev = prev_lf->intraMatches[imatch_prev_ind];

                    for(int aa=0; aa< currentFrame->num_cams_ ; aa++){
                        VLOG(2)<<aa<<": "<< iMatch_prev.matchIndex[aa]<<","<<iMatch_cur.matchIndex[aa];
                    }
                   // drawInterMatch(prev_lf, currentFrame, iMatch_prev,iMatch_cur, "Intermatch");

                    VLOG(2)<<"Initial error : "<<(expected_x - kp.pt.x)<<" , "<<(expected_y - kp.pt.y);
                    VLOG(2)<<camconfig_.R_mats_[camInd];
                    VLOG(2)<<camconfig_.t_mats_[camInd];

                    VLOG(2)<<camconfig_.K_mats_[camInd];
                    Mat P1 = build_Rt(camconfig_.R_mats_[camInd], camconfig_.t_mats_[camInd]);
                    Mat Tbw = Twb.inv();
                    Mat p3d_conv_body = Tbw(cv::Range(0, 3), cv::Range(0, 3)) * pt3d + Tbw(cv::Range(0, 3), cv::Range(3, 4));
                    VLOG(2)<<"point converted tobidy frame:";
                    VLOG(2)<<p3d_conv_body;

                    Mat p3d_conv_cam = P1(cv::Range(0, 3), cv::Range(0, 3)) * p3d_conv_body + P1(cv::Range(0, 3), cv::Range(3, 4));
                    VLOG(2)<<"point converted to cam frame:";
                    VLOG(2)<<p3d_conv_cam;
                    Mat projected = camconfig_.K_mats_[camInd] * p3d_conv_cam;
                    VLOG(2)<<projected.at<double>(0,0) / projected.at<double>(2,0)<<","<<projected.at<double>(1,0) / projected.at<double>(2,0);

                }

                //// This is to account for keypoint scale. This noise was not used in imu_integrate
                auto measurementNoise1 =
                        noiseModel::Isotropic::Sigma(2, orBextractor->GetScaleSigmaSquares()[ kp.octave]);
                noiseModel::Robust::shared_ptr huberModel1 = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(
                        sqrt(5.991)), measurementNoise1);
                graph.emplace_shared<RigResectioningFactor>(huberModel, Symbol('x', 0), K, p_, P_, kp.octave, camInd, bTc);
                fac_to_match.push_back(Idx);
                factorInd++;
                //break;
            }
        }
        Idx++;
    }

    inliers = vector<bool>(matches.size(),true);
    LevenbergMarquardtParams lm_params;
    LevenbergMarquardtParams::SetCeresDefaults(&lm_params);
    lm_params.maxIterations = 25;
    Values result;
    auto startT = high_resolution_clock::now();

    for(int iter =0; iter < 2 ; iter++){
        Values initialEstimate;
        gtsam::Pose3 initPose(gtsam::Rot3(nonlinear_transformation.block<3,3>(0,0)),
                              gtsam::Point3(nonlinear_transformation.col(3)));
        initialEstimate.insert(Symbol('x', 0), initPose);

        result = LevenbergMarquardtOptimizer(graph, initialEstimate, lm_params).optimize();
        //cout<<"Graph Num Factors: "<<graph.nrFactors()<<endl;
        //result.print("Iteration "+ to_string(iter)+"results:\n");
        VLOG(3) << "initial error = " << graph.error(initialEstimate) << endl;
        VLOG(3) << "final error = " << graph.error(result) << endl;
        //auto sT = high_resolution_clock::now();
        int outlier_cnt=0;
        gtsam::Pose3 curest = result.at<gtsam::Pose3>(Symbol('x', 0));
        for(int facInd =0 ; facInd < graph.size(); facInd++){
            if(graph.at(facInd)){
                std::shared_ptr<RigResectioningFactor> fac = std::static_pointer_cast<RigResectioningFactor>(graph.at(facInd));
                // check if this is an oulier?
                gtsam::Point2 errVec = fac->evaluateError(curest);

                gtsam::Matrix infMat = gtsam::Matrix::Identity(2,2) * orBextractor->GetInverseScaleSigmaSquares()[ fac->octave()];
                double error = dot(errVec, infMat*errVec );


                if(error > 5.991){
                    // this is an outlier. mark it
                    graph.remove(facInd);
                    inliers[fac_to_match[facInd]] = false;
                    //VLOG(2)<<"Big reprojection error . Deleting factor.err: "<<error;
                    outlier_cnt++;
                }
            }

        }
        VLOG(3)<<"Number of removed factors as outliers: "<<outlier_cnt;
       // auto stT = high_resolution_clock::now();
       // auto duration = duration_cast<milliseconds>(stT - sT);
        //
        //VLOG(2)<<"Total time taken for removing factors gtsam: "<<duration.count()<<endl;
    }
    auto stopT = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stopT - startT);
    //
    VLOG(3)<<"Total time taken for optimization in pose estimation: "<<duration.count()<<endl;

    gtsam::Pose3 pose = result.at<gtsam::Pose3>(  gtsam::Symbol('x', 0));
    gtsam::Matrix matPose= pose.matrix();
    cv::eigen2cv(matPose, poseOut);
}

void FrontEnd::project3dPoint(const Mat &pt3d, int camInd, Mat Twb, double& exp_x, double& exp_y) {//project this point to see the initial error
    Mat P1 = build_Rt(camconfig_.R_mats_[camInd], camconfig_.t_mats_[camInd]);
    Mat Tbw = Twb.inv();
    Mat p3d_conv_body = Tbw(cv::Range(0, 3), cv::Range(0, 3)) * pt3d + Tbw(cv::Range(0, 3), cv::Range(3, 4));
    Mat p3d_conv_cam = P1(cv::Range(0, 3), cv::Range(0, 3)) * p3d_conv_body + P1(cv::Range(0, 3), cv::Range(3, 4));
    Mat projected = camconfig_.K_mats_[camInd] * p3d_conv_cam;
    exp_x = projected.at<double>(0,0) / projected.at<double>(2,0);
    exp_y = projected.at<double>(1,0) / projected.at<double>(2,0);
}

int FrontEnd::estimatePoseLF(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                             std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T, bool SAC){
    int num_inliers=0;
    switch (posestAlgo_){
        case PC_ALIGN:
            num_inliers=poseFromPCAlignment(lf_prev,lf_cur,matches, inliers, T, SAC );
            break;
        case SEVENTEEN_PT:
            num_inliers=poseFromSeventeenPt(lf_prev,lf_cur,matches, inliers, T, SAC);
            break;
        case G_P3P:
            num_inliers=absolutePoseFromGP3P(lf_prev,lf_cur,matches, inliers, T, SAC);
            break;
        default:
            break;
    }
    return num_inliers;


}
int FrontEnd::poseFromPCAlignment(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                                   std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T, bool SAC ){
    opengv::points_t points1, points2;
    inliers = vector<bool>(matches.size(),false);
    if(initialized_== INITIALIZED)
    {
        for (auto &m : matches) {
            int l_id = lf_prev->lIds[m.queryIdx];
            Mat pt3d= map->getLandmark(l_id)->pt3D;
            Eigen::Vector3d pt;
            for(int j=0; j<3; ++j)
                pt[j] = pt3d.at<double>(j,0);
            points1.push_back(pt);
            points2.push_back( lf_cur->points_3D[m.trainIdx]);
        }
    }
    else{
        for (auto &m : matches) {
            points1.push_back(lf_prev->points_3D[m.queryIdx]);
            points2.push_back( lf_cur->points_3D[m.trainIdx]);
        }
    }


    // normal
    opengv::point_cloud::PointCloudAdapter adapter(points1, points2);
    // run the non-linear optimization over all correspondences
    transformation_t nonlinear_transformation = point_cloud::optimize_nonlinear(adapter);
    cout<<"estimated pose : "<<endl;
    cout<<nonlinear_transformation<<endl;


    //sac style orientation

    //Create a PointCloudSacProblem and Ransac
    /*sac::Ransac<
            sac_problems::point_cloud::PointCloudSacProblem> ransac;
    std::shared_ptr<
            sac_problems::point_cloud::PointCloudSacProblem> relposeproblem_ptr(
            new sac_problems::point_cloud::PointCloudSacProblem(adapter));
    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = 0.2;
    ransac.max_iterations_ = 100;

    ransac.computeModel(0);
    transformation_t nonlinear_transformation = ransac.model_coefficients_;
    cout<<"estimated pose sac :"<<endl;
    std::cout << ransac.model_coefficients_ << std::endl << std::endl;
    std::cout << "the number of inliers in RANSAC is: " << ransac.inliers_.size()<<endl;*/


    //CHECK/DEBUG THE 3d-3d pose estimation
    int ii=0;
    int inlier_count = 0;
    float avg_error = 0;
    for (auto p : points2){
        opengv::point_t  p2_in_1 = nonlinear_transformation.block<3,3>(0,0) * p + nonlinear_transformation.col(3);
        opengv::point_t delta = points1.at(ii) - p2_in_1;
        avg_error = avg_error + delta.norm();
        if(delta.norm() < 0.1){
            inlier_count++;
            inliers[ii] = true;
           // cout<<"delta : "<< delta.norm()<<endl;
            //cout<<"P1 : "<<points1.at(ii) ;
            //cout<<"P2 in 1 : "<<p2_in_1<<endl;
        }
        else{
           // cout<<"delta : "<< delta.norm()<<endl;
            //cout<<"P1 : "<<points1.at(ii) ;
            //cout<<"P2 in 1 : "<<p2_in_1<<endl;
        }

        ii++;
    }
    cout<<"Average error : "<<avg_error/points1.size()<<endl;
    cout<<"Number of inlier intra matches after 3pt : "<<inlier_count<<","<<(float)inlier_count/points1.size()*100.0<<"%"<<endl;


    Mat curPos;
    cv::eigen2cv(nonlinear_transformation, curPos);
    double n = cv::norm(curPos.colRange(3,4));
    /*if(n > 1e-6){
        Mat norm_t = curPos.colRange(3,4) * 2 ;
        norm_t.copyTo(curPos.colRange(3,4));
    }*/

    T = Mat::eye(4, 4, CV_64F);
    curPos.copyTo(T(cv::Range(0, 3), cv::Range(0, 4)));
    return inlier_count;
}

int FrontEnd::poseFromSeventeenPt(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                        std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T,  bool SAC){


    bearingVectors_t bearingVectors1;
    bearingVectors_t bearingVectors2;
    std::vector<int> camCorrespondences1;
    std::vector<int> camCorrespondences2;

    ////////////////////////////////////////////////////

    inliers = vector<bool>(matches.size(),false);

    for (auto &m : matches) {
        IntraMatch iMatch = lf_prev->intraMatches[m.queryIdx];
        for (int i =0; i <currentFrame->num_cams_ ; i++){
            if(iMatch.matchIndex[i] != -1){
                Point2f uv_coord = lf_prev->image_kps_undist[i][iMatch.matchIndex[i]].pt;
                //camconfig_.K_mats_[i].inv();
                double norm_x = (uv_coord.x - camconfig_.K_mats_[i].at<double>(0,2))/camconfig_.K_mats_[i].at<double>(0,0);
                double norm_y = (uv_coord.y - camconfig_.K_mats_[i].at<double>(1,2)) / camconfig_.K_mats_[i].at<double>(1,1);
                Eigen::Vector3d pt;
                pt[0] = norm_x;
                pt[1] = norm_y;
                pt[2] = 1.0;
                bearingVectors1.push_back(pt/pt.norm());
                camCorrespondences1.push_back(i);
                break;
            }
        }

        IntraMatch iMatch_cur = lf_cur->intraMatches[m.trainIdx];
        for (int i =0; i <currentFrame->num_cams_ ; i++){
            if(iMatch_cur.matchIndex[i] != -1){
                Point2f uv_coord = lf_cur->image_kps_undist[i][iMatch_cur.matchIndex[i]].pt;
                //camconfig_.K_mats_[i].inv();
                double norm_x = (uv_coord.x - camconfig_.K_mats_[i].at<double>(0,2))/camconfig_.K_mats_[i].at<double>(0,0);
                double norm_y = (uv_coord.y - camconfig_.K_mats_[i].at<double>(1,2)) / camconfig_.K_mats_[i].at<double>(1,1);
                Eigen::Vector3d pt;
                pt[0] = norm_x;
                pt[1] = norm_y;
                pt[2] = 1.0;
                bearingVectors2.push_back(pt/pt.norm());
                camCorrespondences2.push_back(i);
                break;
            }
        }

    }

    ////////////////////////////////////////////////////

    //for (int i =0; i <currentFrame->num_cams_ ; i++){
   //    (*currentFrame->rotations_ptr)[i];
   // }
    //create non-central relative adapter
    relative_pose::NoncentralRelativeAdapter adapter(
            bearingVectors1,
            bearingVectors2,
            camCorrespondences1,
            camCorrespondences2,
            currentFrame->translations,
            *currentFrame->rotations_ptr);

    //Create a NoncentralRelativePoseSacProblem and Ransac
    sac::Ransac<
            sac_problems::relative_pose::NoncentralRelativePoseSacProblem> ransac;
    std::shared_ptr<sac_problems::relative_pose::NoncentralRelativePoseSacProblem>relposeproblem_ptr(
            new sac_problems::relative_pose::NoncentralRelativePoseSacProblem(
                    adapter,
                    sac_problems::relative_pose::NoncentralRelativePoseSacProblem::SEVENTEENPT));
    ransac.sac_model_ = relposeproblem_ptr;
    ransac.threshold_ = 2.0*(1.0 - cos(atan(sqrt(2.0)*0.5/camconfig_.K_mats_[0].at<double>(0,2))));
    ransac.max_iterations_ = 10000;

    //Run the experiment
    struct timeval tic;
    struct timeval toc;
    gettimeofday( &tic, 0 );
    ransac.computeModel();
    gettimeofday( &toc, 0 );
    double ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));

    //print the results
    // std::cout<<"/////////////////////SEVENTEEN PT RANSAC/////////////////////"<<endl;
    //std::cout << "the ransac threshold is: " << ransac.threshold_ << std::endl;
    //std::cout << "the ransac results is: " << std::endl;
    //std::cout << ransac.model_coefficients_ << std::endl << std::endl;
    //std::cout << "Ransac needed " << ransac.iterations_ << " iterations and ";
    //std::cout << ransac_time << " seconds" << std::endl << std::endl;
    // std::cout << "the number of inliers is: " << ransac.inliers_.size();
    // std::cout << std::endl << std::endl;
    //std::cout << "the found inliers are: " << std::endl;
    //for(size_t i = 0; i < ransac.inliers_.size(); i++)
    //    std::cout << ransac.inliers_[i] << " ";
    //std::cout << std::endl << std::endl;

    transformation_t nonlinear_transformation = ransac.model_coefficients_;
    Mat curPos;
    cv::eigen2cv(nonlinear_transformation, curPos);
    T = Mat::eye(4, 4, CV_64F);
    curPos.copyTo(T(cv::Range(0, 3), cv::Range(0, 4)));



    // get the last estimated pose WRT world into WTp
    Mat rot_p = allPoses.back().colRange(0,3);
    Mat t_p = allPoses.back().colRange(3,4);
    Mat WTp = Mat::eye(4, 4, CV_64F);
    rot_p.copyTo(WTp(cv::Range(0, 3), cv::Range(0, 3)));
    t_p.copyTo(WTp(cv::Range(0, 3), cv::Range(3, 4)));

    // Now convert the current pose WRT world
    unique_lock<mutex> lock(mMutexPose);
    Mat WTc =  WTp*T;
    T = WTc.clone();
    // cout<<"Previous pose: "<<WTp<<endl;
    // cout<<"POSE ESTIMATED FROM SEVENTEEN WRT world: "<<endl;
    // cout<<T<<endl;
    // cout<<"translation :"<< cv::norm(t_p - T(cv::Range(0, 3), cv::Range(3, 4)))<<endl;

    for(size_t i = 0; i < ransac.inliers_.size(); i++)
        inliers[ransac.inliers_[i]] = true;
    return ransac.inliers_.size();

}


int FrontEnd::absolutePoseFromGP3P(MultiCameraFrame* lf_prev, MultiCameraFrame* lf_cur,
                         std::vector<DMatch>& matches, vector<bool>& inliers, Mat& T,  bool SAC){


    bearingVectors_t bearingVectors;
    points_t points;
    std::vector<int> camCorrespondences;

    std::vector<DMatch> matches_fil;
    // For each match
    for (auto &m : matches) {
        Mat pt3d;
        if(initialized_==INITIALIZED){
            int l_id = lf_prev->lIds[m.queryIdx];
            pt3d= map->getLandmark(l_id)->pt3D;
        }
        else{
            pt3d = lf_prev->intraMatches[m.queryIdx].point3D;
        }
        Eigen::Vector3d pt;
        for(int j=0; j<3; ++j)
            pt[j] = pt3d.at<double>(j,0);

        IntraMatch iMatch_cur = lf_cur->intraMatches[m.trainIdx];
        for (int i =0; i <currentFrame->num_cams_ ; i++){
            if(iMatch_cur.matchIndex[i] != -1){
//                if(lf_prev->intraMatches[m.queryIdx].matchIndex[i] == -1){
//                    VLOG(2)<<"This is a non overlapping setup and the kp match is found in a different camera";
//                    break;
//                }

                Point2f uv_coord = lf_cur->image_kps_undist[i][iMatch_cur.matchIndex[i]].pt;
                //camconfig_.K_mats_[i].inv();
                double norm_x = (uv_coord.x - camconfig_.K_mats_[i].at<double>(0,2))/camconfig_.K_mats_[i].at<double>(0,0);
                double norm_y = (uv_coord.y - camconfig_.K_mats_[i].at<double>(1,2)) / camconfig_.K_mats_[i].at<double>(1,1);
                Eigen::Vector3d pt_n;
                pt_n[0] = norm_x;
                pt_n[1] = norm_y;
                pt_n[2] = 1.0;
                /// fill in the opengv correspondence vectors
                points.push_back(pt);
                bearingVectors.push_back(pt_n/pt_n.norm());
                camCorrespondences.push_back(i);
                matches_fil.push_back(m);
                break;
            }
        }
    }
    matches = matches_fil;
    inliers = vector<bool>(matches.size(),false);
    vector<bool> ransac_inliers = vector<bool>(matches.size(),false);
    int num_inliers =0;
    //drawInterMatches(lf_prev, lf_cur, matches_fil, vector<bool>(matches_fil.size(),true), "InterMatches");
    //create a non-central absolute adapter
    absolute_pose::NoncentralAbsoluteAdapter adapter(
            bearingVectors,
            camCorrespondences,
            points,
            currentFrame->translations,
            *currentFrame->rotations_ptr);
     //////////////////////////////////////
     ////////  RANSAC //////////////////////
    //Create a AbsolutePoseSacProblem and Ransac
    //The method is set to GP3P
    sac::Ransac<
            sac_problems::absolute_pose::AbsolutePoseSacProblem> ransac;
    std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem> absposeproblem_ptr(
            new sac_problems::absolute_pose::AbsolutePoseSacProblem( adapter,
                    sac_problems::absolute_pose::AbsolutePoseSacProblem::GP3P));
    ransac.sac_model_ = absposeproblem_ptr;
    ransac.threshold_ = 1.0 - cos(atan(sqrt(2.0)*0.5/camconfig_.K_mats_[0].at<double>(0,2)));
    ransac.max_iterations_ = 100; // 50 for imu_integrate


    //Run the experiment
    struct timeval tic;
    struct timeval toc;
    gettimeofday( &tic, 0 );
    // VLOG(1) << " RANSAC in  absolutePoseFromGP3P" << std::endl;
    ransac.computeModel();
    gettimeofday( &toc, 0 );
    double ransac_time = TIMETODOUBLE(timeval_minus(toc,tic));

    //print the results
    VLOG(2)<<"/////////////////////ABSOLUTE POSE RANSAC/////////////////////"<<endl;
    VLOG(2) << "the ransac results is: " << std::endl;
    VLOG(2) << ransac.model_coefficients_ << std::endl << std::endl;
    VLOG(3) << "Ransac needed " << ransac.iterations_ << " iterations and ";
    VLOG(3) << ransac_time << " seconds" << std::endl << std::endl;
    VLOG(3) << "the number of inliers is: " << ransac.inliers_.size();
    VLOG(3) << std::endl << std::endl;
    //VLOG(3) << "the found inliers are: " << std::endl;
    //for(size_t i = 0; i < ransac.inliers_.size(); i++)
    //    VLOG(3) << ransac.inliers_[i] << " ";
    //VLOG(3) << std::endl << std::endl;

    for(size_t i = 0; i < ransac.inliers_.size(); i++){
        ransac_inliers[ransac.inliers_[i]] = true;
        inliers[ransac.inliers_[i]] = true;
    }

    transformation_t nonlinear_transformation = ransac.model_coefficients_;
    Mat curPos;
    cv::eigen2cv(nonlinear_transformation, curPos);
    T = Mat::eye(4, 4, CV_64F);
    curPos.copyTo(T(cv::Range(0, 3), cv::Range(0, 4)));
    VLOG(2)<<"Ransac Done";
    //drawInterMatches(lf_prev, lf_cur, matches, inliers, "InterMatches");

    ///////////// Optimization //////////////////////

    if(initialized_ == INITIALIZED) {
        //cv::cv2eigen(lf_prev->pose.rowRange(0, 3), nonlinear_transformation);
        OptimizePose(matches, lf_prev->lIds, inliers, nonlinear_transformation, T);
        VLOG(2)<<"Optimization done";
    }
    ////////////////////////////////////////////////

    //drawInterMatches(lf_prev, lf_cur, matches, inliers, "InterMatches");

    for(int i=0; i <inliers.size(); i++){
        if(inliers[i])
            num_inliers++;
    }
    VLOG(2)<<"Ransac Inliers: "<<ransac.inliers_.size()<<endl;
    VLOG(2)<<"Optimization inliers :"<<num_inliers<<endl;
    if(ransac.inliers_.size() > num_inliers){ // this block was not there in imu_integrate
       //drawInterMatches(lf_prev, lf_cur, matches, ransac_inliers, "InterMatches-Ransac");
       // drawInterMatches(lf_prev, lf_cur, matches, inliers, "InterMatches-optimization");
       inliers = ransac_inliers;
       cv::eigen2cv(nonlinear_transformation, curPos);
       T = Mat::eye(4, 4, CV_64F);
       curPos.copyTo(T(cv::Range(0, 3), cv::Range(0, 4)));
       num_inliers = ransac.inliers_.size();
       VLOG(2)<<"ransac has more inliers than optimization. Using ransac result";
    }

    return num_inliers;
}
///////////////////////////////////// Optimization stuff /////////////////////////////////////


void FrontEnd::getDataForGTSAMTriangulation(MultiCameraFrame *lf, int iMatchInd, Cameras &cameras,
                                            gtsam::Point2Vector &measurements, vector<int> &compCamInds,
                                            vector<int> &octaves) {
    IntraMatch iMatch = lf->intraMatches[iMatchInd];
    for (int i1 = 0 ; i1 < lf->num_cams_ ; i1++){
        int ind = iMatch.matchIndex[i1];
        if (ind != -1){
            /// for the LF frame we need to build projection matrices in world frame
            Mat ref_R_cur = camconfig_.R_mats_[i1].t();
            Mat ref_t_cur = -ref_R_cur*camconfig_.t_mats_[i1];
            Mat ref_T_cur = Mat::eye(4, 4, CV_64F);
            ref_R_cur.copyTo(ref_T_cur(cv::Range(0, 3), cv::Range(0, 3)));
            ref_t_cur.copyTo(ref_T_cur(cv::Range(0, 3), cv::Range(3, 4)));
            Mat W_T_cur = lf->pose * ref_T_cur;
            gtsam::Cal3_S2 camK(camconfig_.K_mats_[i1].at<double>(0, 0),
                                camconfig_.K_mats_[i1].at<double>(1, 1), 0,
                                camconfig_.K_mats_[i1].at<double>(0, 2),
                                camconfig_.K_mats_[i1].at<double>(1, 2));
            gtsam::PinholeCamera<gtsam::Cal3_S2> camera1(convertPose3_CV2GTSAM(W_T_cur),camK);
            cameras += camera1;
            measurements += convertPoint2_CV2GTSAM(lf->image_kps_undist[i1][ind]);
            octaves.push_back(lf->image_kps_undist[i1][ind].octave);
            compCamInds.push_back(i1);
        }
    }
}
void FrontEnd::getMapPoints(vector<Point3f>& mapPoints){
    mapPoints.reserve(map->num_lms);
    std::map<int, Landmark*>::iterator it;
    unique_lock<mutex> lock(mMutexPose);
    for(it = map->mapPoints.begin() ; it != map->mapPoints.end() ; ++it) {
        Landmark *l = it->second;
        mapPoints.emplace_back(l->pt3D.at<double>(0,0),l->pt3D.at<double>(1,0),l->pt3D.at<double>(2,0) );
    }
}

double FrontEnd::getSceneDepthStats(MultiCameraFrame* kf){
    vector<double> depthVec;
    Mat Tcw = currentFrame->pose.inv();
    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    Mat tcw = Tcw.col(3).rowRange(0,3);
    for(int i =0; i <kf->lIds.size(); i++) {
        if (kf->lIds[i] != -1) {
            Landmark* lmark = map->getLandmark(kf->lIds[i]);
            Mat pt3d_body = Rcw * lmark->pt3D + tcw;
            depthVec.push_back(pt3d_body.at<double>(2,0));
        }
    }
    sort(depthVec.begin(),depthVec.end());
    double medianDepth = depthVec[(depthVec.size()-1)/2];
    return medianDepth;
}

/////////////////////////////// optimization stufff /////////////////////////////////////////////////
int FrontEnd::triangulateNeighbors(std::map<int, MultiCameraFrame*>& kfMap, std::map<int, vector<DMatch>>& interMatches, vector<double>& depthVec){
    Mat Tcw = currentFrame->pose.inv();
    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    Mat tcw = Tcw.col(3).rowRange(0,3);
    Mat twc = currentFrame->pose.col(3).rowRange(0,3);
    std::map<int, MultiCameraFrame*>::reverse_iterator it;
    int num_triangulated=0;
    VLOG(0) << "Number of neighbors: " << kfMap.size();
    auto startT = high_resolution_clock::now();
    for(it = kfMap.rbegin() ; it != kfMap.rend() ; ++it){
        MultiCameraFrame* kf = it->second;
        // Check if there is enough baseline between the current keyframe and this neighbor
        Mat R_neigh_wc = kf->pose.rowRange(0,3).colRange(0,3);
        Mat t_neigh_wc = kf->pose.col(3).rowRange(0,3);
        double medianDepth = getSceneDepthStats(kf);
        double baseline = norm(twc - t_neigh_wc);
        if(baseline/medianDepth < 0.01)
            continue;
//        VLOG(0) << "Time taken in findInterMatchesBow: "<<duration_cast<milliseconds>(stopT2 - startT2).count();
        // Filter out the matches

        // 1. if the neighbor's matched feature is already assigned a map point
        //2. if the matched feature in the current frame already has a map point
        // if both the features are not alreadu triangulated, checl the epipolar constraints
        vector<bool> inliers;
        vector<Mat> triangulatedPts;
        auto startT3 = high_resolution_clock::now();
        int num = triangulateMatches(kf, currentFrame, interMatches[it->first], inliers, triangulatedPts, depthVec);
        auto stopT3 = high_resolution_clock::now();
        VLOG(0) << "Time taken in triangulateMatches inside triangulateNeighbhors: "<<duration_cast<milliseconds>(stopT3 - startT3).count();
        /// A method which takes the frames, matches and inliers and draws them on images for nov, ov
//        if(num > 0)
//            drawInterMatches(kf, currentFrame, inter_matches, inliers, "Old triangulations");
        // triangulate if all the tests have passed
        VLOG(0)<<"Number of triangulated points old KF "<<kf->frameId<<" current Kf  "<<currentFrame->frameId<<" : "<<num;
        num_triangulated = num + num_triangulated;
        // Make sure the reprojection error is minimum and the cos parallax is withion the bounds

        // create a new map point , insert into the map and the observations
    }
    auto stopT = high_resolution_clock::now();
    VLOG(0) << "Total time taken in triangulate neighbhors for loop: "<<duration_cast<milliseconds>(stopT - startT).count();
    return num_triangulated;
}

void FrontEnd::searchLocalMap2(MultiCameraFrame *prevKF, vector<DMatch> inter_matches_with_landmarks, vector<bool> inliers,
                               vector<DMatch> &inlierAllMapMatches, Mat &refinedPose, vector<int> &alllids, std::map<int, MultiCameraFrame*>& neighbors, std::map<int, vector<DMatch>>& interMatches) {
    // get the set of KeyFrames in which the current inliers have been seen
    std::set<MultiCameraFrame*> kfSet;
    std::map<int, MultiCameraFrame*> kfMap;
    std::set<Landmark*> matchedlmset;
    vector<bool> matchedFeatsCurFrame = vector<bool>(currentFrame->intraMatches.size(),false);
    vector<DMatch> allMapMatches;
    // get the descriptors of the current frame
    vector<Mat> img_desc2;
    cv::Mat img_desc2_mat;
    vector<cv::KeyPoint> kps2;
    auto start0 = high_resolution_clock::now();
    for (auto d :currentFrame->intraMatches){
        img_desc2.push_back(d.matchDesc);
        img_desc2_mat.push_back(d.matchDesc);
        kps2.push_back(KeyPoint(d.uv_ref, 3.0));
    }
    auto stop0 = high_resolution_clock::now();
    VLOG(0) << "Time taken in searchLocalMap2 getting image desc: "<<duration_cast<milliseconds>(stop0 - start0).count();
    // for each feature in current KF, if the vector of lid matches is empty it is not matched yet
    vector<bool> localMapInliers, allMapInliers;
    int old_inlier_lms = 0;
    auto start1 = high_resolution_clock::now();
    for(int i=0; i <inter_matches_with_landmarks.size(); i++){
        if(inliers[i]) {
            DMatch m = inter_matches_with_landmarks[i];
            int l_id = prevKF->lIds[m.queryIdx];
            vector<MultiCameraFrame*> observedKFs = map->getLandmark(l_id)->KFs;
            for(auto& obkf: observedKFs)
                kfMap.insert(pair<int, MultiCameraFrame*>(obkf->kfID , obkf));
            std::copy(observedKFs.begin(), observedKFs.end(), std::inserter(kfSet, kfSet.end()));
            matchedlmset.insert(map->getLandmark(l_id));
            matchedFeatsCurFrame[m.trainIdx] = true; // mark the features which have already been matched.
            alllids.push_back(l_id);
            DMatch oldm;
            oldm.queryIdx = (alllids.size()-1);
            oldm.trainIdx = m.trainIdx;
            allMapMatches.push_back(oldm);
            allMapInliers.push_back(true);
        }
    }
    auto stop1 = high_resolution_clock::now();
    VLOG(0) << "Time taken in searchLocalMap2 getting allMapMatches: "<<duration_cast<milliseconds>(stop1 - start1).count();


    old_inlier_lms = alllids.size();
    // grab the KFset and the lmset that are not already matched
    VLOG(2)<<"Number of older inliers"<<old_inlier_lms;
    VLOG(2)<<"Number of neghbor Keyframes"<<kfSet.size();

    //Get the landmarks that have been observed in the above keyframe set
    Mat Tcw = currentFrame->pose.inv();
    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    Mat tcw = Tcw.col(3).rowRange(0,3);
    std::set<Landmark*> lmSet;
    // Collect the descriptors of all the valid landmnarks
    std::vector<cv::Mat> newlm_vecDescs;
    vector<cv::KeyPoint> kps1;
    vector<int> new_lids;
    vector<vector<int>> lm_projected_cam_ids;
    // construct the component camera poses in world frame
    vector<Mat> W_T_cur_vec;
    for(int camID = 0; camID < currentFrame->num_cams_ ; camID++) {
        Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
        camconfig_.R_mats_[camID].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
        camconfig_.t_mats_[camID].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
        Mat W_T_cur = currentFrame->pose * cur_T_ref.inv();
        W_T_cur_vec.push_back(W_T_cur.clone());
    }

    neighbors = kfMap;
    /// Storing the neighboring keyframes and the matches between them and the current frame
    std::map<int, MultiCameraFrame*>::iterator it;
    std::vector<cv::Mat> unmatechFeaturesVecDescs;
    cv::Mat unmatechFeaturesVecDescsMat;
    vector<int> trueIndices;
    vector<int> kFIdofUnmatchedMatches;
    auto start2 = high_resolution_clock::now();
    for(it = kfMap.begin() ; it != kfMap.end() ; ++it){
        MultiCameraFrame* kf = it->second;
        // find the inter matches between the current frame and the neighboring keyframe and store it in a vector<vector<DMatch>>
        vector<DMatch> inter_matches;
        set<DBoW2::NodeId> words;
        // create a vector to store the feature descriptors of features that do not have landmarks associated with them
        // these descriptors will be used to match with the descriptors of the features in current frame and triangulate them if possible
//        findInterMatchesBow(kf, currentFrame, inter_matches, false);
//        pair<int, vector<DMatch>> p(it->first, inter_matches);
//        interMatches.insert(p);
        for(int i =0; i <kf->lIds.size(); i++){
            if(kf->lIds[i] != -1){
                Landmark* lmark = map->getLandmark(kf->lIds[i]);
                //check if it is already processed
                //check if it is already matched with some feature in this frame
                // If this landmark is already added to the lm->current feature matches move on
                if ( lmSet.find(lmark) == lmSet.end() and matchedlmset.find(lmark) == matchedlmset.end()){

                    lmSet.insert(lmark);
                    // check if it is in the frustum of the current frame
                    Mat pt3d_body = Rcw * lmark->pt3D + tcw;
                    bool acceptlm = false;
                    //Check if the landmark will be seen in the current frame.
                    vector<int> cmids;
                    for(int camID = 0; camID < currentFrame->num_cams_ ; camID++) {
                        Mat pt3d_c = camconfig_.R_mats_[camID] * pt3d_body + camconfig_.t_mats_[camID];
                        double z = pt3d_c.at<double>(2, 0);
                        if (z < 0) // negative depth
                            continue;
                        // if the landmark's normal has a very different view angle dont consider it
                        Mat curDir = lmark->pt3D - W_T_cur_vec[camID].rowRange(0,3).colRange(3,4);
                        if(lmark->normal.dot(curDir) < 0.5*cv::norm(curDir))
                            continue;
                        // project the landmark into the camera and check the bounds
                        Mat tmp = camconfig_.K_mats_[camID] * pt3d_c;
                        tmp = tmp / tmp.at<double>(2, 0);
                        if (tmp.at<double>(0, 0) < 30 || tmp.at<double>(0, 0) > (camconfig_.im_size_.width - 30))
                            continue;
                        if (tmp.at<double>(1, 0) < 30 || tmp.at<double>(1, 0) > (camconfig_.im_size_.height - 30))
                            continue;
                        /// Check the angle of the normal of the landmark and the current camera observations

                        // This is a valid landmark visible in this camera frame
                        // accept lm will remain false if the landmark is not seen in any of the cameras
                        // if it atleast in the frustum of one camera we can accept it
                        cmids.push_back(camID);
                        acceptlm = true;
                    }
                    if(acceptlm){
                        // get the descriptor of this landmark for further matching
                        // for convenienjce we take the descriptor of the latest KF
                        MultiCameraFrame* kf = lmark->KFs.back();
                        IntraMatch im1 = kf->intraMatches[lmark->featInds.back()];
                        newlm_vecDescs.push_back(im1.matchDesc);
                        //pt3dVec.push_back(lm->pt3D);
                        new_lids.push_back(lmark->lId);
                        lm_projected_cam_ids.push_back(cmids);
                        alllids.push_back(lmark->lId);
                        kps1.push_back(KeyPoint(im1.uv_ref, 3.0));


                    }
                }

            }
            else{
                // this feature doesn't have a landmark associated with it
                // add it to the vector of unmatched features
                // for convenienjce we take the descriptor of the latest KF
                IntraMatch im1 = kf->intraMatches[i];
                unmatechFeaturesVecDescs.push_back(im1.matchDesc);
                unmatechFeaturesVecDescsMat.push_back(im1.matchDesc);
                trueIndices.push_back(i);
                kFIdofUnmatchedMatches.push_back(kf->kfID);
            }
        }
    }
    auto stop2 = high_resolution_clock::now();
    VLOG(0) << "Time taken to searchlocalmap2 getting all landmarks for loop : "<<duration_cast<milliseconds>(stop2 - start2).count();

    VLOG(0) << "Number of rows in unmatched features: "<<unmatechFeaturesVecDescsMat.rows;
    auto start3 = high_resolution_clock::now();
    // convert the unmatched features into bag of words
    DBoW2::BowVector unmatchedFeaturesBoW;
    DBoW2::FeatureVector unmatchedFeaturesFeatVec;
    fbow::fBow unmatchedBowVector;
    fbow::fBow2 unmatchedFeatVec;
    fbow::fBow currentFrameBowVector;
    fbow::fBow2 currentFrameFeatVec;
    // convert the current frame into bow vector and feature vector using fbow
    auto start7 = high_resolution_clock::now();
    orb_vocabulary_fbow->transform(img_desc2_mat,4,currentFrameBowVector,currentFrameFeatVec);
//    orb_vocabulary->transform(unmatechFeaturesVecDescs, unmatchedFeaturesBoW, unmatchedFeaturesFeatVec, 4);
    orb_vocabulary_fbow->transform(unmatechFeaturesVecDescsMat,4,unmatchedBowVector,unmatchedFeatVec);
    auto stop7 = high_resolution_clock::now();
    VLOG(0) << "Time taken to transform unmatched features into bow: "<<duration_cast<milliseconds>(stop7 - start7).count();

    // match the unmatched features with features in the current frame
    auto start6 = high_resolution_clock::now();
    vector<unsigned int> unmatchedInd1, unmatchedInd2;
//    InterMatchingBow(unmatchedFeaturesFeatVec, currentFrame->lfFeatVec, unmatechFeaturesVecDescs, img_desc2, unmatchedInd1, unmatchedInd2);
    InterMatchingFBow(unmatchedFeatVec, currentFrameFeatVec, unmatechFeaturesVecDescs, img_desc2, unmatchedInd1, unmatchedInd2);
    auto stop6 = high_resolution_clock::now();

    vector<DMatch> unmatchedMatches;
    auto start8 = high_resolution_clock::now();
    for (int i = 0; i < unmatchedInd1.size(); i++) {
        DMatch m;
        m.queryIdx = trueIndices[unmatchedInd1[i]];
        m.trainIdx = unmatchedInd2[i];
        interMatches[kFIdofUnmatchedMatches[unmatchedInd1[i]]].push_back(m);
    }
    auto stop8 = high_resolution_clock::now();
    VLOG(0) << "Time taken to create unmatched matches : "<<duration_cast<milliseconds>(stop8 - start8).count();
    auto stop3 = high_resolution_clock::now();
    VLOG(0) << "Time taken to match unmatched features with current frame: "<<duration_cast<milliseconds>(stop3 - start3).count();


    VLOG(2)<<"Number of local map points found"<<lmSet.size();

    VLOG(2) << "Number of accepted Landmarks new : " << new_lids.size();

    // transform all the decriptors of the accepted landmarks into BoW v ectors for matching
    DBoW2::BowVector new_lmBoW;
    DBoW2::FeatureVector new_lmFeatVec;

    orb_vocabulary->transform(newlm_vecDescs, new_lmBoW, new_lmFeatVec, 4);


    vector<unsigned int> ind1, ind2;
    //Match the landmark descriptors with the current frame descriptors
    InterMatchingBow(new_lmFeatVec, currentFrame->lfFeatVec, newlm_vecDescs, img_desc2 , ind1, ind2 );
    ///filter matches accoriding to the viewing camera


    //VISUALIZE NEW MAP MATCHES
    VLOG(2)<<"Number of new landmark matches:"<<ind1.size()<<endl;
    assert(ind1.size() == ind2.size());
    vector<vector<vector<DMatch>>> vizMatches(currentFrame->num_cams_,vector<vector<DMatch>>(currentFrame->num_cams_, vector<DMatch>()));
    vector<DMatch> matches;

    auto start4 = high_resolution_clock::now();
    for(int i=0; i<ind1.size(); i++){
        // if the matched feature in current frame is not already matched only then add it as a new match
        // We could additionally check if this is a good match than what has been matched before. but not doing that right now.
        if( !matchedFeatsCurFrame[ind2[i]]){
            DMatch m;
            m.queryIdx = ind1[i];
            m.trainIdx = ind2[i];

            Landmark* lm = map->getLandmark(new_lids[m.queryIdx]);
            MultiCameraFrame* prevKF = map->getLandmark(new_lids[m.queryIdx])->KFs.back();
            //assert (prevKF->frameId == lfFrames.back()->frameId);
            IntraMatch im1 = prevKF->intraMatches[lm->featInds.back()];
            IntraMatch im2 = currentFrame->intraMatches[m.trainIdx];
            int ii=0, ii2=0;
            if (im1.mono){
                ii=0;
                for(int featInd : im1.matchIndex){
                    if(featInd != -1)
                        break;
                    ii++;
                }
                assert(ii != prevKF->num_cams_);

            }

            if(im2.mono){
                for(int featInd : im2.matchIndex){
                    if(featInd != -1)
                        break;
                    ii2++;
                }
                assert(ii2 != currentFrame->num_cams_);
            }
            if(im1.mono and im2.mono) {
                for(auto cid: lm_projected_cam_ids[ind1[i]]) {
                    if(cid == ii2){
                        matches.push_back(m);
                        vizMatches[ii][ii2].push_back(m);
                        DMatch m2;
                        m2.queryIdx = old_inlier_lms + ind1[i];
                        m2.trainIdx = ind2[i];
                        allMapMatches.push_back(m2);
                        allMapInliers.push_back(false);
                    }

                }
            }
        }

    }
    auto stop4 = high_resolution_clock::now();
    VLOG(0) << "Time taken to match new landmarks with current frame: "<<duration_cast<milliseconds>(stop4 - start4).count();

//
//        Mat img_matches_mono;
//        Mat mask = cv::Mat::zeros(matches.size(),1,CV_8UC1);
//        mask = mask *255;
//        std::shared_ptr<MultiCameraFrame> lf_prev = lfFrames.back();
//        for(int i=0; i <currentFrame->num_cams_; i++){
//            for(int j=0; j < currentFrame->num_cams_; j++){
//                Mat img_matches_mono;
//                cv::drawMatches(lf_prev->imgs[i], kps1, currentFrame->imgs[j], kps2, vizMatches[i][j], img_matches_mono, Scalar::all(-1),
//                                Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
//
//                ///draw the inatrmatch 2D points as well
//                ////-- Show detected matches
//                imshow("Inter Matches"+to_string(i)+to_string(j), img_matches_mono);
//                waitKey(0);
//            }
//        }


    auto start5 = high_resolution_clock::now();
    transformation_t poseIn;
    cv::cv2eigen( currentFrame->pose.rowRange(0,3), poseIn);

    OptimizePose(allMapMatches, alllids, allMapInliers, poseIn, refinedPose );

    int num_inliers=0;
    for(int i=0; i <allMapInliers.size(); i++){
        if(allMapInliers[i]){
            num_inliers++;
            inlierAllMapMatches.push_back(allMapMatches[i]);
        }

    }
    if ((num_inliers - old_inlier_lms) > 10){
        currentFrame->pose = refinedPose.clone();
        VLOG(1)<<"Number of new matches in local Map : "<<(allMapMatches.size() - old_inlier_lms);
        VLOG(1)<<"Number of inliers in local Map : "<<(num_inliers - old_inlier_lms);
        VLOG(1)<<"Refined Pose from localMap: "<<refinedPose<<endl;
    }
    else{
        //reszet to older inliers
        inlierAllMapMatches.clear();
        inlierAllMapMatches.insert(inlierAllMapMatches.end(), allMapMatches.begin(), next(allMapMatches.begin()+old_inlier_lms));
        VLOG(0)<<"Not enough inliers in local Map. Pose remains as it is: "<<currentFrame->pose<<endl;
    }
    auto stop5 = high_resolution_clock::now();
    VLOG(0) << "Time taken to optimize pose: "<<duration_cast<milliseconds>(stop5 - start5).count();

}

/////////////////////////////////// optimization stufff /////////////////////////////////////////////////
//int FrontEnd::triangulateNeighbors(std::map<int, MultiCameraFrame*> kfMap, vector<double>& depthVec){
//    Mat Tcw = currentFrame->pose.inv();
//    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    Mat tcw = Tcw.col(3).rowRange(0,3);
//    Mat twc = currentFrame->pose.col(3).rowRange(0,3);
//    std::map<int, MultiCameraFrame*>::reverse_iterator it;
//    int num_triangulated=0;
//    for(it = kfMap.rbegin() ; it != kfMap.rend() ; ++it){
//        MultiCameraFrame* kf = it->second;
//        // Check if there is enough baseline between the current keyframe and this neighbor
//        Mat R_neigh_wc = kf->pose.rowRange(0,3).colRange(0,3);
//        Mat t_neigh_wc = kf->pose.col(3).rowRange(0,3);
//        double medianDepth = getSceneDepthStats(kf);
//        double baseline = norm(twc - t_neigh_wc);
//        if(baseline/medianDepth < 0.01)
//            continue;
//        //Find matches between the current frame and the neighboring keyframe except for the previous one
//        std::vector<DMatch> inter_matches;
//        auto start = high_resolution_clock::now();
//        findInterMatchesBow(kf, currentFrame, inter_matches, false);
//        auto stop = high_resolution_clock::now();
//        auto duration = duration_cast<milliseconds>(stop - start);
//        VLOG(0)<<"Time taken for matching with neighbor KF: "<<duration.count()<<" ms";
//        // Filter out the matches
//
//        // 1. if the neighbor's matched feature is already assigned a map point
//        //2. if the matched feature in the current frame already has a map point
//        // if both the features are not alreadu triangulated, checl the epipolar constraints
//        vector<bool> inliers;
//        vector<Mat> triangulatedPts;
//        int num = triangulateMatches(kf, currentFrame, inter_matches, inliers, triangulatedPts, depthVec);
//        /// A method which takes the frames, matches and inliers and draws them on images for nov, ov
////        if(num > 0)
////            drawInterMatches(kf, currentFrame, inter_matches, inliers, "Old triangulations");
//        // triangulate if all the tests have passed
//        VLOG(2)<<"Number of triangulated points old KF "<<kf->frameId<<" : "<<num;
//        num_triangulated = num + num_triangulated;
//        // Make sure the reprojection error is minimum and the cos parallax is withion the bounds
//
//        // create a new map point , insert into the map and the observations
//    }
//    return num_triangulated;
//}
//
//void FrontEnd::searchLocalMap2(MultiCameraFrame *prevKF, vector<DMatch> inter_matches_with_landmarks, vector<bool> inliers,
//                         vector<DMatch> &inlierAllMapMatches, Mat &refinedPose, vector<int> &alllids, std::map<int, MultiCameraFrame*>& neighbors) {
//    // get the set of KeyFrames in which the current inliers have been seen
//    std::set<MultiCameraFrame*> kfSet;
//    std::map<int, MultiCameraFrame*> kfMap;
//    std::set<Landmark*> matchedlmset;
//    vector<bool> matchedFeatsCurFrame = vector<bool>(currentFrame->intraMatches.size(),false);
//    vector<DMatch> allMapMatches;
//    // for each feature in current KF, if the vector of lid matches is empty it is not matched yet
//    //
//    vector<bool> localMapInliers, allMapInliers;
//    int old_inlier_lms = 0;
//    for(int i=0; i <inter_matches_with_landmarks.size(); i++){
//        if(inliers[i]) {
//            DMatch m = inter_matches_with_landmarks[i];
//            int l_id = prevKF->lIds[m.queryIdx];
//            vector<MultiCameraFrame*> observedKFs = map->getLandmark(l_id)->KFs;
//            for(auto& obkf: observedKFs)
//                kfMap.insert(pair<int,MultiCameraFrame*>(obkf->kfID , obkf));
//            std::copy(observedKFs.begin(), observedKFs.end(), std::inserter(kfSet, kfSet.end()));
//            matchedlmset.insert(map->getLandmark(l_id));
//            matchedFeatsCurFrame[m.trainIdx] = true; // mark the features which have already been matched.
//            alllids.push_back(l_id);
//            DMatch oldm;
//            oldm.queryIdx = (alllids.size()-1);
//            oldm.trainIdx = m.trainIdx;
//            allMapMatches.push_back(oldm);
//            allMapInliers.push_back(true);
//        }
//    }
//
//
//    old_inlier_lms = alllids.size();
//    // grab the KFset and the lmset that are not already matched
//    VLOG(2)<<"Number of older inliers"<<old_inlier_lms;
//    VLOG(2)<<"Number of neghbor Keyframes"<<kfSet.size();
//
//    //Get the landmarks that have been observed in the above keyframe set
//    Mat Tcw = currentFrame->pose.inv();
//    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
//    Mat tcw = Tcw.col(3).rowRange(0,3);
//    std::set<Landmark*> lmSet;
//    // Collect the descriptors of all the valid landmnarks
//    std::vector<cv::Mat> newlm_vecDescs;
//    vector<cv::KeyPoint> kps1;
//    vector<int> new_lids;
//    vector<vector<int>> lm_projected_cam_ids;
//    // construct the component camera poses in world frame
//    vector<Mat> W_T_cur_vec;
//    for(int camID = 0; camID < currentFrame->num_cams_ ; camID++) {
//        Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
//        camconfig_.R_mats_[camID].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
//        camconfig_.t_mats_[camID].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
//        Mat W_T_cur = currentFrame->pose * cur_T_ref.inv();
//        W_T_cur_vec.push_back(W_T_cur.clone());
//    }
//
//    /// Storing the neighboring keyframes
//    neighbors = kfMap;
//    std::map<int, MultiCameraFrame*>::iterator it;
//    for(it = kfMap.begin() ; it != kfMap.end() ; ++it){
//        MultiCameraFrame* kf = it->second;
//        for(int i =0; i <kf->lIds.size(); i++){
//            if(kf->lIds[i] != -1){
//                Landmark* lmark = map->getLandmark(kf->lIds[i]);
//                //check if it is already processed
//                //check if it is already matched with some feature in this frame
//                // If this landmark iints already added to the lm->current feature matches move on
//                if ( lmSet.find(lmark) == lmSet.end() and matchedlmset.find(lmark) == matchedlmset.end()){
//
//                    lmSet.insert(lmark);
//                    // check if it is in the frustum of the current frame
//                    Mat pt3d_body = Rcw * lmark->pt3D + tcw;
//                    bool acceptlm = false;
//                    //Check if the landmark will be seen in the current frame.
//                    vector<int> cmids;
//                    for(int camID = 0; camID < currentFrame->num_cams_ ; camID++) {
//                        Mat pt3d_c = camconfig_.R_mats_[camID] * pt3d_body + camconfig_.t_mats_[camID];
//                        double z = pt3d_c.at<double>(2, 0);
//                        if (z < 0) // negative depth
//                            continue;
//                        // if the landmark's normal has a very different view angle dont consider it
//                        Mat curDir = lmark->pt3D - W_T_cur_vec[camID].rowRange(0,3).colRange(3,4);
//                        if(lmark->normal.dot(curDir) < 0.5*cv::norm(curDir))
//                            continue;
//                        // project the landmark into the camera and check the bounds
//                        Mat tmp = camconfig_.K_mats_[camID] * pt3d_c;
//                        tmp = tmp / tmp.at<double>(2, 0);
//                        if (tmp.at<double>(0, 0) < 30 || tmp.at<double>(0, 0) > (camconfig_.im_size_.width - 30))
//                            continue;
//                        if (tmp.at<double>(1, 0) < 30 || tmp.at<double>(1, 0) > (camconfig_.im_size_.height - 30))
//                            continue;
//                        /// Check the angle of the normal of the landmark and the current camera observations
//
//                        // This is a valid landmark visible in this camera frame
//                        // accept lm will remain false if the landmark is not seen in any of the cameras
//                        // if it atleast in the frustum of one camera we can accept it
//                        cmids.push_back(camID);
//                        acceptlm = true;
//                    }
//                    if(acceptlm){
//                        // get the descriptor of this landmark for further matching
//                        // for convenienjce we take the descriptor of the latest KF
//                        MultiCameraFrame* kf = lmark->KFs.back();
//                        IntraMatch im1 = kf->intraMatches[lmark->featInds.back()];
//                        newlm_vecDescs.push_back(im1.matchDesc);
//                        //pt3dVec.push_back(lm->pt3D);
//                        new_lids.push_back(lmark->lId);
//                        lm_projected_cam_ids.push_back(cmids);
//                        alllids.push_back(lmark->lId);
//                        kps1.push_back(KeyPoint(im1.uv_ref, 3.0));
//
//                    }
//                }
//
//            }
//        }
//    }
//    VLOG(2)<<"Number of local map points found"<<lmSet.size();
//
//    VLOG(2) << "Number of accepted Landmarks new : " << new_lids.size();
//
//    // transform all the decriptors of the accepted landmarks into BoW vectors for matching
//    DBoW2::BowVector new_lmBoW;
//    DBoW2::FeatureVector new_lmFeatVec;
//
//    orb_vocabulary->transform(newlm_vecDescs, new_lmBoW, new_lmFeatVec, 4);
//    vector<Mat> img_desc2;
//    vector<cv::KeyPoint> kps2;
//    for (auto d :currentFrame->intraMatches){
//        img_desc2.push_back(d.matchDesc);
//        kps2.push_back(KeyPoint(d.uv_ref, 3.0));
//    }
//
//    vector<unsigned int> ind1, ind2;
//    //Match the landmark descriptors with the current frame descriptors
//    auto start = std::chrono::high_resolution_clock::now();
//    InterMatchingBow(new_lmFeatVec, currentFrame->lfFeatVec, newlm_vecDescs, img_desc2 , ind1, ind2 );
//    auto finish = std::chrono::high_resolution_clock::now();
//    auto elapsed = duration_cast<milliseconds>(finish - start);
//    VLOG(0) << "Elapsed time for matching lm descs with curr frame descs: " << elapsed.count();
//    ///filter matches accoriding to the viewing camera
//
//
//    //VISUALIZE NEW MAP MATCHES
//                    VLOG(2)<<"Number of new landmark matches:"<<ind1.size()<<endl;
//    assert(ind1.size() == ind2.size());
//    vector<vector<vector<DMatch>>> vizMatches(currentFrame->num_cams_,vector<vector<DMatch>>(currentFrame->num_cams_, vector<DMatch>()));
//    vector<DMatch> matches;
//
//    for(int i=0; i<ind1.size(); i++){
//        // if the matched feature in current frame is not already matched only then add it as a new match
//        // We could additionally check if this is a good match than what has been matched before. but not doing that right now.
//        if( !matchedFeatsCurFrame[ind2[i]]){
//            DMatch m;
//            m.queryIdx = ind1[i];
//            m.trainIdx = ind2[i];
//
//            Landmark* lm = map->getLandmark(new_lids[m.queryIdx]);
//            MultiCameraFrame* prevKF = map->getLandmark(new_lids[m.queryIdx])->KFs.back();
//            //assert (prevKF->frameId == lfFrames.back()->frameId);
//            IntraMatch im1 = prevKF->intraMatches[lm->featInds.back()];
//            IntraMatch im2 = currentFrame->intraMatches[m.trainIdx];
//            int ii=0, ii2=0;
//            if (im1.mono){
//                ii=0;
//                for(int featInd : im1.matchIndex){
//                    if(featInd != -1)
//                        break;
//                    ii++;
//                }
//                assert(ii != prevKF->num_cams_);
//
//            }
//
//            if(im2.mono){
//                for(int featInd : im2.matchIndex){
//                    if(featInd != -1)
//                        break;
//                    ii2++;
//                }
//                assert(ii2 != currentFrame->num_cams_);
//            }
//            if(im1.mono and im2.mono) {
//                for(auto cid: lm_projected_cam_ids[ind1[i]]) {
//                    if(cid == ii2){
//                        matches.push_back(m);
//                        vizMatches[ii][ii2].push_back(m);
//                        DMatch m2;
//                        m2.queryIdx = old_inlier_lms + ind1[i];
//                        m2.trainIdx = ind2[i];
//                        allMapMatches.push_back(m2);
//                        allMapInliers.push_back(false);
//                    }
//
//                }
//            }
//        }
//
//    }
//
////
////        Mat img_matches_mono;
////        Mat mask = cv::Mat::zeros(matches.size(),1,CV_8UC1);
////        mask = mask *255;
////        MultiCameraFrame* lf_prev = lfFrames.back();
////        for(int i=0; i <currentFrame->num_cams_; i++){
////            for(int j=0; j < currentFrame->num_cams_; j++){
////                Mat img_matches_mono;
////                cv::drawMatches(lf_prev->imgs[i], kps1, currentFrame->imgs[j], kps2, vizMatches[i][j], img_matches_mono, Scalar::all(-1),
////                                Scalar::all(-1),vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
////
////                ///draw the inatrmatch 2D points as well
////                ////-- Show detected matches
////                imshow("Inter Matches"+to_string(i)+to_string(j), img_matches_mono);
////                waitKey(0);
////            }
////        }
//
//
//    transformation_t poseIn;
//    cv::cv2eigen( currentFrame->pose.rowRange(0,3), poseIn);
//
//    OptimizePose(allMapMatches, alllids, allMapInliers, poseIn, refinedPose );
//
//    int num_inliers=0;
//    for(int i=0; i <allMapInliers.size(); i++){
//        if(allMapInliers[i]){
//            num_inliers++;
//            inlierAllMapMatches.push_back(allMapMatches[i]);
//        }
//
//    }
//    if ((num_inliers - old_inlier_lms) > 10){
//        currentFrame->pose = refinedPose.clone();
//        VLOG(1)<<"Number of new matches in local Map : "<<(allMapMatches.size() - old_inlier_lms);
//        VLOG(1)<<"Number of inliers in local Map : "<<(num_inliers - old_inlier_lms);
//        VLOG(1)<<"Refined Pose from localMap: "<<refinedPose<<endl;
//    }
//    else{
//        //reszet to older inliers
//        inlierAllMapMatches.clear();
//        inlierAllMapMatches.insert(inlierAllMapMatches.end(), allMapMatches.begin(), next(allMapMatches.begin()+old_inlier_lms));
//        VLOG(1)<<"Not enough inliers in local Map. Pose remains as it is: "<<currentFrame->pose<<endl;
//    }
//
//}

//todo: this function is new - post merge
int FrontEnd::triangulateMatches(MultiCameraFrame* prev_kf, MultiCameraFrame* curFrame, vector<DMatch> inter_matches, vector<bool>& inliers,
                                 vector<Mat>& triangulatedPts, vector<double>& depthVec){
    vector<Mat> ProjMats1, ProjMats2;
    for(int c_ind=0; c_ind < curFrame->num_cams_ ; c_ind++){
        Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
        camconfig_.R_mats_[c_ind].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
        camconfig_.t_mats_[c_ind].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
        Mat cur_T_W = cur_T_ref * prev_kf->pose.inv();
        //Mat P1 = cv::Mat_<double>(cur_T_W(cv::Range(0,3), cv::Range(0,4)));
        ProjMats1.push_back(cur_T_W.clone());
    }
    for(int c_ind=0; c_ind < curFrame->num_cams_ ; c_ind++){
        Mat cur_T_ref = Mat::eye(4, 4, CV_64F);
        camconfig_.R_mats_[c_ind].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
        camconfig_.t_mats_[c_ind].copyTo(cur_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
        Mat cur_T_W = cur_T_ref * curFrame->pose.inv();
       // Mat P1 = cv::Mat_<double>(cur_T_W(cv::Range(0,3), cv::Range(0,4)));
        ProjMats2.push_back(cur_T_W.clone());
    }
//    VLOG(2)<<"Prev KF pose: "<<prev_kf->pose;
//    VLOG(2)<<"Current KF pose: "<<curFrame->pose;
//    VLOG(2)<<"Current KF pose: "<<currentFrame->pose;
    /// Filter matched based on epipolar constraint ///ASUMOTION RIGHT NOWE IS that it is monocular or non-overlapping
//    for(int i =0; i < curFrame->num_cams_ ; i++){
//        int c1=i;
//        int c2=i;
//        Mat cam_T_ref = Mat::eye(4, 4, CV_64F);
//        camconfig_.R_mats_[i].copyTo(cam_T_ref(cv::Range(0, 3), cv::Range(0, 3)));
//        camconfig_.t_mats_[i].copyTo(cam_T_ref(cv::Range(0, 3), cv::Range(3, 4)));
//        Mat cam_T_W1 = cam_T_ref * prev_kf->pose.inv();
//
//        Mat cam_T_W2 = cam_T_ref * currentFrame->pose.inv();
//
//        Mat Tji = cam_T_W2 * cam_T_W1.inv();
//        Mat t_ji_x =  (Mat_<double>(3,3) << 0, -Tji.at<double>(2,3), Tji.at<double>(1,3),
//                Tji.at<double>(2,3), 0, -Tji.at<double>(0,3),
//                -Tji.at<double>(1,3), Tji.at<double>(0,3), 0);
//        Mat F21 = camconfig_.K_mats_[c2].t().inv() * t_ji_x * Tji(cv::Range(0, 3), cv::Range(0, 3)) * camconfig_.K_mats_[c1].inv();
//        F21s.push_back(F21.clone());
//    }

  ////////////// EPIPolar constraimts///////////////
    int num_triangulated=0;
    int num_initial_inliers = 0;
    vector<double> parllaxVec;
    //Do a triangulation with non linear refinement of the inliers using both the LF frames
    int numt1=0, numt2=0, numt3=0;
    double avg_depth=0.0, median_scene_depth = 0.0;
    triangulatedPts = vector<Mat>(inter_matches.size());
    inliers = vector<bool>(inter_matches.size(), true);

    for (int i = 0 ; i < inter_matches.size(); i++){
//        /// if the current match is an outlier move on to next
//        if(!inliers[i])
//            continue;
        ///Get the intramatches and their corresponding 2D observations
        DMatch m = inter_matches[i];
        // 1. if the neighbor's matched feature is already assigned a map point
        int l_id_neigh = prev_kf->lIds[m.queryIdx];

        //2. if the matched feature in the current frame already has a map point
        int l_id_cur = currentFrame->lIds[m.trainIdx];
        // if both the features are not alreadu triangulated, checl the epipolar constraints
        if(l_id_neigh != -1 or l_id_cur != -1) {
            inliers[i] = false;
            continue;
        }

        vector<Mat> PJs, centres;
        vector<int> view_inds;
        vector<Point2f> kps;
        int num_views =0;
        std::vector<Mat_<double> >  xs;
        vector<int> kp_octaves;
        Mat o1, o2 ;
        //extract the 3D-2D correspondences in world coordintes
        get3D_2DCorrs(prev_kf, m.queryIdx, prev_kf->pose, ProjMats1, PJs, xs, view_inds, kps,kp_octaves);
        o1 = -1 * PJs[0](cv::Range(0, 3), cv::Range(0, 3)).t() * PJs[0](cv::Range(0, 3), cv::Range(3, 4));
        int nextI = PJs.size();
        get3D_2DCorrs(currentFrame, m.trainIdx, curFrame->pose, ProjMats2, PJs, xs, view_inds, kps, kp_octaves);
        o2 = -1 * PJs[nextI](cv::Range(0, 3), cv::Range(0, 3)).t() * PJs[nextI](cv::Range(0, 3), cv::Range(3, 4));

        /// Filter matched based on epipolar constraint ///ASUMOTION RIGHT NOWE IS that it is monocular
        Mat Tji = ProjMats2[0] * ProjMats1[0].inv();
        Mat t_ji_x =  (Mat_<double>(3,3) << 0, -Tji.at<double>(2,3), Tji.at<double>(1,3),
                Tji.at<double>(2,3), 0, -Tji.at<double>(0,3),
                -Tji.at<double>(1,3), Tji.at<double>(0,3), 0);
        Mat F21 = camconfig_.K_mats_[view_inds[nextI]].t().inv() * t_ji_x * Tji(cv::Range(0, 3), cv::Range(0, 3)) * camconfig_.K_mats_[view_inds[0]].inv();

        // Epipolar line in image l2 = F21 * kp1
        float a = kps[0].x * F21.at<double>(0,0)+ kps[0].y * F21.at<double>(0,1)+F21.at<double>(0,2);
        float b = kps[0].x * F21.at<double>(1,0)+ kps[0].y * F21.at<double>(1,1)+F21.at<double>(1,2);
        float c = kps[0].x * F21.at<double>(2,0)+ kps[0].y * F21.at<double>(2,1)+F21.at<double>(2,2);

//                float den = a*a+b*b;
//                den = den ? 1./std::sqrt(den) : 1.;
//                a *= den; b *= den; c *= den;
//                den = a*a + b*b;

        float num = a * kps[nextI].x + b * kps[nextI].y + c;
        float den = a*a+b*b;

        if(den == 0){
            VLOG(3)<<"EPIPOLAR CONSTRAINT FAILED zero";
            inliers[i] = false;
            continue;
        }
        else{
            if(num*num/den >= 4.0){
                VLOG(3)<<"EPIPOLAR CONSTRAINT FAILED: "<<num*num/den;
                inliers[i] = false;
                continue;
            }
            else
                VLOG(3)<<"EPIPOLAR CONSTRAINT PASSED";
        }

        /////////////// Epipolar COnstraint


        //todo: Triangulation methods implementation
        ///// Triangulation using  opencv sfm and normalized coordinates///////////
        cv::Mat pt3d;
        mcslam::geom::triangulatePoints(xs, PJs, pt3d);
        // check the reprojection errors of the points and the parallalx angle
        int jj=0;

        for (auto P : PJs){

            int cam_ind = view_inds[jj];
            //cout<<"landmark camrea view: "<<cam_ind<<endl;
            Mat p3d_conv = P(cv::Range(0, 3), cv::Range(0, 3)) * pt3d +  P(cv::Range(0, 3), cv::Range(3, 4));
            if(p3d_conv.at<double>(2,0) < 0){
                inliers[i] = false;
                break;
            }
//                    VLOG(2)<<"3D point: "<< pt3d;
//                    VLOG(2)<<"3D point converted: "<<p3d_conv;
//                    VLOG(2)<<"Proj Mat: "<<P;
//                    Mat tmp = Mat::eye(4, 4, CV_64F);
//                    P.copyTo(tmp(cv::Range(0, 3), cv::Range(0, 4)));
//                    VLOG(2)<<"Proj inverse this shuld be equal to pose:"<<tmp.inv();
//                    VLOG(2)<<"Kp: "<<kps[jj];

            Mat projected = camconfig_.K_mats_[cam_ind] * p3d_conv;
            double expected_x = projected.at<double>(0,0) / projected.at<double>(2,0);
            double expected_y = projected.at<double>(1,0) / projected.at<double>(2,0);
            double err = (expected_x - kps[jj].x)*(expected_x - kps[jj].x)+(expected_y - kps[jj].y)*(expected_y - kps[jj].y);
            err = err*orBextractor->GetInverseScaleSigmaSquares()[ kp_octaves[jj]];
            jj++;
            if(err>5.991){
                inliers[i] = false;
                break;
            }

        }

        // cout<<"--------------------"<<endl;
        if(inliers[i]){
            // Check parallax
            cv::Mat normal1 = pt3d - o1;
            double dist1 = cv::norm(normal1);

            cv::Mat normal2 = pt3d - o2;
            double dist2 = cv::norm(normal2);

            double cosParallax = normal1.dot(normal2)/(dist1*dist2);

            if(cosParallax < 0.99998 and cosParallax > 0.5) { //
                //drawInterMatch(prev_kf, currentFrame, prev_kf->intraMatches[m.queryIdx],currentFrame->intraMatches[m.trainIdx], "inlier triangulation");

                triangulatedPts[i] = pt3d;
                num_triangulated++;
                parllaxVec.push_back(cosParallax);
                avg_depth += dist2;
                depthVec.push_back(dist2);

                int l_id = map->insertLandmark( pt3d, prev_kf, m.queryIdx, prev_kf->intraMatches[m.queryIdx].uv_ref);
                Landmark* lm = map->getLandmark(l_id);
                lm->addLfFrame(currentFrame, m.trainIdx, currentFrame->intraMatches[m.trainIdx].uv_ref);
                prev_kf->lIds[m.queryIdx] = l_id;
                currentFrame->lIds[m.trainIdx] = l_id;
                VLOG(3)<<"Inserted landmark at LID: "<<l_id<<" -- "<<pt3d.at<double>(0,0)<<","<<pt3d.at<double>(1,0)<<","<<pt3d.at<double>(2,0);
            }

        }

    }
//    sort(depthVec.begin(),depthVec.end());
//    size_t idx = depthVec.size()/2;
//    median_scene_depth = depthVec[idx];
//    VLOG(2)<<"Number of triangulated points old: "<<num_triangulated;
//    VLOG(2)<<"Average depth of the triangulated points Old: "<<avg_depth/num_triangulated;
//    VLOG(2)<<"Median depth of the triangulated points old : "<<median_scene_depth;
    return num_triangulated;
}


bool FrontEnd::trackFrame(){
    bool viz_track = false;
    log_KF_ =0;
    Mat all;
    //////////////  This //////////////////////////////////////
    all.create(currentFrame->imgs[0].rows, currentFrame->imgs[0].cols * currentFrame->num_cams_, CV_8UC3);
    for(int i=0; i < camconfig_.num_cams_ ; i++){
        Mat imgBGR;
        cvtColor(currentFrame->imgs[i],imgBGR , COLOR_GRAY2BGR);
        imgBGR.copyTo(all.colRange(camconfig_.im_size_.width*i, camconfig_.im_size_.width*(i+1)));

    }
    /// grab the current frame
    MultiCameraFrame* lf_cur = currentFrame;

    /// Initialization ??
    if(initialized_ == NOT_INITIALIZED || initialized_ == REINITIALIZING){
        initialization();
        return true;
    }

    if(tracking_failure_){
        if( num_frames_since_tracking_failure <= num_trials_to_track){
            num_frames_since_tracking_failure++;
        }
        else{
            num_frames_since_tracking_failure =0;
            // we need to re-initiaze the vision part;
            initialized_ = REINITIALIZING;
            return false;
        }

    }

    /// After initialization we need to track each incoming frame
    /// WRT the last keyframe.
    /// grab the last keyframe
    MultiCameraFrame* prev_KF;
    for (int i = lfFrames.size() - 1; i >= 0; i --) {
        prev_KF = lfFrames[i];
        //the prev frame could be a dummy kf
        if(prev_KF->dummy_kf){
            continue;
        }
        else{
            break;
        }
    }
//    prev_KF = lfFrames.back();

    // VLOG(1)<<"------------------- NEW FRAME --------------------"<<endl;
    VLOG(1)<<"Number of landmarks: "<<prev_KF->numTrackedLMs<<endl;

    auto startT = high_resolution_clock::now();

    /// Find inter matches between current frame and the last keyframe
    std::vector<DMatch> inter_matches;
    opengv::points_t interMatch_3d_1, interMatch_3d_2;

    switch (interMatch_) {
        case BF_MATCH:
            findInterMatches(prev_KF, currentFrame, inter_matches, false);
            break;
        case BoW_MATCH:
            // Todo: We can directky output a mask vector for matches with landmarks
            findInterMatchesBow(prev_KF, currentFrame, inter_matches, false);
            break;
    }

    VLOG(0) <<"Number of intermatches b/w prev and curr kf: "<<inter_matches.size()<<endl;
    auto stop_feat_match = high_resolution_clock::now();
    auto duration_feat_match = duration_cast<milliseconds>(stop_feat_match - startT);
    VLOG(1)<<"Total time taken for matching features: "<<duration_feat_match.count()<<endl;
    if(inter_matches.size() < 60)
    {
        VLOG(0)<<"Not Enough Matches between frames" << endl;
        deleteCurrentFrame();
        tracking_failure_ = true;
        return false;
    }

    //testing
    int inter_matches_size = inter_matches.size();

    /// We will get two sets of matches
    /// 1) already tracked landmarks 2) new matches which are tracked in the current frame
    ///extract the matches associated with tracked landmarks
    std::vector<DMatch> inter_matches_with_landmarks, new_inter_matches;
    for(auto m : inter_matches){
        ///check if the feature corresponding to reference keyframe has a landmark
        int l_id = prev_KF->lIds[m.queryIdx];
        if (l_id == -1){
            new_inter_matches.push_back(m);
        }
        else{
            inter_matches_with_landmarks.push_back(m);
        }

    }

    Mat T = Mat::eye(4, 4, CV_64F);

    /// Get the landmarks corresponding to th inter matches (maintain vec<lm*>, vec<lmid>)
    /// align the current frame to these landmarks

    T = Mat::eye(4, 4, CV_64F);
    /// find pose of the light field frame using 3D-3d PC alignment
    vector<bool> inliers;
    //VLOG(2)<<"trackFrame before estimate pose"<<endl;
    // startT = high_resolution_clock::now();
    int num_inliers = 0;
    int num_inliers_refKF=0;

    VLOG(2) << "Number of inter_matches_with_landmarks : "<<inter_matches_with_landmarks.size()<<endl;
    num_inliers = estimatePoseLF(prev_KF, lf_cur, inter_matches_with_landmarks, inliers, T, false );


    // auto stopT = high_resolution_clock::now();
    // auto duration = duration_cast<milliseconds>(stopT - startT);
    // VLOG(2)<<"Time taken for pose estimation : "<<duration.count()<<endl;

    // VLOG(2) << "Total Number of intermatches "<<inter_matches.size()<<endl;
    // VLOG(2) << "Number of inter_matches_with_landmarks : "<<inter_matches_with_landmarks.size()<<endl;
    // VLOG(2) << "Number of new intermatches : "<<new_inter_matches.size()<<endl;
    // VLOG(1) << "Number of tracked landmarks : " << num_inliers << ","<<(float)num_inliers/prev_KF->numTrackedLMs<<endl;
    VLOG(1) << "Number of tracked landmarks : " << num_inliers <<endl;
    log_num_intramatches_ = currentFrame->intraMatches.size();
    log_num_matches_ = inter_matches.size() ;
    log_num_lm_matches_ = inter_matches_with_landmarks.size();
    log_num_tracked_lms_ = num_inliers;
    log_num_new_matches_ = new_inter_matches.size();


    int pp=0;
    double tracking_parallax =0.0;
    vector<double> tracking_depthVec;
    Mat Tcw = currentFrame->pose.inv();
    Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    Mat tcw = Tcw.col(3).rowRange(0,3);
    for(auto& m : inter_matches_with_landmarks){
        if(inliers[pp]){
            IntraMatch im2 = lf_cur->intraMatches[m.trainIdx];

            int l_id = prev_KF->lIds[m.queryIdx];
            Mat pt3d= map->getLandmark(l_id)->pt3D;
            Mat pt3d_body = Rcw * pt3d + tcw;
            tracking_depthVec.push_back(pt3d_body.at<double>(2,0));

            Point2f p2;
            int camIndCur=0;
            Scalar cc;
            if(im2.mono){
                cc=  Scalar(0,255,0);
            }
            else{

                cc=Scalar(0,0,255);
            }
            for(int featInd: im2.matchIndex){
                if(featInd != -1){
                    p2 = lf_cur->image_kps_undist[camIndCur][featInd].pt;
                    cv::circle(all,p2+Point2f(camIndCur * camconfig_.im_size_.width, 0),4,cc, 2);
                }
                camIndCur++;
            }

        }
        pp++;
    }
    cv::resize(all,all,cv::Size(camconfig_.im_size_.width * lf_cur->num_cams_/2, camconfig_.im_size_.height/2));
    cv::imshow("images", all);
    //if(currentFrame->frameId < 5)
   //     cv::waitKey(0);
   // else
        // cv::waitKey(4);
    VLOG(0)<<"Number of inliers in the pose estimation: "<<num_inliers<<endl;
    if (num_inliers < 10){
        deleteCurrentFrame();
        tracking_failure_ = true;
        return false;
    }
    // we found enough inliers/reset tracking failure
    tracking_failure_ = false;
    num_frames_since_tracking_failure = 0;

    /// we are directly localizing WRT the landmraks in global frame.
    /// no need of transform chaining

    currentFrame->pose = T.clone();

    // VLOG(2) << "Initial Pose estimate "<< currentFrame->pose <<endl;

    Mat relativePose = prev_KF->pose.col(3) - T.col(3);
    double baseline = norm(relativePose);
    Mat relative_transformation = prev_KF->pose.inv() * T;

    sort(tracking_depthVec.begin(),tracking_depthVec.end());
    size_t idx = tracking_depthVec.size()/2;
    double med_tracked_depth = tracking_depthVec[idx];

    VLOG(0)<<"Ratio Baseline to median depth: "<<baseline/med_tracked_depth;

    double gamma = abs(atan2(relative_transformation.at<double>(2,1), relative_transformation.at<double>(2,2))) *  (180.0/3.141592653589793238463);

    double rotation = abs(atan2(-1 * relative_transformation.at<double>(2,0),
                                sqrt(pow(relative_transformation.at<double>(2,1), 2) +
                                     pow(relative_transformation.at<double>(2,2), 2)))) *  (180.0/3.141592653589793238463);
    // VLOG(2)<<" translation: "<<baseline<<endl;
    // VLOG(2)<<" rot angle: "<<rotation<<", gamma: "<<gamma<<endl;
    /////////////////// Check if this has a potential to be inserted as a keyframe ////////////////////////////
    /// 1. number of tracked points
    /// 2. number of tracked close points

    bool cond1 = (baseline >= kf_translation_threshold || rotation >= kf_rotation_threshold);
    bool cond2 = !cond1 and  (float)num_inliers/prev_KF->numTrackedLMs < 0.4;
    if(!cond1 and !cond2){
        VLOG(0)<<" Deleting frame due to less motion and more inliers. No scope for keyframe";
        deleteCurrentFrame();
        return false;
    }

    vector<DMatch> allMapMatchesInliers;
    vector<int> allLids;
    Mat refinedPose;
    startT = high_resolution_clock::now();
    ///if the inliers are too little, tru to localize with respect to the local map

    // todo: githb : new addition post merge
    std::map<int, MultiCameraFrame*> neighbors;
    std::map<int, vector<DMatch>> inter_matches_with_landmarks_map;
//    searchLocalMap2(prev_KF, inter_matches_with_landmarks, inliers, allMapMatchesInliers, refinedPose, allLids,neighbors);
    searchLocalMap2(prev_KF, inter_matches_with_landmarks, inliers, allMapMatchesInliers, refinedPose, allLids,neighbors, inter_matches_with_landmarks_map);
    auto stopT = high_resolution_clock::now();
    auto durationS = duration_cast<milliseconds>(stopT - startT);
    VLOG(0)<<"Time taken to run search local Map and optimize :"<<durationS.count()<<endl;

    // check: currentframe->pose is being set properly inside the searchlocalmap2 function. we shouldnt uncomment here
    // currentFrame->pose = refinedPose.clone();
    ///////////////// Get the local map points from neighboring keyframes and refine the matches as well as pose estimates//////////////////////

    //Mat all;
    if(viz_track){
        vector<cv::KeyPoint> kps1, kps2;
        int ind = 0;
        for (auto &d : prev_KF->intraMatches) {

            kps1.push_back(KeyPoint(d.uv_ref, 3.0));
            ind++;
        }
        ind = 0;
        for (auto &d : lf_cur->intraMatches) {

            kps2.push_back(KeyPoint(d.uv_ref, 3.0));
            ind++;
        }

        ///////////////////////////////////////////////////////////
        //////////////  This //////////////////////////////////////
        ///////////////////////////////////////////////////////////
        all.create(prev_KF->imgs[0].rows, prev_KF->imgs[0].cols * prev_KF->num_cams_, CV_8UC3);
        for(int i=0; i < camconfig_.num_cams_ ; i++){
            Mat imgBGR;
            cvtColor(lf_cur->imgs[i],imgBGR , COLOR_GRAY2BGR);
            imgBGR.copyTo(all.colRange(camconfig_.im_size_.width*i, camconfig_.im_size_.width*(i+1)));

        }


        for(auto& m : new_inter_matches){
            IntraMatch im1 = prev_KF->intraMatches[m.queryIdx];
            IntraMatch im2 = lf_cur->intraMatches[m.trainIdx];
            Point2f p1, p2;
            int camIndCur=0;
            p1 = Point2f (0,0);
            if(im2.mono){
                for(int featInd: im2.matchIndex){
                    if(featInd != -1)
                        break;
                    camIndCur++;
                }

            }
            //// In this visualization P2 is always drawn in the current component camera
            //// if the p1 is also seen in the same component camera the arrow is drawn in that comp camera
            //// if p1 and p2 are both intra matches it is drawn in the reference camera
            //// if
            p2 = im2.uv_ref;
            ///check if the im1's cam Ind matches current intra match cam
            if(im1.matchIndex[camIndCur] != -1){
                /// if the cam Indices of both the features match. We just draw the arrow
                p1 = prev_KF->image_kps_undist[camIndCur][im1.matchIndex[camIndCur]].pt;
            }
            else if(!im1.mono and camIndCur == 0){
                /// if the cam indices do not match, but it is because the prev feature is an intra match and the current feature's ref cam is 0
                /// draw the arrow in 0th camera
                p1 = im1.uv_ref;
            }

            //cv::circle(all,p2+Point2f(camIndCur * camconfig_.im_size_.width, 0),3,Scalar(0,170,30), 2);
            //if(p1.x !=0 and p1.y != 0)
            //    cv::arrowedLine(all, p1+ Point2f(camIndCur * camconfig_.im_size_.width, 0), p2+Point2f(camIndCur * camconfig_.im_size_.width, 0), Scalar(0,170,30), 2);
        }

        for (int i =0; i < allMapMatchesInliers.size(); i++) {
            DMatch m = allMapMatchesInliers[i];
            IntraMatch im2 = currentFrame->intraMatches[m.trainIdx];

            for(int ci=0; ci < camconfig_.num_cams_ ; ci++){
                if(im2.matchIndex[ci] != -1){
                    Point2f p2 = lf_cur->image_kps_undist[ci][im2.matchIndex[ci]].pt;
                    cv::circle(all,p2+Point2f(ci * camconfig_.im_size_.width, 0),3,Scalar(0,170,10), 2);
                }
            }

        }
    }
    cv::resize(all,all,cv::Size(camconfig_.im_size_.width * lf_cur->num_cams_/2, camconfig_.im_size_.height/2));
    cv::imshow("images", all);
    // if(currentFrame->frameId < 5)
    //     cv::waitKey(0);
    // else
    //     cv::waitKey(0);

    ///we can get the inliers (featids in current frame) through reprojection error check as well. Right now
    /// we are only sticking with 3D-3D error within PCAlignment method

    // check the percentage of intramatches in the last frame tracked
    // if the tracked fall below 25% of intramatches
    // compare the inliers in the currentframe WRT the Map points in last keyframe
    int num_triangulated=0;
    int prevlms=prev_KF->numTrackedLMs;
    bool ret = false;

    relativePose = prev_KF->pose.col(3) - T.col(3);
    baseline = norm(relativePose);
    relative_transformation = prev_KF->pose.inv() * T;


    gamma = abs(atan2(relative_transformation.at<double>(2,1), relative_transformation.at<double>(2,2))) *  (180.0/3.141592653589793238463);

    rotation = abs(atan2(-1 * relative_transformation.at<double>(2,0),
                                sqrt(pow(relative_transformation.at<double>(2,1), 2) +
                                     pow(relative_transformation.at<double>(2,2), 2)))) *  (180.0/3.141592653589793238463);
    // VLOG(2)<<" translation: "<<baseline<<endl;
    // VLOG(2)<<" rot angle: "<<rotation<<", gamma: "<<gamma<<endl;

    if (num_inliers == 0){
        VLOG(0)<<"Inliers 0 after local map otimization. Should not come here";
        deleteCurrentFrame();
        tracking_failure_ = true;
        return ret;
    }

    //std::vector<char> inliers_triangulated, outliers_triangulated;
    num_inliers_refKF = num_inliers;
    /// Conditions for key frame insertion
    cond1 = (float)num_inliers/prev_KF->numTrackedLMs <= 0.4 && (baseline >= kf_translation_threshold || rotation >= kf_rotation_threshold);

    cond2 = !cond1 and  num_inliers < 0.3 * prev_KF->numTrackedLMs; //cond2 = !cond1 and  num_inliers < 35; for imu_integrate
    num_inliers = allMapMatchesInliers.size();
    bool cond3 = baseline/med_tracked_depth > 0.01;
    //LOGGING
    VLOG(2)<<"cond1: "<<cond1<<" cond2: "<<cond2<<" cond3 : "<<cond3;
    vector<int> n_rays (currentFrame->num_cams_, 0);
    if(cond1 or cond2  ) {

        /// if num of inliers falls below a threshold insert the new keyframe
        /// - assign landmarks to features, optimize R|t and
        /// else we have enough tracked points. Save the current pose and move on to next frame
        //Add additional local Map matches found
        for (int i =0; i < allMapMatchesInliers.size(); i++) {
            DMatch m = allMapMatchesInliers[i];
            int l_id = allLids[m.queryIdx];
            Landmark* lm = map->getLandmark(l_id);
            lm->addLfFrame(lf_cur, m.trainIdx, lf_cur->intraMatches[m.trainIdx].uv_ref);
            lf_cur->lIds[m.trainIdx] = l_id;
            //VLOG(2)<<"lm normal update: "<<lm->normal;
            ///logging
            n_rays[ lf_cur->intraMatches[m.trainIdx].n_rays - 1]++;
        }

        lf_cur->num_inliers_per_view = n_rays;
        n_rays = vector<int>(currentFrame->num_cams_, 0);
        /// Triangulate new landmarks and insert them as well.
        /// Actually we need to insert them only if they are seen in min_frames
        ///////////////////////////////////////////////////////////////////////////

        /////////////triangulate using our older method////////////////
//        vector<bool> tri_inliers_old(new_inter_matches.size(), true);
//        vector<Mat> triangulatedPtsOld;
//        triangulateMatches(prev_KF, lf_cur, new_inter_matches, tri_inliers_old, triangulatedPtsOld);
        ////////////////////////////////////////////////////////////////
        auto startT1 = high_resolution_clock::now();
        int num_triangulated = mapping(currentFrame, prev_KF, new_inter_matches, neighbors, inter_matches_with_landmarks_map, num_triangulated, n_rays);
//        int num_triangulated = mapping(currentFrame, prev_KF, new_inter_matches, neighbors, num_triangulated, n_rays);
        auto stopT1 = high_resolution_clock::now();
        auto durationT1 = duration_cast<milliseconds>(stopT1 - startT1);
        VLOG(0)<<"Time taken for mapping new lms is: " << durationT1.count() << endl;

        prev_KF->numTrackedLMs = prev_KF->numTrackedLMs + num_triangulated ;
        currentFrame->numTrackedLMs = num_inliers + num_triangulated ;
        currentFramePose =  currentFrame->pose.rowRange(0,3).clone();        //T.rowRange(0, 3).clone();

        currentFrame->num_matches_refKF = inter_matches.size();
        currentFrame->num_matches_refKF_lms = inter_matches_with_landmarks.size();
        currentFrame->num_matches_refKF_new = new_inter_matches.size();
        currentFrame->num_matches_localMap = allMapMatchesInliers.size();
        currentFrame->num_inliers_refKF_lms = num_inliers_refKF;
        currentFrame->num_inliers_localMap = num_inliers;
        currentFrame->num_triangulated = num_triangulated ;
        currentFrame->num_triangulated_per_view = n_rays ;
        n_rays.clear();

        ///Map statistics- print what is th average number of keyframes the landmarks are tracked in
        std::map<int, Landmark*>::iterator it;
        std::map<int, Landmark*> mapPoints = map->mapPoints;
        double avg_num_kfs_lm_tracked=0.0;
        int num_lm_tracked_in_three =0.0;
        int num_lm_tracked_in_two =0.0;
        int num_lms_map=0;
        for(it = mapPoints.begin() ; it != mapPoints.end() ; ++it){
            Landmark* l = it->second;
            int nm = l->KFs.size();
            avg_num_kfs_lm_tracked += nm;
            if(nm == 2)
                num_lm_tracked_in_two++;
            else if(nm == 3)
                num_lm_tracked_in_three++;
            num_lms_map++;
        }
        VLOG(2) << "Average number of KFs the landmarks are tracked in: " << avg_num_kfs_lm_tracked / (double)num_lms_map;
        VLOG(2) << "number of landmarks tracked in three keyframes: " << num_lm_tracked_in_three;
        VLOG(2) << "number of landmarks tracked in two keyframes: " << num_lm_tracked_in_two;


        ret = true;
        //////////////////////////////////////////////////////////////////////////

    }

    std::stringstream ss;
    ss <<std::setprecision(6)<<std::fixed<<currentFrame->timeStamp<<","<<std::setw(4)<< lfFrames.size()<<","<<std::setw(12)  <<prevlms<<","<<std::setw(11)  <<inter_matches_with_landmarks.size()<<","
                        <<std::setw(12)  <<new_inter_matches.size()<<","<<std::setw(11)  <<num_inliers<<","<<std::setw(11)  <<num_triangulated;
    std::string s = ss.str();
    VLOG(3)<<s<<endl;

//    if(useGPS){
//        if(lfFrames.back()->dummy_kf){
//            if(lf_cur->timeStamp - lfFrames.back()->timeStamp < 0.01){
//                VLOG(0) <<" Curr frame very close to prev dummy kf !!!!! ";
//                ret = false;
//                VLOG(0) << " ========= ";
//            }
//        }
//
//    }

    if(!ret)
        deleteCurrentFrame();

    return ret;
}

//int FrontEnd::mapping(const MultiCameraFrame *lf_cur, MultiCameraFrame *prev_KF, const vector<DMatch> &new_inter_matches,
//                      std::map<int, MultiCameraFrame *> &neighbors, int num_triangulated, vector<int> &n_rays) {
int FrontEnd::mapping(const MultiCameraFrame *lf_cur, MultiCameraFrame *prev_KF, const vector<DMatch> &new_inter_matches,
                      std::map<int, MultiCameraFrame *> &neighbors, std::map<int, vector<DMatch>>& interMatches,int num_triangulated, vector<int> &n_rays) {
    vector<double> depthVec;
    double avg_depth;
    double med_depth;
    n_rays = vector<int>(currentFrame->num_cams_, 0);
    auto startT = high_resolution_clock::now();

    TriangulateNewLandmarks(lf_cur, prev_KF, new_inter_matches, num_triangulated,
                            n_rays, depthVec, avg_depth);

    ////// Triangulate unmatched points with previosu keyframes////////
    int num_tri_old = 0;
    VLOG(0) <<"Number of total intermatches: " << interMatches.size();
    num_tri_old = triangulateNeighbors(neighbors, interMatches, depthVec);
//    num_tri_old = triangulateNeighbors(neighbors, depthVec);

    log_num_triangulated_ = num_triangulated + num_tri_old;
    log_KF_=1;
    auto stopT = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(stopT - startT);
    VLOG(0) << "time taken for Triangulating new features: " << duration.count() << endl;

    VLOG(0) << "Number of newly triangulated points : " << num_triangulated + num_tri_old << endl;
    VLOG(1) << "Number depthVec : " << depthVec.size() << endl;
    VLOG(1) << "Average Depth of triangulated points:" << avg_depth / num_triangulated << endl;

    sort(depthVec.begin(),depthVec.end());
    size_t idx = depthVec.size()/2;
    if(depthVec.size() == 0){
        return num_tri_old;
    }

    med_depth = depthVec[idx];
    VLOG(2) << "Median Depth of triangulate points: " << med_depth;

    //if( num_triangulated < 20){
//    VLOG(2)<<"Not enough landmarks have been created. BAD Frame";
//    return false;
//}
    insertKeyFrame();
    return num_triangulated + num_tri_old;
}

void FrontEnd::TriangulateNewLandmarks(const MultiCameraFrame *lf_cur, MultiCameraFrame *prev_KF, const vector<DMatch> &new_inter_matches,
                                       int &num_triangulated, vector<int> &n_rays, vector<double> &depthVec,
                                       double &avg_depth) {
    avg_depth= 0.0;
    vector<bool> triangulation_inliers(new_inter_matches.size(), true);
    double  med_depth=0.0;
    int cnt_behind_camera=0;
    int cnt_less_parallax = 0;
    int cnt_big_chi_err = 0;
    int cnt_already_assigned_lm =0;
    //Do a triangulation with non linear refinement of the inliers using both the LF frames
    for (int i = 0 ; i < new_inter_matches.size(); i++){

        ///Get the intramatches and their corresponding 2D observations
        DMatch m = new_inter_matches[i];
        vector<Mat> PJs;
        vector<int> view_inds;
        vector<Point2f> kps;
        int num_views =0;
        vector<Mat_<double> >  xs;

        if (lf_cur->lIds[m.trainIdx] != -1)
        {
            cnt_already_assigned_lm++;
            VLOG(3)<< "This intramatch feature in currentFrame is already assigned a landmark inside search localMap";
            continue;
        }
        ///// Triangulation using  opencv sfm and normalized coordinates///////////
        //triangulate and get the 3D point   ,
        Mat pt3d;
        /// Non Linear optimization ///
        Cameras cameras;
        gtsam::Point2Vector measurements;
        vector<int> compCamInds, octaves;
        int firstLFEnd=0;
        int firstLFRaysCnt =0;
        int secLFRaysCnt =0;
        bool optimize_tri = true;

        getDataForGTSAMTriangulation(prev_KF, m.queryIdx, cameras, measurements, compCamInds, octaves);
        firstLFEnd = cameras.size();
        firstLFRaysCnt = cameras.size();

        getDataForGTSAMTriangulation(currentFrame, m.trainIdx, cameras, measurements, compCamInds, octaves);

        secLFRaysCnt = cameras.size() - firstLFRaysCnt;
        double distanceThresh = -1;
        double outlierReprojThresh = 0; // we do not apply reprojection error check here. but use it later.
        gtsam::TriangulationParameters triParams(0.001, false, distanceThresh,outlierReprojThresh);
        gtsam::TriangulationResult TriResult = triangulateSafe(cameras,measurements,triParams);
        /// ONLY N-OV CASE

        if (TriResult.valid()){

            if(optimize_tri){
                ////////////////////// OPTIMIZATION ////////////////////////
                NonlinearFactorGraph graph;
                static SharedNoiseModel unit(noiseModel::Unit::Create(2));
                for (size_t j = 0; j < measurements.size(); j++) {
                    PinholeCamera<Cal3_S2> camera_i = cameras[j];
                    graph.emplace_shared<TriangulationFactor<PinholeCamera<Cal3_S2>> > //
                            (camera_i, measurements[j], unit, Symbol('P', 0));
                }

                LevenbergMarquardtParams params;
                params.verbosityLM = LevenbergMarquardtParams::SUMMARY;
//                params.verbosity = NonlinearOptimizerParams::ERROR;
                params.lambdaInitial = 1;
                params.lambdaFactor = 10;
                params.maxIterations = 50;
                params.absoluteErrorTol = 1.0;
                params.verbosityLM = LevenbergMarquardtParams::SILENT;
                params.verbosity = NonlinearOptimizerParams::SILENT;
                params.linearSolverType = NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY;

                Values result;

                for(int iter =0; iter < 1 ; iter++) {
                    Values initialEstimate;
                    initialEstimate.insert(Symbol('P', 0), *TriResult);
                    try{
                        result = LevenbergMarquardtOptimizer(graph, initialEstimate, params).optimize();

                        //cout << "Triangulation final error- iter:"<<iter<<" = " << graph.error(result) << endl;
                        gtsam::Point3 curest = result.at<gtsam::Point3>(Symbol('P', 0));
                        for(int facInd =0 ; facInd < graph.size(); facInd++) {
                            if (graph.at(facInd)) {
                                std::shared_ptr<TriangulationFactor<PinholeCamera<Cal3_S2>>> fac = std::static_pointer_cast<TriangulationFactor<PinholeCamera<Cal3_S2>>>(
                                        graph.at(facInd));
                                // check if this is an oulier?
                                gtsam::Point2 errVec = fac->evaluateError(curest);
                                gtsam::Matrix infMat = gtsam::Matrix::Identity(2,2) * orBextractor->GetInverseScaleSigmaSquares()[ octaves[facInd]];
                                double error = dot(errVec, infMat*errVec );
                                //double error = dot(errVec, errVec);
                                if (error > 5.991) {
                                    // this is an outlier. mark it
                                    triangulation_inliers[i] = false;
                                    cnt_big_chi_err++;
//                                        if(compCamInds[0] == 2){
//                                            drawInterMatch(prev_KF, currentFrame, prev_KF->intraMatches[m.queryIdx],currentFrame->intraMatches[m.trainIdx], "Traingulation Error");
//                                            VLOG(2)<<"Big chi-square error in triangulation"<<error;
//                                            VLOG(2)<<"triangulated cameras: "<<compCamInds[0]<<","<<compCamInds[1]<<"  Size: "<<compCamInds.size();
//                                        }

                                    break;

                                }
                            }
                        }


                    }
                    catch(gtsam::CheiralityException e){
//                            if(compCamInds[0] == 2){
//                                drawInterMatch(prev_KF, currentFrame, prev_KF->intraMatches[m.queryIdx],currentFrame->intraMatches[m.trainIdx], "Traingulation Error");
//                                VLOG(2)<<"Cheirality Exception at "<< e.nearbyVariable();
//                                VLOG(2)<<"triangulated cameras: "<<compCamInds[0]<<","<<compCamInds[1]<<"  Size: "<<compCamInds.size();
//
//                            }

                        triangulation_inliers[i] = false;
                        break;
                    }

                }
                if(result.exists(Symbol('P', 0)) and triangulation_inliers[i]){
                    gtsam::Point3 res = result.at<gtsam::Point3>(Symbol('P', 0));
                    pt3d = (cv::Mat_<double>(3, 1) << res.x(), res.y(), res.z());
                    //convert the point from world to camera frame
                    // Here we ar4e just computing the angle between the rig body pose1, rigbody pose2 and the 3D triangulated point.
                    // we are not exactly computing the angle between the rays/bearing vectors of the cameras in which triangulated point is seen.
                    // This is a safe thing to do with the asumption that the cameras are closes to each opther when compared to the triangulated point.
                    Mat ray = pt3d - currentFrame->pose.colRange(3, 4).rowRange(0, 3) ;
                    double dist = cv::norm(ray);
                    /// measure the  parallax and choose only points that have enough baseline between them
                    Mat ray1 = pt3d - prev_KF->pose.colRange(3,4).rowRange(0,3) ;
                    float dist1 = cv::norm(ray1);
                    float cosParallax = ray1.dot(ray)/(dist1*dist);
                    if(cosParallax >= 0.99998)
                    {
                        triangulation_inliers[i] = false;
                        cnt_less_parallax++;
//                            if(compCamInds[0] == 2){
//                                drawInterMatch(prev_KF, currentFrame, prev_KF->intraMatches[m.queryIdx],currentFrame->intraMatches[m.trainIdx], "Traingulation Error");
//                                VLOG(2)<<"Not enough motion between frames based on parallax";
//                                VLOG(2)<<"triangulated cameras: "<<compCamInds[0]<<","<<compCamInds[1]<<"  Size: "<<compCamInds.size();
//                                VLOG(2)<<"dist1: "<<dist1<<" Dist2: "<<dist<<" Parallax: "<<cosParallax;
//                            }

                    }
                    else{
                        depthVec.push_back(dist);
                        avg_depth += dist;
                    }


                }

                ////////////////////////////////////////////////////////////
            }
            else{

                gtsam::Point3 actual = *TriResult;
                triangulation_inliers[i] = true;
                pt3d = (cv::Mat_<double>(3, 1) << actual.x(), actual.y(), actual.z());
                //convert the point from world to camera frame
                Mat ray = pt3d - currentFrame->pose.colRange(3, 4).rowRange(0, 3) ;
                double dist = cv::norm(ray);

                Mat ray1 = pt3d - prev_KF->pose.colRange(3,4).rowRange(0,3) ;
                float dist1 = cv::norm(ray1);
                float cosParallax = ray1.dot(ray)/(dist1*dist);
                if(cosParallax >= 0.99998)
                {
                    triangulation_inliers[i] = false;
                    cnt_less_parallax++;
                    VLOG(2)<<"Not enough motion between frames based on parallax";
                }
                else{
                    depthVec.push_back(dist);
                    avg_depth += dist;
                }

            }

        }
        else{
            triangulation_inliers[i] = false;
            //VLOG(2)<<"Invalid Tri result";
            if(TriResult.Degenerate())
                VLOG(2)<<"Degenerate: "<<TriResult.degenerate();
            if(TriResult.BehindCamera()){
                cnt_behind_camera++;
                VLOG(3)<<"behind camera: "<<TriResult.behindCamera();
            }

            else if(TriResult.Outlier())
                VLOG(2)<<"Outlier: "<<TriResult.outlier();
            else if(TriResult.FarPoint())
                VLOG(2)<<"Far POint: "<<TriResult.farPoint();

//                if(compCamInds[0] == 2){
//                    drawInterMatch(prev_KF, currentFrame, prev_KF->intraMatches[m.queryIdx],currentFrame->intraMatches[m.trainIdx], "Traingulation Error");
//                    VLOG(2)<<"triangulated cameras: "<<compCamInds[0]<<","<<compCamInds[1]<<"  Size: "<<compCamInds.size();
//                }

        }

        if(triangulation_inliers[i]){

            //Eigen::Vector3d pt;
            //for(int j=0; j<3; ++j)
            //    pt[j] = pt3d.at<double>(j,0);
            //create Landmarks for each inlier and insert them into global map and well as
            // the keyframes.
            int l_id = map->insertLandmark(pt3d, prev_KF, m.queryIdx, prev_KF->intraMatches[m.queryIdx].uv_ref);
            Landmark* lm = map->getLandmark(l_id);
            lm->addLfFrame(currentFrame, m.trainIdx, currentFrame->intraMatches[m.trainIdx].uv_ref);

            prev_KF->lIds[m.queryIdx] = l_id;
            currentFrame->lIds[m.trainIdx] = l_id;
            num_triangulated++;
            n_rays[currentFrame->intraMatches[m.trainIdx].n_rays - 1]++;
            VLOG(3)<<"Inserted landmrak at LID: "<<l_id<<" -- "<<pt3d.at<double>(0,0)<<","<<pt3d.at<double>(1,0)<<","<<pt3d.at<double>(2,0);

        }
        else{
            triangulation_inliers[i] = false;
        }

    }
    VLOG(2) << "Number of behind_camera points: " << cnt_behind_camera;
    VLOG(2) << "Number of low parallax points: " << cnt_less_parallax;
    VLOG(2) << "Number of big chi error points: " << cnt_big_chi_err;
    VLOG(2) << "Number of intramatch feature already assigned to landmarks points: " << cnt_already_assigned_lm;
}

//////////////////////////////////////////////////////////////////////
/////////////// VISUALIZATION AND ANALYSIS FUNCTIONS ////////////////
////////////////////////////////////////////////////////////////////
//synchronized callback
//void FrontEnd::findLoopCandidates(vector<Mat>& images, vector<Mat>& segMasks){
//    MultiCameraFrame* fr = new MultiCameraFrame(camconfig_) ;//new MultiCameraFrame(images, segMasks, orb_vocabulary,
//                         // orBextractor, orBextractors, camconfig_, current_frameId++,
//                          //0.0, false);
//    fr->extractFeaturesParallel();
//    fr->parseandadd_BoW();
//
//    /*std::vector<array<int, 5>> matches_map;
//    vector<DBoW2::NodeId > words_;
//    fr->computeIntraMatches(matches_map, words_);
//
//    std::vector<array<int, 5> > filtered_intra_matches;
//    assert(words_.size() == matches_map.size());
//    set<DBoW2::NodeId > filtered_words;
//    filterIntraMatches( matches_map, fr, filtered_intra_matches, words_,filtered_words );
//    cout<<"Total number of filetered intra matches : "<<filtered_intra_matches.size()<<endl;
//
//    orb_vocabulary->transform(fr->intraMatchDescs,fr->lfBoW,fr->lfFeatVec,2);*/
//    vector<Mat> all_descriptors;
//    for(int i =0 ; i < fr->num_cams_; i++){
//        for(int j=0; j <fr->image_descriptors[i].size(); j++){
//            Point2f p = fr->image_kps[i][j].pt;
//            if(fr->segMasks[i].at<float>(p.y, p.x) < 0.7){
//                all_descriptors.push_back(fr->image_descriptors[i][j]);
//            }
//        }
//        //all_descriptors.insert(all_descriptors.end(), currentFrame->image_descriptors[i].begin(), currentFrame->image_descriptors[i].end());
//    }
//    DBoW2::BowVector bowVec_all;
//    DBoW2::FeatureVector featVec_all;
//    orb_vocabulary->transform(all_descriptors,bowVec_all,featVec_all,2);
//
//
//    vector<double> scores;
//    vector<double> stamps;
//    // For LF frame
//    lcLF_->queryDatabase(bowVec_all,7,scores, stamps);
//    cout.precision(10);
//    for(int i =0; i <scores.size(); i++){
//        cout<<" image "<<(i+1)<<" score: "<<scores.at(i)<<" stamp: "<<stamps.at(i)<<endl;
//    }
//    //for monocular case
//    DBoW2::BowVector bowVec;
//    DBoW2::FeatureVector featVec;
//    vector<double> scores_mono;
//    vector<double> stamps_mono;
//    orb_vocabulary->transform(fr->image_descriptors[2],bowVec,featVec,2);
//    lcMono_->queryDatabase(bowVec,7,scores_mono, stamps_mono);
//    cout.precision(10);
//    for(int i =0; i <scores_mono.size(); i++){
//        cout<<" image "<<(i+1)<<" score: "<<scores_mono.at(i)<<" stamp: "<<stamps_mono.at(i)<<endl;
//    }
//
//
//
//}


boost::json::array FrontEnd::vecToJson(const std::vector<std::vector<double>>& vec) {
    boost::json::array jsonArray;
    for (const auto& subVec : vec) {
        boost::json::array subJsonArray;
        for (const auto& val : subVec) {
            subJsonArray.push_back(val);
        }
        jsonArray.push_back(subJsonArray);
    }
    return jsonArray;
}

boost::json::array FrontEnd::matToJson(const std::vector<cv::Mat>& vec) {

    boost::json::array jsonArray;
    for (const auto& mat : vec)
    {
        // Convert each cv::Mat to a vector
        std::vector<uchar> vecMat(mat.begin<uchar>(), mat.end<uchar>());

        // Convert the vector to a boost::json::array
        boost::json::value jsonMat = boost::json::value_from(vecMat);

        // Add the boost::json::array to the boost::json::array
        jsonArray.push_back(jsonMat);
    }
    return jsonArray;
}

boost::json::value FrontEnd::loadMap(const string &fileName)
{
    std::ifstream file(fileName);
    if(!file)
    {
        throw std::runtime_error("Unable to open file : " + fileName);
    }
    std::stringstream buffer;
    buffer << file.rdbuf();

    return boost::json::parse(buffer.str());
}

void FrontEnd::appendLogsJSONformat(const std::string &entryId, const std::vector<int>& landmarks,
                                    const std::vector<std::vector<double>>& points,
                                    std::vector<cv::Mat> &descriptor, double timestamp, const std::vector<double>& position,
                                    const boost::json::array& poseArray)
{
    boost::json::object entryObj;
    entryObj["time"] = timestamp;
    entryObj["position"] = boost::json::value_from(position);
    entryObj["pose"] = poseArray;
    entryObj["l_ids"] = boost::json::value_from(landmarks);
    entryObj["points"] = vecToJson(points);
    boost::json::array descriptorArray;
    descriptorArray = matToJson(descriptor);
    entryObj["descriptor"] = descriptorArray;

    json_file_obj[entryId] = entryObj;

}


void FrontEnd::serializeJSONObject() const
{
    // TODO: Take the file from the
    std::ofstream file(mapFile);
    file << boost::json::serialize(json_file_obj);
    file.close();
}

void FrontEnd::appendLogs(bool loop_detection, gtsam::NavState prevState_, gtsam::imuBias::ConstantBias prevBias_)
{
    VLOG(0) << "Adding logs . . . " << std::endl;
    // *** For the JSON file.
    std::string entryId = "entry_" + std::to_string(EntryCount);
    std::vector<int> landmarks;
    double timestamp;
    std::vector<std::vector<double>> points;
    std::vector<cv::Mat> descriptors;
    std::vector<double> position;
    position.reserve(3);
    // ***

    std::map<int, Landmark*>::iterator it;

    auto f = lfFrames.back();

    if(lfFrames.size() == 2 && init_mode == true){
        auto fr_0 = lfFrames[0];
        logFile2_ << "x" << " " << fr_0->kfID <<" "<< setprecision(15) << poseTimeStamps[0]
                  << " " << fr_0->pose.at<double>(0, 0) << " " << fr_0->pose.at<double>(0, 1) << " " << fr_0->pose.at<double>(0, 2) << " " << fr_0->pose.at<double>(0, 3)
                  << " " << fr_0->pose.at<double>(1, 0) << " " << fr_0->pose.at<double>(1, 1) << " " << fr_0->pose.at<double>(1, 2) << " " << fr_0->pose.at<double>(1, 3)
                  << " " << fr_0->pose.at<double>(2, 0) << " " << fr_0->pose.at<double>(2, 1) << " " << fr_0->pose.at<double>(2, 2) << " " << fr_0->pose.at<double>(2, 3)
                  << " " << fr_0->pose.at<double>(3, 0) << " " << fr_0->pose.at<double>(3, 1) << " " << fr_0->pose.at<double>(3, 2) << " " << fr_0->pose.at<double>(3, 3) << "\n";
        init_mode = false;
    }

    logFile2_  << "x" << " " << f->kfID <<" "<< setprecision(15) << poseTimeStamps.back()
               << " " << f->pose.at<double>(0, 0) << " " << f->pose.at<double>(0, 1) << " " << f->pose.at<double>(0, 2) << " " << f->pose.at<double>(0, 3)
               << " " << f->pose.at<double>(1, 0) << " " << f->pose.at<double>(1, 1) << " " << f->pose.at<double>(1, 2) << " " << f->pose.at<double>(1, 3)
               << " " << f->pose.at<double>(2, 0) << " " << f->pose.at<double>(2, 1) << " " << f->pose.at<double>(2, 2) << " " << f->pose.at<double>(2, 3)
               << " " << f->pose.at<double>(3, 0) << " " << f->pose.at<double>(3, 1) << " " << f->pose.at<double>(3, 2) << " " << f->pose.at<double>(3, 3) << "\n";

    // Store the time-stamp.
    timestamp = poseTimeStamps.back();
    // Store the position of the keyframe.
    position.emplace_back(f->pose.at<double>(0,3));
    position.emplace_back(f->pose.at<double>(1,3));
    position.emplace_back(f->pose.at<double>(2,3));
    // store the poses
    boost::json::array poseArray = vecToJson(matToVec(f->pose));
    // Store the landmarks.
    landmarks = f->lIds;
    // Store the descriptors.
    for (auto &d : f->intraMatches)
        descriptors.push_back(d.matchDesc);
    // Store the 3D points of the map.
    for (auto& l : landmarks)
    {
        if(l == -1)
            continue;
        std::vector<double> point(3);
        cv::Mat point3 = map->getLandmark(l)->pt3D;
        point[0] = point3.at<double>(0,0);
        point[1] = point3.at<double>(1,0);
        point[2] = point3.at<double>(2,0);
        points.emplace_back(point);
    }
    // Add to the JSON file.
    appendLogsJSONformat(entryId, landmarks, points, descriptors, timestamp, position, poseArray);
    EntryCount += 1;
    Vector3 velocity = prevState_.v();
    Vector3 acc_bias = prevBias_.accelerometer();
    Vector3 gyr_bias = prevBias_.gyroscope();
    logFile2_  << "imu_est" << " " << f->kfID <<" "<< setprecision(15) <<
               velocity.x() << " " << velocity.y() << " " << velocity.z() << " " <<
               acc_bias(0)  << " " << acc_bias(1)  << " " << acc_bias(2)  << " " <<
               gyr_bias(0)  << " " << gyr_bias(1)  << " " << gyr_bias(2)  << "\n" ;

    // Append only the new map points to the logs.
    for (it = map->mapPoints.begin(); it != map->mapPoints.end(); ++it) {
        std::vector<double> point(3);
        Landmark *l = it->second;
        if(l->lId <= prev_point)
        {
            continue;
        }
        landmarks.emplace_back(l->lId);
        point[0] = l->pt3D.at<double>(0,0);
        point[1] = l->pt3D.at<double>(1,0);
        point[2] = l->pt3D.at<double>(2, 0);
        points.emplace_back(point);
        int frameIdx = 0;
        logFile2_ << "l" << " " << l->lId << " " << " " << l->pt3D.at<double>(0, 0) << " " << l->pt3D.at<double>(1, 0) << " "
                  << l->pt3D.at<double>(2, 0) << "\n";
        for (auto f: l->KFs)
        {

            //get the key point index of each component camera where this landmark is seen
            IntraMatch *intraMatch = &f->intraMatches[l->featInds[frameIdx]];
            for (int i = 0; i < f->num_cams_; i++) {
                int kp_ind = intraMatch->matchIndex[i];
                if (kp_ind != -1) {
                    VLOG(3) << f->kfID << " " << i << " " << f->image_kps_undist[i][kp_ind].pt.x << " "
                            << f->image_kps_undist[i][kp_ind].pt.y << f->image_descriptors[i][kp_ind] << "\n";
                    logFile2_ << "e" << " " << f->kfID << " " << i << " " << f->image_kps_undist[i][kp_ind].pt.x
                              << " " << f->image_kps_undist[i][kp_ind].pt.y << "\n";
                }
            }
            frameIdx++;
        }

    }
    if(!map->mapPoints.empty())
    {
        prev_point = map->mapPoints.rbegin()->first;
    }

    if(loop_detection)
    {
        // If loop detection occurs add the relative pose between the recognised frames.
        gtsam::Matrix4 poseMatrix = result.relative_pose.matrix();
        logFile2_ << "k" << " " << lfFrames[result.match]->kfID << " " << currentFrame->kfID << " " <<
                  poseMatrix(0,0) << " " << poseMatrix(0,1) << " " << poseMatrix(0,2) << " " << poseMatrix(0,3) << " " <<
                  poseMatrix(1,0) << " " << poseMatrix(1,1) << " " << poseMatrix(1,2) << " " << poseMatrix(1,3) << " " <<
                  poseMatrix(2,0) << " " << poseMatrix(2,1) << " " << poseMatrix(2,2) << " " << poseMatrix(2,3) << " " <<
                  poseMatrix(3,0) << " " << poseMatrix(3,1) << " " << poseMatrix(3,2) << " " << poseMatrix(3,3) << "\n";
        VLOG(0) << "LOOP_CLOSURE factors recorded." << std::endl;
        for(int i = 0; i < result.lIds.size(); i++)
        {
            logFile2_ << "m" << " " << currentFrame->kfID << " " << result.lIds[i] << " ";
            for(auto &inner : result.measurements[i])
            {
                logFile2_ << std::get<0>(inner) << " " << std::get<1>(inner).x << " " << std::get<1>(inner).y << " ";
            }
            logFile2_ << "\n";
        }
    }
}

std::vector<std::vector<double>> FrontEnd::matToVec(const cv::Mat& mat) {
    std::vector<std::vector<double>> vec;
    for (int i = 0; i < mat.rows; ++i) {
        std::vector<double> row;
        for (int j = 0; j < mat.cols; ++j) {
            row.push_back(mat.at<double>(i, j));
        }
        vec.push_back(row);
    }
    return vec;
}

/// This method takes key points of two images corresponding to two cameras
/// Undistorts them returns their normalized coordinates or bearing vectors
/// \param kps_1  : vector of key points in first image
/// \param kps_2  : vector of key points in second image
/// \param cam1  : index of first camera
/// \param cam2  : index of second camera
///  IMU Pre-integration related functions

Eigen::Quaterniond  FrontEnd::world_imu_frame(std::deque<mcslam::ImuSample>& imu_msgs){

   Eigen::MatrixXd acceleration_matrix(imu_msgs.size(), 3);

   for(int i=0; i<imu_msgs.size(); i++){
       acceleration_matrix(i,0) = imu_msgs[i].accel.x();
       acceleration_matrix(i,1) = imu_msgs[i].accel.y();
       acceleration_matrix(i,2) = imu_msgs[i].accel.z();
   }

   Eigen::MatrixXd g_matrix(imu_msgs.size(), 3);
   g_matrix.setZero();

   g_matrix.col(2) = Eigen::VectorXd::Constant(imu_msgs.size(), 9.81);
   Eigen::MatrixXd R = kabsch(acceleration_matrix, g_matrix);
   world_imu_rotation = R;
   Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::Matrix3d(R)).normalized();

   return q;

}

Eigen::MatrixXd FrontEnd::kabsch(Eigen::MatrixXd A, Eigen::MatrixXd B){

   assert(A.rows() == B.rows());

   int n = A.rows();
   int m = A.cols();

   Eigen::VectorXd EA = A.colwise().mean();
   Eigen::VectorXd EB = B.colwise().mean();


   Eigen::MatrixXd H = ((A.rowwise() - EA.transpose()).transpose() * (B.rowwise() - EB.transpose())) / n;
   Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

   Eigen::MatrixXd U = svd.matrixU();
   Eigen::MatrixXd VT = svd.matrixV();
   Eigen::MatrixXd D =  svd.singularValues();

   double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
   if (d > 0)
       d = 1.0;
   else
       d = -1.0;


   Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
   I(2, 2) = d;

   Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

    VLOG(0) << "Rotation Kabsch: " << R << endl;

   return R;

    // compute the centroid of each matrix
//    Eigen::Vector3d centroid_A = A.colwise().mean();
//    Eigen::Vector3d centroid_B = B.colwise().mean();
//
//    // compute the cross product of the 2 vectors to find the angle of rotation
//    Eigen::Vector3d cross_product = centroid_A.cross(centroid_B);
//
//    // compute the rotation by computing the dot product of the 2 vectors
//    double dot_product = centroid_A.dot(centroid_B);
//
//    // convert the axis angle representation to a quaternion
//    Eigen::AngleAxisd angle_axis;
//    angle_axis.angle() = acos(dot_product/(centroid_A.norm() * centroid_B.norm()));
//    angle_axis.axis() = cross_product.normalized();
//    Eigen::Quaterniond q(angle_axis);
//    return q.toRotationMatrix();

}

void FrontEnd::imu_initialize_noise(gtsam::imuBias::ConstantBias prior_imu_bias){



    // set the prior bias
    bias_prior = prior_imu_bias;
    VLOG(0) << "bias prior: " << bias_prior << endl;
    // TODO (Shankar): This can be directly assigned, unnecessary variable creation.
    auto w_coriolis = gtsam::Vector3(0, 0, 0);

    // Create GTSAM preintegration params
    if(relocal && ImuMapFrame == 0 && g_norm < 0.0){
        params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(-g_norm);
    }
    else{
        params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(g_norm);
    }
    // acc noise
    params->setAccelerometerCovariance(gtsam::I_3x3 * pow(acc_n, 2));
    // gyro noise
    params->setGyroscopeCovariance(gtsam::I_3x3 * pow(gyr_n, 2));
    // bias acc
    params->biasAccCovariance = gtsam::I_3x3 * pow(acc_w, 2);
    // bias gyro
    params->biasOmegaCovariance = gtsam::I_3x3 * pow(gyr_w, 2);
    params->omegaCoriolis = w_coriolis;

    double integration_sigma = 1e-8;
    params->setIntegrationCovariance( gtsam::I_3x3 * pow(integration_sigma, 2) );  //1e-8

   // //error in bias
    params->biasAccOmegaInt = gtsam::I_6x6 * 1e2;// 1e-5;

    priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished()); // rad,rad,rad,m, m, m
    priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 0.1); // m/s
    bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
    if (relocal){
        bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6, 1e0);
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e0); // m/s
    }

   // Create GTSAM preintegration object
    this->imu_integrator_comb = new gtsam::PreintegratedCombinedMeasurements(params, bias_prior);

   return;
}

void FrontEnd::imu_initialize(std::deque<mcslam::ImuSample> imu_msgs, double image_time){

        if(imu_msgs_initialized.size() < 200){
            VLOG(0) << "IMU NOT INITIALIZED" << endl;

            VLOG(0) << imu_msgs_initialized.size() ;

            for(int i = 0; i < imu_msgs.size(); i++){
                imu_msgs_initialized.push_back(imu_msgs[i]);

                 appendIMUValue(&imu_msgs_initialized.back());
            }
            return;
        }
        Eigen::Quaterniond q = world_imu_frame(imu_msgs_initialized);

        // calculate bias:
        double acc_x_sum = 0;
        double acc_y_sum = 0;
        double acc_z_sum = 0;
        double gyr_x_sum = 0;
        double gyr_y_sum = 0;
        double gyr_z_sum = 0;

        for (int i = 0; i< imu_msgs_initialized.size(); i++){
            double acc_x = imu_msgs_initialized[i].accel.x();
            double acc_y = imu_msgs_initialized[i].accel.y();
            double acc_z = imu_msgs_initialized[i].accel.z();
            double gyr_x = imu_msgs_initialized[i].gyro.x();
            double gyr_y = imu_msgs_initialized[i].gyro.y();
            double gyr_z = imu_msgs_initialized[i].gyro.z();

            acc_x_sum += acc_x;
            acc_y_sum += acc_y;
            acc_z_sum += acc_z;
            gyr_x_sum += gyr_x;
            gyr_y_sum += gyr_y;
            gyr_z_sum += gyr_z;

        }
        bias_acc(0) = acc_x_sum / imu_msgs_initialized.size();
        bias_acc(1) = acc_y_sum / imu_msgs_initialized.size();
        bias_acc(2) = acc_z_sum / imu_msgs_initialized.size();

        bias_gyr(0) = gyr_x_sum / imu_msgs_initialized.size();
        bias_gyr(1) = gyr_y_sum / imu_msgs_initialized.size();
        bias_gyr(2) = gyr_z_sum / imu_msgs_initialized.size();


        Eigen::Vector4d q_coeffs(q.w(), q.x(), q.y(), q.z());

        gtsam::Rot3 prior_rotation = gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z());
        gtsam::Point3 prior_point(0.0, 0.0, 0.0);

        gtsam::Pose3 prior_pose(prior_rotation, prior_point);
        gtsam::Vector3 prior_velocity(0.0, 0.0, 0.0);

        // correct bias_Acc w.r.t g_norm
        bias_acc(2) = bias_acc(2) - g_norm;

        // set bias_prior to be bias_acc and bias_gyr
        gtsam::imuBias::ConstantBias prior_imu_bias(gtsam::Vector3(bias_acc(0), bias_acc(1), bias_acc(2)), gtsam::Vector3(bias_gyr(0), bias_gyr(1), bias_gyr(2)));

        imu_initialize_noise(prior_imu_bias);

        imu_initialized = true;
        imu_msgs_initialized.clear();
        VLOG(0) << "IMU INITIALIZED" << endl;
        return;

}


void FrontEnd::imu_values_removal(double image_time){

    //unique mutex lock
    unique_lock<mutex> lock(mMutexImu);
    // remove messages that came before the first keyframe
    while(!imu_msgs_combined.empty()){
        double imuTime = imu_msgs_combined.front().timestamp;
        if(imuTime > image_time){
            break;
        }
        imu_msgs_combined.pop_front();
    }

}

/// This is a method which performs pre-integration on
/// all the IMU messages that have been accumulated upto "image_time"
/// \param image_time
void FrontEnd::imu_preintegration(double image_time){

    // preintegration
    unique_lock<mutex> lock(mMutexImu);

    VLOG(1) << "Time difference between images: " <<   image_time - prev_image_time << endl;
    VLOG(1) << "IMU messages in this time interval: " <<  (image_time - prev_image_time) * 200<< endl;
    int initial_size = imu_msgs_combined.size();
    VLOG(0) << "number of imu messages before preintegration: " << imu_msgs_combined.size() << endl;
    VLOG(1) << "image time: " << std::setprecision(15) << image_time << endl;
    VLOG(1) << " first imu time " << std::setprecision(15) << imu_msgs_combined.front().timestamp << endl;
    imu_message_empty = false;

    if(imu_msgs_combined.empty()){

        imu_message_empty = true;

    }

    while(!imu_msgs_combined.empty()){
        double imuTime = imu_msgs_combined.front().timestamp;

        // assert (imuTime < image_time) ;
        if(imuTime > image_time){
            break;
        }

        double dt = (last_imu_time < 0) ? (1.0 / 200.0) : (imuTime - last_imu_time);

        imu_integrator_comb->integrateMeasurement(gtsam::Vector3(imu_msgs_combined.front().accel.x(), imu_msgs_combined.front().accel.y(), imu_msgs_combined.front().accel.z()),
                                                 gtsam::Vector3(imu_msgs_combined.front().gyro.x(), imu_msgs_combined.front().gyro.y(), imu_msgs_combined.front().gyro.z()),
                                                 dt);


        appendIMUValue(&imu_msgs_combined.front());
        last_imu_time = imu_msgs_combined.front().timestamp;

        imu_msgs_combined.pop_front();
    }

    int final_size = imu_msgs_combined.size();

    if(initial_size - final_size == 0){
        VLOG(0) << "imu messges removed: " << initial_size - final_size << endl;
        VLOG(0) <<"no imu message!!";
    }

    // special condition - if imu message is just 1 between the kfs
    if(initial_size - final_size < 3){
        VLOG(0) << "bad condition";
        VLOG(0) << "imu messges removed: " << initial_size - final_size << endl;
        imu_message_empty = true;
    }


    prev_image_time = image_time;
    VLOG(0) << "number of imu messages after preintegration: " << imu_msgs_combined.size() << endl;
    VLOG(1) << " Last imu time " << std::setprecision(15) <<  last_imu_time;
    VLOG(1) <<"imu message empty - preint " << imu_message_empty;

}


// GPS related functions
tuple<double, double, double> FrontEnd::geodetic_to_enu(mcslam::GnssFix gps_msg){

    double x, y, z;
    
    double lat = gps_msg.latitude;
    double lon = gps_msg.longitude;
    double alt = gps_msg.altitude;

    // Alternatively: const Geocentric& earth = Geocentric::WGS84();

    double lat0 = refLat;
    double lon0 = refLon;
    double alt0 = refAlt;

    GeographicLib::LocalCartesian proj(lat0, lon0, alt0, GeographicLib::Geocentric::WGS84());
    
    proj.Forward(lat, lon, alt, x, y, z);

    return make_tuple(x, y, z);

}

tuple<double, double, double> FrontEnd::enu_wrt_origin(tuple<double, double, double> gps_message){

    double x, y, z;
    
    double gps_diff_x = get<0>(gps_message) - get<0>(gps_enu_origin);
    double gps_diff_y = get<1>(gps_message) - get<1>(gps_enu_origin);
    double gps_diff_z = get<2>(gps_message) - get<2>(gps_enu_origin);

    return make_tuple(gps_diff_x, gps_diff_y, gps_diff_z);

}

void FrontEnd::gps_ENU_reference_params(mcslam::GnssFix gps_msg){

    refLat = gps_msg.latitude;
    refLon = gps_msg.longitude;
    refAlt = gps_msg.altitude;

    return;

}


void FrontEnd::interpolation_vins_GPS(vector<gps_umeyama_data> gps_umeyama_data_vec, Eigen::MatrixXd &gps_eigen , Eigen::MatrixXd &vins_eigen){


    for(int i =0; i< gps_umeyama_data_vec.size(); i++){

        //1. store the gps messages
        tuple<double, double, double> gps_enu = geodetic_to_enu(gps_umeyama_data_vec[i].gps_msg);
        gps_eigen(i, 0) = get<0>(gps_enu);
        gps_eigen(i, 1) = get<1>(gps_enu);
        gps_eigen(i, 2) = get<2>(gps_enu);

        VLOG(0) << " GPS: " << gps_eigen(i, 0) << ", " << gps_eigen(i, 1) << ", " << gps_eigen(i, 2);


        //2. compute the alpha for interpolation
        double kf1_time = gps_umeyama_data_vec[i].prev_keyframe->timeStamp;
        double kf2_time = gps_umeyama_data_vec[i].next_keyframe->timeStamp;
        double gps_time = gps_umeyama_data_vec[i].gps_msg.timestamp;

        double alpha = (gps_time - kf1_time)/(kf2_time - kf1_time);

        //3. compute position: lf frame pose is Twc - so to get Twb.

        cv::Mat kf1_pose = gps_umeyama_data_vec[i].prev_keyframe->pose * Tbc_cv.inv();
        cv::Mat kf2_pose = gps_umeyama_data_vec[i].next_keyframe->pose * Tbc_cv.inv();

        // kf pose body with respect to the vio frame
        gtsam::Pose3 kf1_pose_vb = convertPose3_CV2GTSAM(kf1_pose);
        gtsam::Pose3 kf2_pose_vb = convertPose3_CV2GTSAM(kf2_pose);

        // extract translation
        Eigen::Vector3d kf1_translation(kf1_pose.at<double>(0,3), kf1_pose.at<double>(1,3), kf1_pose.at<double>(2,3));
        Eigen::Vector3d kf2_translation(kf2_pose.at<double>(0,3), kf2_pose.at<double>(1,3), kf2_pose.at<double>(2,3));

        // compute interpolated translation
        Eigen::Vector3d interpolated_translation = (1-alpha)*kf1_translation + alpha*kf2_translation;

        //4. compute rotation

        Eigen::MatrixXd kf1_rot_vb = kf1_pose_vb.rotation().matrix();
        Eigen::MatrixXd kf2_rot_vb = kf2_pose_vb.rotation().matrix();

        Eigen::MatrixXd  kf2_rot_bv = kf2_rot_vb.transpose();

        Eigen::MatrixXd kf2_rot_kf1 = kf2_rot_bv * kf1_rot_vb;

        gtsam::Rot3 R_kv = gtsam::Rot3(gtsam::Rot3::Expmap( alpha * gtsam::Rot3::Logmap(gtsam::Rot3(kf2_rot_kf1))).matrix() * (kf1_rot_vb.transpose()));
        gtsam::Rot3 R = gtsam::Rot3(R_kv.transpose()); //rotation of the kth frame in the vio frame

        // convert this rotation to eigen

        Eigen::Matrix3d R_eigen = R.matrix();

        //5. create a gtsam pose for this rotation and translation.

        gtsam::Pose3 interpolated_pose(R, gtsam::Point3(interpolated_translation(0), interpolated_translation(1), interpolated_translation(2)));

        //6. convert this pose cv::Mat

        cv::Mat interpolated_pose_body = convertPose3_GTSAM2CV(interpolated_pose);

        // 7. multiply this interpolated pose with T_gps_body to get the pose in the gps frame

        // translation componenet of Tbg.inv() as eigne vector
        
        Eigen::Vector3d Tbg_translation(Tbg.at<double>(0,3), Tbg.at<double>(1,3), Tbg.at<double>(2,3));

        //gives pose of gps in vins frames
        Eigen::Vector3d  pose_vins_gps = R_eigen * Tbg_translation + interpolated_translation;

        Eigen::Vector3d position_vins_gps(pose_vins_gps(0), pose_vins_gps(1),pose_vins_gps(2));


        //8. store this position in vins_eigen
        vins_eigen(i, 0) = position_vins_gps(0);
        vins_eigen(i, 1) = position_vins_gps(1);
        vins_eigen(i, 2) = position_vins_gps(2);

    }
}




void FrontEnd::gps_initialize_kabsch(std::deque<mcslam::GnssFix>  gps_msgs_combined, MultiCameraFrame* vins_kf){


    cv::Mat vins_pose_mat = vins_kf->pose;
    gtsam::Pose3 vins_pose;
    vins_pose = convertPose3_CV2GTSAM(vins_pose_mat);

    if(gps_umeyama_data_vec.size() < 15){

        gps_umeyama_data data;
        data.gps_msg = gps_msgs_combined.back();
        data.prev_keyframe = vins_kf;
        data.next_keyframe = NULL;

        if(gps_umeyama_data_vec.size() > 0){
            //to handle cases when we have vision kfs coming late and have only 1 kf between 2 gps messages
            if(gps_umeyama_data_vec.back().next_keyframe == NULL ){
                // check: when does this condition not get satisfied?
                if(gps_umeyama_data_vec.back().gps_msg.timestamp > vins_kf->timeStamp){
                    return;
                }
                // check: redundant?
                gps_umeyama_data_vec.back().next_keyframe = vins_kf;
            }
        }
        gps_umeyama_data_vec.push_back(data);
        gps_vins_next = true;

        VLOG(0) << "GPS NOT INITIALIZED" << endl;
        return;
    }

    // edge case
    // the last gps message might not have saved the next kf. so wait until it gets stored
    if(gps_umeyama_data_vec.back().next_keyframe == NULL){
        return;
    }

    //store the reference lat and long
    gps_ENU_reference_params(gps_umeyama_data_vec[0].gps_msg);

    // create eigen matrices for both gps and vins poses
    Eigen::MatrixXd gps_eigen(gps_umeyama_data_vec.size(), 3);
    Eigen::MatrixXd vins_eigen(gps_umeyama_data_vec.size(), 3);

    //convert vins messages to ENU frame and perform interpolation.
    interpolation_vins_GPS(gps_umeyama_data_vec, gps_eigen, vins_eigen);

    //steps:
    // 1. centroid and subtract from all points
    // 2. get eigen matrices for both points
    // 3. get the rotation matrix from the two eigen matrices
    // 4. get the translation vector from the two centroids
    // 5. create a transformation matrix from the rotation matrix and translation vector

    // 1. centroid and subtract from all points
    Eigen::Vector3d gps_centroid = gps_eigen.colwise().mean();
    Eigen::Vector3d vins_centroid = vins_eigen.colwise().mean();

    Eigen::MatrixXd gps_eigen_centered(gps_eigen.rows(), gps_eigen.cols());
    Eigen::MatrixXd vins_eigen_centered(vins_eigen.rows(), vins_eigen.cols());

    gps_eigen_centered = gps_eigen.rowwise() - gps_centroid.transpose();
    vins_eigen_centered = vins_eigen.rowwise() - vins_centroid.transpose();

    // 2. get eigen matrices for both points - already done

    // 3. get the rotation matrix from the two eigen matrices - rotation from gps -> vins frame
    Eigen::MatrixXd R_vins_gps = kabsch(gps_eigen_centered, vins_eigen_centered  );

    // 4. get the translation vector from the two centroids
    Eigen::Vector3d t_vins_gps = vins_centroid - R_vins_gps * gps_centroid;

    // 5. create a transformation matrix from the rotation matrix and translation vector

    Eigen::Matrix4d T_vins_gps = Eigen::Matrix4d::Identity();
    T_vins_gps.block<3,3>(0,0) = R_vins_gps;
    T_vins_gps.block<3,1>(0,3) = t_vins_gps;

    // eigen to cv::Mat
    cv::eigen2cv(T_vins_gps, T_body_gps);
    VLOG(0) << "T_body_gps: " << endl  << T_body_gps << endl;

    gps_initialized = true;

    for(int i =0; i < 10; i++){
        VLOG(0) << "===============INIT DONE====================" << endl;
    }

    VLOG(0) << "GPS init done" << endl;
    gps_umeyama_data_vec.clear();

}

double FrontEnd::gps_euclidean(tuple<double, double, double> gps_msg1, tuple<double, double, double> gps_msg2){

    return sqrt(pow(get<0>(gps_msg1) - get<0>(gps_msg2), 2) + pow(get<1>(gps_msg1) - get<1>(gps_msg2), 2) + pow(get<2>(gps_msg1) - get<2>(gps_msg2), 2));
}

bool FrontEnd::validGPSmessage(gps_message gps_data_){


        double KF_time = lfFrames.back()->timeStamp;

        VLOG(0) <<"gps message time " << std::setprecision(15) << gps_msgs_combined.front().timestamp;
        VLOG(0) <<"prev image time " << std::setprecision(15) << KF_time;

        if(gps_msgs_combined.front().timestamp < KF_time){
            // old message:
//            VLOG(0) <<"gps message time " << std::setprecision(15) << gps_msgs_combined.front().timestamp;
//            VLOG(0) <<"prev image time " << std::setprecision(15) << KF_time;
            VLOG(0) << "Old GPS message" << endl;
            gps_msgs_combined.pop_front();
            return false;

        }

        if(gps_msgs_combined.front().timestamp > KF_time + 0.5){
            // new message - wait for more vision frames maybe
            // check:
            VLOG(0) << "New GPS message" << endl;
            return false;

        }

        VLOG(0) <<"gps message 1"  << get<0>(gps_data_.prev_gps_message_enu) <<" " << get<1>(gps_data_.prev_gps_message_enu) << " " << get<2>(gps_data_.prev_gps_message_enu);
        VLOG(0) <<"gps message 2"  << get<0>(gps_data_.curr_gps_message_enu) <<" " << get<1>(gps_data_.curr_gps_message_enu) << " " << get<2>(gps_data_.curr_gps_message_enu);

        // check if euclidean distance between the two gps messages is grater than 0.5m
        if(gps_euclidean(gps_data_.prev_gps_message_enu, gps_data_.curr_gps_message_enu) > 0.5){
            VLOG(0) << std::setprecision(10) << " gps distance is " << gps_euclidean(gps_data_.prev_gps_message_enu, gps_data_.curr_gps_message_enu);
            return true;
        }
        else{
            VLOG(0) << std::setprecision(10) << " gps distance is " << gps_euclidean(gps_data_.prev_gps_message_enu, gps_data_.curr_gps_message_enu);
            gps_msgs_combined.pop_front();
            return false;
        }
}




void FrontEnd::appendIMUValue(mcslam::ImuSample* message)
{
//    VLOG(0) << "Appending IMU values..." << std::endl;
    logFile2_ << "imu_raw" << " " << setprecision(20)
    << message->timestamp << " "
    << message->accel.x() << " " << message->accel.y() << " " << message->accel.z() << " "
    << message->gyro.x() << " " << message->gyro.y() << " " << message->gyro.z() << "\n";
}

void FrontEnd::appendGPSValue(gps_message gps_data_stored, int gps_kf){

    logFile2_ << "g" <<  " " << gps_kf << " " << std::setprecision(15) <<  gps_data_stored.tStamp << " " << get<0>(gps_data_stored.curr_gps_message_enu)
            << " " << get<1>(gps_data_stored.curr_gps_message_enu) << " " << get<2>(gps_data_stored.curr_gps_message_enu)
            << " " << refLat << " " << refLon << " " << refAlt
            << "\n";
    VLOG(0) << "GPS factors recorded. at time " << std::setprecision(15) <<  gps_data_stored.tStamp << std::endl;

}



