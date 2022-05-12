#include <iostream>
#include <algorithm>
#include <string>
#include <numeric>

// not needed if compiler flag `-DCERES_FOUND` is set
// #define CERES_FOUND true

#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <boost/filesystem.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>

#define _USE_OPENCV true
// #include <libs/MVS/Interface.h>
#include <OpenMVS/MVS/Interface.h>

// using namespace cv;
// using namespace std;
namespace fs = boost::filesystem;

class StructureFromMotion {

public:
    StructureFromMotion(const std::string& dir,
            const float matchSurvivalRate = 0.5f,
            const bool viz = false,
            const std::string mvs = "",
            const std::string cloud = "",
            const bool saveDebug = false)
        :PAIR_MATCH_SURVIVAL_RATE(matchSurvivalRate),
         visualize(viz),
         saveMVS(mvs),
         saveCloud(cloud),
         saveDebugVisualizations(saveDebug)

    {
        findImagesInDiretcory(dir);
    }

    void runSfM() {
        extractFeatures();
        matchFeatures();
        buildTracks();
        reconstructFromTracks();
        if (visualize) {
            visualize3D();
        }
        if (saveMVS != "") {
            saveToMVSFile();
        }
        if (saveCloud != "") {
            CV_LOG_INFO(TAG, "Save point cloud to: " + saveCloud);
            cv::viz::writeCloud(saveCloud, pointCloud, pointCloudColor);
        }
    }

private:
    void findImagesInDiretcory(const std::string& dir) {
        CV_LOG_INFO(TAG, "Finding images in " + dir);

        cv::utils::fs::glob(dir, "*.jpg", imagesFilenames);
        cv::utils::fs::glob(dir, "*.JPG", imagesFilenames);
        cv::utils::fs::glob(dir, "*.png", imagesFilenames);
        cv::utils::fs::glob(dir, "*.PNG", imagesFilenames);

        std::sort(imagesFilenames.begin(), imagesFilenames.end());

        CV_LOG_INFO(TAG, "Found " + std::to_string(imagesFilenames.size()) + " images");

        CV_LOG_INFO(TAG, "Reading images...");
        for (const auto& i : imagesFilenames) {
            CV_LOG_INFO(TAG, i);
            images[i] = cv::imread(i);
            imageIDs[i] = images.size() - 1;
        }
    }

    void extractFeatures() {
        CV_LOG_INFO(TAG, "Extract Features");

        auto detector = cv::AKAZE::create();
        auto extractor = cv::AKAZE::create();

        for (const auto& i : imagesFilenames) {
            cv::Mat grayscale;
            cv::cvtColor(images[i], grayscale, cv::COLOR_BGR2GRAY);
            detector->detect(grayscale, keypoints[i]);
            extractor->compute(grayscale, keypoints[i], descriptors[i]);

            CV_LOG_INFO(TAG, "Found " + std::to_string(keypoints[i].size()) + " keypoints in " + i);

            if (saveDebugVisualizations) {
                cv::Mat out;
                cv::drawKeypoints(images[i], keypoints[i], out, cv::Scalar(0,0,255));
                cv::imwrite(fs::basename(fs::path(i)) + "_features.jpg", out);
            }
        }
    }

    std::vector<cv::DMatch> matchWithRatioTest(const cv::DescriptorMatcher& matcher, const cv::Mat& desc1, const cv::Mat& desc2) {
        // Raw match
        std::vector< std::vector<cv::DMatch> > nnMatch;
        matcher.knnMatch(desc1, desc2, nnMatch, 2);

        // Ratio test filter
        std::vector<cv::DMatch> ratioMatched;
        for (size_t i = 0; i < nnMatch.size(); i++) {
            cv::DMatch first = nnMatch[i][0];
            float dist1 = nnMatch[i][0].distance;
            float dist2 = nnMatch[i][1].distance;

            if (dist1 < MATCH_RATIO_THRESHOLD * dist2) {
                ratioMatched.push_back(first);
            }
        }

        return ratioMatched;
    }

    void matchFeatures() {
        CV_LOG_INFO(TAG, "Match Features");

        cv::BFMatcher matcher(cv::NORM_HAMMING);

        for (size_t i = 0; i < imagesFilenames.size() - 1; ++i) {
            for (size_t j = i + 1; j < imagesFilenames.size(); ++j) {
                const std::string imgi = imagesFilenames[i];
                const std::string imgj = imagesFilenames[j];

                // Match with ratio test filter
                std::vector<cv::DMatch> match = matchWithRatioTest(matcher, descriptors[imgi], descriptors[imgj]);

                // Reciprocity test filter
                std::vector<cv::DMatch> matchRcp = matchWithRatioTest(matcher, descriptors[imgj], descriptors[imgi]);
                std::vector<cv::DMatch> merged;
                for (const cv::DMatch& dmr : matchRcp) {
                    bool found = false;
                    for (const cv::DMatch& dm : match) {
                        // Only accept match if 1 matches 2 AND 2 matches 1.
                        if (dmr.queryIdx == dm.trainIdx and dmr.trainIdx == dm.queryIdx) {
                            merged.push_back(dm);
                            found = true;
                            break;
                        }
                    }
                    if (found) {
                        continue;
                    }
                }

                // Fundamental matrix filter
                std::vector<std::uint8_t> inliersMask(merged.size());
                std::vector<cv::Point2f> imgiPoints, imgjPoints;
                for (const auto& m : merged) {
                    imgiPoints.push_back(keypoints[imgi][m.queryIdx].pt);
                    imgjPoints.push_back(keypoints[imgj][m.trainIdx].pt);
                }
                cv::findFundamentalMat(imgiPoints, imgjPoints, inliersMask);

                std::vector<cv::DMatch> final;
                for (size_t m = 0; m < merged.size(); m++) {
                    if (inliersMask[m]) {
                        final.push_back(merged[m]);
                    }
                }

                if ((float)final.size() / (float)match.size() < PAIR_MATCH_SURVIVAL_RATE) {
                    CV_LOG_INFO(TAG, "Final match '" + imgi + "'->'" + imgj + "' has less than " + std::to_string(PAIR_MATCH_SURVIVAL_RATE) + " inliers from orignal. Skip");
                    continue;
                }

                matches[make_pair(imgi, imgj)] = final;

                CV_LOG_INFO(TAG, "Matching " + imgi + " and " + imgj + ": " + std::to_string(final.size()) + " / " + std::to_string(match.size()));

                if (saveDebugVisualizations) {
                    cv::Mat out;
                    std::vector<cv::DMatch> rawmatch;
                    matcher.match(descriptors[imgi], descriptors[imgj], rawmatch);
                    std::vector<std::pair<std::string, std::vector<cv::DMatch>& > > showList{
                            {"Raw Match", rawmatch},
                            {"Ratio Test Filter", match},
                            {"Reciprocal Filter", merged},
                            {"Epipolar Filter", final}
                    };
                    for (size_t i = 0; i< showList.size(); i++) {
                        cv::drawMatches(images[imgi], keypoints[imgi],
                                    images[imgj], keypoints[imgj],
                                    showList[i].second, out, CV_RGB(255,0,0));
                        cv::putText(out, showList[i].first, cv::Point(10,50), cv::FONT_HERSHEY_COMPLEX, 2.0, CV_RGB(255,255,255), 2);
                        cv::putText(out, "# Matches: " + std::to_string(showList[i].second.size()), cv::Point(10,100), cv::FONT_HERSHEY_COMPLEX, 1.0, CV_RGB(255,255,255));
                        cv::imwrite(fs::basename(fs::path(imgi)) + "_" + fs::basename(fs::path(imgj)) + "_" + std::to_string(i) + ".jpg", out);
                    }
                }
            }
        }
    }

    void buildTracks() {
        CV_LOG_INFO(TAG, "Build tracks");

        using namespace boost;

        struct ImageFeature {
            std::string image;
            size_t featureID;
        };
        typedef adjacency_list < listS, vecS, undirectedS, ImageFeature > Graph;
        typedef graph_traits < Graph >::vertex_descriptor Vertex;

        std::map<std::pair<std::string, int>, Vertex> vertexByImageFeature;

        Graph g;

        // Add vertices - image features
        for (const auto& imgi : keypoints) {
            for (size_t i = 0; i < imgi.second.size(); i++) {
                Vertex v = add_vertex(g);
                g[v].image = imgi.first;
                g[v].featureID = i;
                vertexByImageFeature[std::make_pair(imgi.first, i)] = v;
            }
        }

        // Add edges - feature matches
        for (const auto& match : matches) {
            for (const cv::DMatch& dm : match.second) {
                Vertex& vI = vertexByImageFeature[std::make_pair(match.first.first, dm.queryIdx)];
                Vertex& vJ = vertexByImageFeature[std::make_pair(match.first.second, dm.trainIdx)];
                add_edge(vI, vJ, g);
            }
        }

        using Filtered  = filtered_graph<Graph, keep_all, std::function<bool(Vertex)>>;
        Filtered gFiltered(g, keep_all{}, [&g](Vertex vd) { return degree(vd, g) > 0; });

        // Get connected components
        std::vector<int> component(num_vertices(gFiltered), -1);
        int num = connected_components(gFiltered, &component[0]);
        std::map<int, std::vector<Vertex> > components;
        for (size_t i = 0; i != component.size(); ++i) {
            if (component[i] >= 0) {
                components[component[i]].push_back(i);
            }
        }
        // Filter bad components (with more than 1 feature from a single image)
        std::vector<int> vertexInGoodComponent(num_vertices(gFiltered), -1);
        std::map<int, std::vector<Vertex> > goodComponents;
        for (const auto& c : components) {
            std::set<std::string> imagesInComponent;
            bool isComponentGood = true;
            for (int j = 0; j < c.second.size(); ++j) {
                const std::string imgId = g[c.second[j]].image;
                if (imagesInComponent.count(imgId) > 0) {
                    // Image already represented in this component
                    isComponentGood = false;
                    break;
                } else {
                    imagesInComponent.insert(imgId);
                }
            }
            if (isComponentGood) {
                for (int j = 0; j < c.second.size(); ++j) {
                    vertexInGoodComponent[c.second[j]] = 1;
                }
                goodComponents[c.first] = c.second;
            }
        }

        Filtered gGoodComponents(g, keep_all{}, [&vertexInGoodComponent](Vertex vd) {
            return vertexInGoodComponent[vd] > 0;
        });

        CV_LOG_INFO(TAG, "Total number of components found: " + to_string(components.size()));
        CV_LOG_INFO(TAG, "Number of good components: " + to_string(goodComponents.size()));
        const int accum = std::accumulate(goodComponents.begin(),
                                          goodComponents.end(), 0,
                                          [](int a, std::pair<const int, std::vector<Vertex> >& v){
                                                    return a+v.second.size();
                                                });
        CV_LOG_INFO(TAG, "Average component size: " + to_string((float)accum / (float)(goodComponents.size())));

        if (saveDebugVisualizations) {
            struct my_node_writer {
                my_node_writer(Graph& g_, const std::map<std::string,int>& iid_) : g (g_), iid(iid_) {};
                void operator()(std::ostream& out, Vertex v) {
                    const int imgId = iid[g[v].image];
                    out << " [label=\"" << imgId << "\" colorscheme=\"accent8\" fillcolor="<<(imgId+1)<<" style=filled]";
                };
                Graph g;
                std::map<std::string,int> iid;
            };
            std::ofstream ofs("match_graph_good_components.dot");
            write_graphviz(ofs, gGoodComponents, my_node_writer(g, imageIDs));
            std::ofstream ofsf("match_graph_filtered.dot");
            write_graphviz(ofsf, gFiltered, my_node_writer(g, imageIDs));
        }

        // Each component is a track
        const size_t nViews = imagesFilenames.size();
        tracks.resize(nViews);
        for (int i = 0; i < nViews; i++) {
            tracks[i].create(2, goodComponents.size(), CV_64FC1);
            tracks[i].setTo(-1.0);
        }
        int i = 0;
        for (auto c = goodComponents.begin(); c != goodComponents.end(); ++c, ++i) {
            for (const int v : c->second) {
                const int imageID = imageIDs[g[v].image];
                const size_t featureID = g[v].featureID;
                const cv::Point2f p = keypoints[g[v].image][featureID].pt;
                tracks[imageID].at<double>(0, i) = p.x;
                tracks[imageID].at<double>(1, i) = p.y;
            }
        }

        if (saveDebugVisualizations) {
            std::vector<cv::Scalar> colors = {CV_RGB(240, 248, 255),
                                     CV_RGB(250, 235, 215),
                                     CV_RGB(0, 255, 255),
                                     CV_RGB(127, 255, 212),
                                     CV_RGB(240, 255, 255),
                                     CV_RGB(245, 245, 220),
                                     CV_RGB(255, 228, 196),
                                     CV_RGB(255, 235, 205),
                                     CV_RGB(0, 0, 255),
                                     CV_RGB(138, 43, 226),
                                     CV_RGB(165, 42, 42),
                                     CV_RGB(222, 184, 135)};

            std::vector<cv::Mat> imagesM;
            for (const auto m : images) imagesM.push_back(m.second);
            cv::Mat out;
            cv::hconcat(std::vector<cv::Mat>(imagesM.begin(), imagesM.begin() + 4), out);
            cv::RNG& rng = cv::theRNG();
            const cv::Size imgS = imagesM[0].size();
            for (int tId = 0; tId < 20; tId++) {
                const int trackId = rng(tracks[0].cols); // Randomize a track ID

                // Show track over images
                for (int i = 0; i < 3; i++) {
                    cv::Point2f a = cv::Point2f(tracks[i].col(trackId));
                    cv::Point2f b = cv::Point2f(tracks[i + 1].col(trackId));

                    if (a.x < 0 or a.y < 0 or b.x < 0 or b.y < 0) {
                        continue;
                    }

                    const cv::Scalar c = colors[tId % colors.size()];
                    a.x += imgS.width * i;
                    b.x += imgS.width * (i + 1);
                    cv::circle(out, a, 7, c, cv::FILLED);
                    cv::circle(out, b, 7, c, cv::FILLED);
                    cv::line(out, a, b, c, 3);
                }
                cv::imwrite("tracks.jpg", out);

                // Show track patches
                const int patchSize = 20;
                const cv::Point2f patch(patchSize, patchSize);
                for (int i = 0; i < tracks.size(); i++) {
                    cv::Point2f a = cv::Point2f(tracks[i].col(trackId));
                    if (a.x < patchSize or a.y < patchSize or
                        a.x > imgS.width-patchSize or a.y > imgS.height-patchSize) {
                        continue;
                    }

                    cv::imwrite("track_" + to_string(trackId) + "_" + std::to_string(i) + ".png",
                            imagesM[i](cv::Rect(a - patch, a + patch)));
                }
            }
        }
    }

    bool reconstructFromTracks() {
        CV_LOG_INFO(TAG, "Reconstruct from " + std::to_string(tracks[0].cols) + " tracks");
        const cv::Size imgS = images.begin()->second.size();
        const float f = std::max(imgS.width,imgS.height);
        cv::Mat K = cv::Mat(cv::Matx33f{f, 0.0, imgS.width/2.0f,
                            0.0, f, imgS.height/2.0f,
                            0.0, 0.0, 1.0});
        cv::sfm::reconstruct(tracks, Rs, Ts, K, points3d, true);

        K.copyTo(K_);

        CV_LOG_INFO(TAG, "Reconstruction: ");
        CV_LOG_INFO(TAG, "Estimated 3D points: " + std::to_string(points3d.size()));
        CV_LOG_INFO(TAG, "Estimated cameras: " + std::to_string(Rs.size()));
        CV_LOG_INFO(TAG, "Refined intrinsics: ");
        CV_LOG_INFO(TAG, K_);

        if (Rs.size() != imagesFilenames.size()) {
            CV_LOG_ERROR(TAG, "Unable to reconstruct all camera views (" + std::to_string(imagesFilenames.size()) + ")");
            return false;
        }

        if (tracks[0].cols != points3d.size()) {
            CV_LOG_WARNING(TAG, "Unable to reconstruct all tracks (" + std::to_string(tracks[0].cols) + ")");
        }

        // Create the point cloud
        pointCloud.clear();
        for (const auto &p : points3d) pointCloud.emplace_back(cv::Vec3f(p));

        // Get the point colors
        pointCloudColor.resize(pointCloud.size(), cv::Vec3b(0,255,0));
        std::vector<cv::Point2f> point2d(1);
        for (int i = 0; i < (int)pointCloud.size(); i++) {
            for (int j = 0; j < imagesFilenames.size(); ++j) {
                cv::Mat point3d = cv::Mat(pointCloud[i]).reshape(1, 1);
                cv::projectPoints(point3d, Rs[j], Ts[j], K_, cv::Mat(), point2d);
                if (point2d[0].x < 0 or point2d[0].x >= imgS.width or point2d[0].y < 0 or
                    point2d[0].y >= imgS.height) {
                    continue;
                }
                pointCloudColor[i] = images[imagesFilenames[j]].at<cv::Vec3b>(point2d[0]);
                break;
            }
        }

        return true;
    }

    void visualize3D() {
        CV_LOG_INFO(TAG, "Visualize reconstruction");

        if (saveDebugVisualizations) {
            // 3d point reprojections
            cv::Mat points2d;
            cv::Mat points3dM(points3d.size(), 1, CV_32FC3);
            for (int i = 0 ; i < points3d.size(); i++) {
                points3dM.at<cv::Vec3f>(i) = cv::Vec3f(points3d[i]);
            }
            for (int j = 0; j < imagesFilenames.size(); j++) {
                cv::projectPoints(points3dM, Rs[j], Ts[j], K_, cv::noArray(), points2d);

                cv::Mat out;
                images[imagesFilenames[j]].copyTo(out);
                for (int i = 0; i < points2d.rows; i++) {
                    circle(out, points2d.at<cv::Point2f>(i), 3, CV_RGB(255, 0, 0), cv::FILLED);
                }
                cv::imwrite("reprojection_" + std::to_string(j) + ".jpg", out);
            }
        }

        // Create 3D windows
        cv::viz::Viz3d window("Coordinate Frame");
        window.setWindowSize(cv::Size(500, 500));
        window.setWindowPosition(cv::Point(150, 150));
        window.setBackgroundColor(cv::viz::Color::white());

        // Recovering cameras
        std::vector<cv::Affine3d> path;
        for (size_t i = 0; i < Rs.size(); ++i)
            path.push_back(cv::Affine3d(Rs[i],Ts[i]));

        // Add the pointcloud
        cv::viz::WCloud cloud_widget(pointCloud, pointCloudColor);
        window.showWidget("point_cloud", cloud_widget);
        // Add cameras
        window.showWidget("cameras_frames_and_lines", cv::viz::WTrajectory(path, cv::viz::WTrajectory::BOTH, 0.1, cv::viz::Color::black()));
        window.showWidget("cameras_frustums", cv::viz::WTrajectoryFrustums(path, K_, 0.1, cv::viz::Color::navy()));
        window.setViewerPose(path[0]);

        /// Wait for key 'q' to close the window
        CV_LOG_INFO(TAG, "Press 'q' to close ... ")

        window.spin();
    }

    void saveToMVSFile() {
        CV_LOG_INFO(TAG, "Save reconstruction to MVS file: " + saveMVS)

        MVS::Interface interface;
        MVS::Interface::Platform p;

        // Add camera
        MVS::Interface::Platform::Camera c;
        const cv::Size imgS = images[imagesFilenames[0]].size();
        c.K = cv::Matx33d(K_);
        c.R = cv::Matx33d::eye();
        c.C = cv::Point3d(0,0,0);
        c.name = "Camera1";
        c.width = imgS.width;
        c.height = imgS.height;
        p.cameras.push_back(c);

        // Add views
        p.poses.resize(Rs.size());
        for (size_t i = 0; i < Rs.size(); ++i) {
            cv::Mat t = -Rs[i].t() * Ts[i];
            p.poses[i].C.x = t.at<double>(0);
            p.poses[i].C.y = t.at<double>(1);
            p.poses[i].C.z = t.at<double>(2);
            cv::Mat r; Rs[i].copyTo(r);
            cv::Mat(r).convertTo(p.poses[i].R, CV_64FC1);

            // Add corresponding image
            MVS::Interface::Image image;
            image.cameraID = 0;
            image.poseID = i;
            image.name = imagesFilenames[i];
            image.platformID = 0;
            interface.images.push_back(image);
        }
        p.name = "Platform1";
        interface.platforms.push_back(p);

        // Add point cloud
        for (size_t k = 0; k < points3d.size(); ++k) {
            MVS::Interface::Color c;
            MVS::Interface::Vertex v;
            v.X = cv::Vec3f(points3d[k]);

            // Reproject to see if in image bounds and get the RGB color
            cv::Mat point3d;
            cv::Mat(points3d[k].t()).convertTo(point3d, CV_32FC1);
            for (uint32_t j = 0; j < tracks.size(); ++j) {
                std::vector<cv::Point2f> points2d(1);
                cv::projectPoints(point3d, Rs[j], Ts[j], K_, cv::Mat(), points2d);
                if (points2d[0].x < 0 or points2d[0].x > imgS.width or
                    points2d[0].y < 0 or points2d[0].y > imgS.height) {
                    continue;
                } else {
                    c.c = images[imagesFilenames[j]].at<cv::Vec3b>(points2d[0]);
                    v.views.push_back({j, 1.0});
                }
            }

            interface.verticesColor.push_back(c);
            interface.vertices.push_back(v);
        }

        MVS::ARCHIVE::SerializeSave(interface, saveMVS);
    }

    std::vector<cv::String> imagesFilenames;
    std::map<std::string, int> imageIDs;
    std::map<std::string, cv::Mat> images;
    std::map<std::string, std::vector<cv::KeyPoint> > keypoints;
    std::map<std::string, cv::Mat> descriptors;
    std::map<std::pair<std::string, std::string>, std::vector<cv::DMatch> > matches;
    std::vector<cv::Mat> Rs, Ts;
    std::vector<cv::Mat> points3d;
    std::vector<cv::Mat> tracks;
    std::vector<cv::Vec3f> pointCloud;
    std::vector<cv::Vec3b> pointCloudColor;
    cv::Matx33f K_;

    const float MATCH_RATIO_THRESHOLD = 0.8f; // Nearest neighbor matching ratio
    const float PAIR_MATCH_SURVIVAL_RATE;     // Ratio of surviving matches for a successful stereo match
    const bool visualize;                     // Show 3D visualization of the sprase cloud?
    const std::string saveMVS;                     // Save the reconstruction in MVS format for OpenMVS?
    const std::string saveCloud;                   // Save the reconstruction to a point cloud file?
    const bool saveDebugVisualizations;       // Save debug visualizations from the reconstruction process

    const std::string TAG = "StructureFromMotion";
};


int main(int argc, char** argv) {
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_DEBUG);

    cv::CommandLineParser parser(argc, argv,
                                 "{help h ? |       | help message}"
                                 "{@dir     | .     | directory with image files for reconstruction }"
                                 "{mrate    | 0.5   | Survival rate of matches to consider image pair success }"
                                 "{viz      | false | Visualize the sparse point cloud reconstruction? }"
                                 "{debug    | false | Save debug visualizations to files? }"
                                 "{mvs      |       | Save reconstruction to an .mvs file. Provide filename }"
                                 "{cloud    |       | Save reconstruction to a point cloud file (PLY, XYZ and OBJ). Provide filename}"
    );

    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }

    StructureFromMotion sfm(parser.get<std::string>("@dir"),
                            parser.get<float>("mrate"),
                            parser.get<bool>("viz"),
                            parser.get<std::string>("mvs"),
                            parser.get<std::string>("cloud"),
                            parser.get<bool>("debug")
                            );
    sfm.runSfM();

    return 0;
}
