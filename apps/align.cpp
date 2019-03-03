#include <iostream>
// #include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pclomp/ndt_omp.h>

#include <boost/filesystem.hpp>
#include <boost/timer.hpp>
#include <vector>
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>

#include <boost/program_options.hpp>

#include <boost/circular_buffer.hpp>
#include <pcl/filters/extract_indices.h>

namespace fsys = boost::filesystem;
namespace po = boost::program_options;

bool process_command_line(int argc, char** argv,
                          std::string& dir,
                          std::string& out)
{
    try
    {
        po::options_description desc("Program Usage", 1024, 512);
        desc.add_options()
                ("help",     "produce help message")
                ("dir",   po::value<std::string>(&dir),      "set the pcd folder path")
                ("out",   po::value<std::string>(&out),       "set the output pcd name")
                ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << "\n";
            return false;
        }

        // There must be an easy way to handle the relationship between the
        // option "help" and "host"-"port"-"config"
        // Yes, the magic is putting the po::notify after "help" option check
        po::notify(vm);
    }
    catch(std::exception& e)
    {
        std::cerr << "Error: " << e.what() << "\n";
        return false;
    }
    catch(...)
    {
        std::cerr << "Unknown error!" << "\n";
        return false;
    }

    return true;
}

bool bGetFileList(
    std::string strFolderPath, 
    std::vector<std::string> &vecFileList)
{
    vecFileList.erase(vecFileList.begin(), vecFileList.end());
    std::vector<fsys::path> flist;
    bool bReturn = false;
    fsys::path p(strFolderPath);
    if (is_directory(p))
    {
        bReturn = true;
        std::copy(fsys::directory_iterator(p), fsys::directory_iterator(), std::back_inserter(flist));
        std::sort(flist.begin(), flist.end());

        for (auto& it : flist)
        {
            vecFileList.push_back(fsys::canonical(it).string());
        }
    }
    else
    {
        bReturn = false;
    }
    return bReturn;
}



int main(int argc, char **argv)
{
    std::string pcdFolderPath = "../../pcd_20190214_zmp_around";
    std::string pcdOutputName = "test_pcd.pcd";

    bool result = process_command_line(argc, argv, pcdFolderPath, pcdOutputName);
    if (!result)
    {
        return 1;
    }



    /// Get file list
    std::vector<std::string> vecFileList;
    if(!bGetFileList(pcdFolderPath, vecFileList))
    {
        std::cout << "Folder not found" << std::endl;
        exit(1);
    }


    /// NDT_omp
    int num_threads = omp_get_max_threads();
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setResolution(1.0);//trans e =0.0001
    ndt_omp->setTransformationEpsilon (0.0001);
    ndt_omp->setNumThreads(num_threads);
    ndt_omp->setNeighborhoodSearchMethod(pclomp::KDTREE);

    /// Get first pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if(pcl::io::loadPCDFile(vecFileList[0], *target_cloud)) {
        std::cerr << "failed to load " << vecFileList[0] << std::endl;
        return 0;
    }
    Eigen::Matrix4f guess = Eigen::Matrix4f::Identity ();

    std::vector<Eigen::Matrix4f> acc_poses;
    acc_poses.push_back(guess);
    std::vector<Eigen::Matrix4f> edge_pose;

    const int cloud_list_max_size = 300;
    boost::circular_buffer< pcl::PointCloud<pcl::PointXYZ> > cloud_list(cloud_list_max_size);
    int cloud_list_size = 0;



    boost::posix_time::ptime start = boost::posix_time::second_clock::local_time();
    int count = 0;

    for (auto it = vecFileList.begin() + 1 ; it != vecFileList.end(); it++)
    {
        std::string source_pcd = *it;

        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        if(pcl::io::loadPCDFile(source_pcd, *source_cloud)) {
          std::cerr << "failed to load " << source_pcd << std::endl;
          return 0;
        }

        /// Downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        voxelgrid.setLeafSize(0.2f, 0.2f, 0.2f);

        voxelgrid.setInputCloud(target_cloud);
        voxelgrid.filter(*downsampled);

        if(0 == count) {
            *target_cloud = *downsampled;
            cloud_list.push_back(*target_cloud);
            cloud_list_size++;
        }


        voxelgrid.setInputCloud(source_cloud);
        voxelgrid.filter(*downsampled);
        source_cloud = downsampled;




        /// Registration
        ndt_omp->setInputTarget(target_cloud);
        ndt_omp->setInputSource(source_cloud);

        pcl::PointCloud<pcl::PointXYZ> tmp_cloud;
        ndt_omp->align(tmp_cloud, guess);

        edge_pose.push_back(guess.inverse() * ndt_omp->getFinalTransformation());
        guess = ndt_omp->getFinalTransformation();
        acc_poses.push_back(guess);



        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::transformPointCloud (*source_cloud, transformed_cloud, guess);


        cloud_list_size++;
        /// Add point cloud
        //*target_cloud += transformed_cloud;
        if(cloud_list_size < cloud_list_max_size)
        {
            *target_cloud += transformed_cloud;
        } else { // remove front
            *target_cloud += transformed_cloud;
            //*target_cloud -= cloud_list.front();
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(target_cloud);


            pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
            for (int i = 0; i < cloud_list.front().size(); i++)
            {
                inliers->indices.push_back(i);
            }
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*target_cloud);
        }

        cloud_list.push_back(transformed_cloud);




        /// Time elapsed
        boost::posix_time::ptime end = boost::posix_time::second_clock::local_time();
        boost::posix_time::time_duration dur = end - start;
        int elapsed_min = dur.minutes();
        int elapsed_sec = dur.seconds() % 60;
        /// Time remain
        double timeForOneLoop = ((double)dur.seconds())/((double)count);
        double loopRemained = vecFileList.size() - count;
        int remain = timeForOneLoop * loopRemained;

        std::cout << '\r'
                  << (count++)+1 << " / " << vecFileList.size()
                  << "  time elapsed:" <<  elapsed_min << "min "
                  << elapsed_sec << "sec, Remain: "
                  << remain/60 << "min" << remain % 60 << "sec" << std::flush;

        if (count > 10)
        {
            break;
        }

    }
    pcl::io::savePCDFileASCII (pcdOutputName, *target_cloud);
    std::cerr << "\nSaved " << target_cloud->points.size () << " data points to test_pcd.pcd." << std::endl;


    return 0;
}