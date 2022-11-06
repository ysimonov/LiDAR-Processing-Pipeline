#include "lidar_processing_pipeline.hpp"

#define BOOST_FILESYSTEM_VERSION 3
#define BOOST_FILESYSTEM_NO_DEPRECATED
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

namespace fs = boost::filesystem;

/**
 * @brief   Return the filenames of all files that have the specified extension
 *          in the specified directory and all subdirectories.
 */
std::vector<fs::path> readFilenamesExt(fs::path const &root, std::string const &ext)
{
    std::vector<fs::path> paths;
    if (fs::exists(root) && fs::is_directory(root))
    {
        for (const auto &entry : fs::recursive_directory_iterator(root))
        {
            if (fs::is_regular_file(entry) && entry.path().extension() == ext)
                paths.emplace_back(entry.path());
        }
    }
    return paths;
}

/**
 * @brief Main function that reads data from .pcd files and calls pipeline methods
 */
int main()
{
    // path to data folder
    std::string filepath = "../data/";

    // read file names
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<fs::path> filenames = readFilenamesExt(filepath, ".pcd");

    if (filenames.empty())
    {
        std::cerr << "Could not find data files" << std::endl;
        return EXIT_FAILURE;
    }

    // sort names lexicographically
    std::sort(filenames.begin(), filenames.end());
    std::vector<fs::path>::iterator filename_iterator = filenames.begin();

    while (1)
    {
        // check if there are any more frames to read
        if (filename_iterator == filenames.end())
        {
            break;
        }

        // read data file
        std::string filename = (*filename_iterator).string();
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1)
        {
            PCL_ERROR("Clouldn't read .pcd file\n");
            return EXIT_FAILURE;
        }
        unsigned int number_of_points = cloud->size();
        std::cout << "Loaded " << number_of_points << " points from " << filename << " file.\n";

        // process data here
        // TODO

        // increment file name iterator
        filename_iterator++;
    }

    return EXIT_SUCCESS;
}