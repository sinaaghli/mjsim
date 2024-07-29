#ifndef MJSIM_DATA_H__
#define MJSIM_DATA_H__

#include <memory>
#include <mutex>
#include <string>
#include <fstream>
#include <filesystem>

class mjsim_data {
public:
    mjsim_data(const std::string& dir){
        storatge_dir_ = dir;
    }

    void UpdateURDFPath(const std::string& urdf_path)
    {
        std::lock_guard<std::mutex> lock(urdf_mutex_);
        urdf_file_path_ = std::make_shared<std::string>(urdf_path);
    }

    std::shared_ptr<std::string> GetURDFFilePath()
    {
        std::lock_guard<std::mutex> lock(urdf_mutex_);
        auto tmp = urdf_file_path_;
        urdf_file_path_.reset();
        return tmp;
    }

private:
    std::mutex urdf_mutex_;
    std::string storatge_dir_;
    std::shared_ptr<std::string> urdf_file_path_;
};

#endif // MJSIM_DATA_H__