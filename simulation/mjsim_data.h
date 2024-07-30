#ifndef MJSIM_DATA_H__
#define MJSIM_DATA_H__

#include <memory>
#include <mutex>
#include <string>
#include <fstream>
#include <filesystem>

class mjsim_data {
public:
    struct jointstate{
        jointstate():
            jname(""),
            jposition(0),
            jvelocity(0.0),
            jtorque(0.0){};
        std::string jname;
        double jposition;
        double jvelocity;
        double jtorque;
    };

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

    void UpdateJointStates(const std::vector<jointstate>& joint_states){
        std::lock_guard<std::mutex> lock(jstate_mutex_);
        jstatevec_ = joint_states;
    }

    std::vector<jointstate> GetJointStates(){
        std::lock_guard<std::mutex> lock(jstate_mutex_);
        return jstatevec_;
    }

private:
    std::mutex urdf_mutex_;
    std::string storatge_dir_;
    std::shared_ptr<std::string> urdf_file_path_;

    std::mutex jstate_mutex_;
    std::vector<jointstate> jstatevec_;
};

#endif // MJSIM_DATA_H__