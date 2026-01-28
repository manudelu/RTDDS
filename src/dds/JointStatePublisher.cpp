#include "sensor_msgs/msg/JointStatePubSubTypes.hpp"
#include "JointStateMsg.hpp"
#include "RobotModel.hpp"

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

#define XDDP_PORT_LABEL "xddp-joint-state"

using namespace eprosima::fastdds::dds;

std::atomic<bool> running {true};

void sigint_handler(int)
{
    running = false;
}

int open_xddp_device(const char* device_path) {
    int xddp_fd = -1;
    std::cout << "Waiting for Xenomai RT server..." << std::endl;

    while (running) {
        xddp_fd = open(device_path, O_RDONLY);
        if (xddp_fd >= 0) break;

        if (errno == ENOENT) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        } else if (errno != EINTR) {
            std::cerr << "Failed to open XDDP device: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1)); 
        }
    }

    if (xddp_fd >= 0) {
        std::cout << "Connected to XDDP device!" << std::endl;
    }

    return xddp_fd;
}

class JointStatePublisher
{
private:
    sensor_msgs::msg::JointState joint_state_;
    DomainParticipant* participant_;
    Publisher* publisher_;
    Topic* topic_;
    DataWriter* writer_;
    TypeSupport type_;

    class PubListener : public DataWriterListener
    {
    private:
        std::atomic_int matched_;

    public:
        PubListener() : matched_{0} {}
        ~PubListener() override = default;

        void on_publication_matched(DataWriter* writer, const PublicationMatchedStatus& info) override
        {
            if (writer == nullptr) return;
        
            if (info.current_count_change == 1)
            {
                matched_ += info.current_count_change;
                std::cout << "Publisher matched. Total subscribers: " << matched_ << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ += info.current_count_change;
                std::cout << "Publisher unmatched. Total subscribers: " << matched_ << std::endl;
            }
        }

        int get_matched() const { return matched_; }

    } listener_;

    void cleanup() {
        if (writer_ != nullptr) {
            publisher_->delete_datawriter(writer_);
            writer_ = nullptr;
        }
        if (publisher_ != nullptr) {
            participant_->delete_publisher(publisher_);
            publisher_ = nullptr;
        }
        if (topic_ != nullptr) {
            participant_->delete_topic(topic_);
            topic_ = nullptr;
        }
        if (participant_ != nullptr) {
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }
    }

public:
    JointStatePublisher()
        : participant_(nullptr)
        , publisher_(nullptr)
        , topic_(nullptr)
        , writer_(nullptr)
        , type_(new sensor_msgs::msg::JointStatePubSubType())
    {
    }

    virtual ~JointStatePublisher()
    {
        cleanup();
        std::cout << "Publisher cleanup completed." << std::endl;
    }

    bool init(const std::vector<std::string>& joint_names, int domain_id = 0)
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(domain_id, participantQos);
        if (participant_ == nullptr) {
            std::cerr << "Failed to create participant" << std::endl;
            return false;
        }

        type_.register_type(participant_);
        topic_ = participant_->create_topic("rt/joint_states", type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (topic_ == nullptr) {
            std::cerr << "Failed to create topic" << std::endl;
            cleanup();
            return false;
        }

        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
        if (publisher_ == nullptr) {
            std::cerr << "Failed to create publisher" << std::endl;
            cleanup();
            return false;
        }  

        DataWriterQos writer_qos;
        writer_qos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;  
        writer_qos.history().kind = KEEP_LAST_HISTORY_QOS;
        writer_qos.history().depth = 1; 
        writer_qos.resource_limits().max_samples = 1;
        writer_qos.resource_limits().allocated_samples = 1;
        
        writer_ = publisher_->create_datawriter(topic_, writer_qos, &listener_);
        if (writer_ == nullptr) {
            std::cerr << "Failed to create datawriter" << std::endl;
            cleanup();
            return false;
        }

        joint_state_.name() = joint_names;
        joint_state_.position().resize(joint_names.size(), 0.0);
        joint_state_.velocity().resize(joint_names.size(), 0.0);
        joint_state_.effort().resize(joint_names.size(), 0.0);
        joint_state_.header().frame_id() = "";

        return true;
    }

    void publish(const std::vector<double>& positions, 
                 const std::vector<double>& velocities,
                 uint64_t rt_timestamp_ns)
    {
        if (listener_.get_matched() <= 0)
            return;

        joint_state_.position() = positions;
        joint_state_.velocity() = velocities;

        // Use RT timestamp
        joint_state_.header().stamp().sec() = rt_timestamp_ns / 1000000000ULL;
        joint_state_.header().stamp().nanosec() = rt_timestamp_ns % 1000000000ULL;

        writer_->write(&joint_state_);
    }
};

int main(int argc, char** argv)
{
    std::signal(SIGINT, sigint_handler);
    
    char device_path[256];
    snprintf(device_path, sizeof(device_path), "/proc/xenomai/registry/rtipc/xddp/%s", XDDP_PORT_LABEL);
    int xddp_fd = open_xddp_device(device_path);
    if (xddp_fd < 0) return 1;

    std::vector<std::string> joint_names_vec(
        spot.joint_names, spot.joint_names + SpotRobot::dofs
    );

    JointStatePublisher pub;
    if (!pub.init(joint_names_vec, 42)) {
        std::cerr << "Failed to initialize publisher\n";
        close(xddp_fd);
        return 1;
    }
    
    SpotJointStateMsg msg;
    std::vector<double> positions(SpotRobot::dofs);
    std::vector<double> velocities(SpotRobot::dofs);
    
    std::cout << "Publishing joint states..." << std::endl;

    while (running)
    {
        int ret = read(xddp_fd, &msg, sizeof(msg));
        if (ret <= 0) {
            if (errno == EINTR) continue;

            close(xddp_fd);
            xddp_fd = open_xddp_device(device_path);
            if (xddp_fd < 0) {
                break;
            }
            continue;
        }

        for (int i = 0; i < SpotRobot::dofs; ++i) {
            positions[i] = msg.positions[i];
            velocities[i] = msg.velocities[i];
        }

        pub.publish(positions, velocities, msg.timestamp_ns);
    }

    close(xddp_fd);
    return 0;
}