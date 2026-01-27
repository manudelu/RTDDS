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
        ~PubListener() override {}

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
        std::cout << "Publisher cleanup completed." << std::endl;
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
    }

    bool init(const std::vector<std::string>& joint_names)
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(42, participantQos);
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
    snprintf(device_path, sizeof(device_path), 
             "/proc/xenomai/registry/rtipc/xddp/%s", XDDP_PORT_LABEL);
    
    // Wait for device with timeout
    int xddp_fd = -1;
    int retry_count = 0;
    const int max_retries = 50;
    
    std::cout << "Waiting for Xenomai RT server..." << std::endl;
    
    while (xddp_fd < 0 && retry_count < max_retries && running) {
        xddp_fd = open(device_path, O_RDONLY);
        if (xddp_fd < 0) {
            if (errno == ENOENT) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                retry_count++;
            } else {
                std::cerr << "Failed to open XDDP device: " << strerror(errno) << std::endl;
                return 1;
            }
        }
    }
    
    if (xddp_fd < 0) {
        std::cerr << "Timeout waiting for XDDP device" << std::endl;
        return 1;
    }
    
    std::cout << "Connected to XDDP device!" << std::endl;

    JointStatePublisher* pub = new JointStatePublisher();
    if (!pub->init(robot.joint_names)) {
        std::cerr << "Failed to initialize publisher\n";
        close(xddp_fd);
        return 1;
    }
    
    struct rt_joint_state_msg msg;
    std::vector<double> positions(N_JOINTS);
    std::vector<double> velocities(N_JOINTS);
    
    uint64_t last_seq = 0;
    auto last_print = std::chrono::steady_clock::now();

    std::cout << "Publishing joint states..." << std::endl;

    while (running)
    {
        int ret = read(xddp_fd, &msg, sizeof(msg));
        if (ret <= 0) {
            if (errno == EINTR) continue;
            std::cerr << "XDDP read error: " << strerror(errno) << std::endl;
            break;
        }

        // Check for dropped messages
        if (msg.seq != last_seq + 1 && last_seq != 0) {
            std::cerr << "Dropped " << (msg.seq - last_seq - 1) 
                      << " messages" << std::endl;
        }
        last_seq = msg.seq;

        // Periodic status
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_print).count() >= 2) {
            std::cout << "seq=" << msg.seq << " publishing OK" << std::endl;
            last_print = now;
        }

        for (int i = 0; i < N_JOINTS; ++i) {
            positions[i] = msg.positions[i];
            velocities[i] = msg.velocities[i];
        }

        pub->publish(positions, velocities, msg.timestamp_ns);
    }

    close(xddp_fd);
    delete pub;
    return 0;
}
