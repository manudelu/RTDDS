#include "sensor_msgs/msg/JointStatePubSubTypes.hpp"

#include <chrono>
#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <atomic>
#include <csignal>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

std::atomic<bool> running {true};

void sigint_handler(int)
{
    running = false;
}

class JointStateSubscriber
{
private:

    DomainParticipant* participant_;
    Subscriber* subscriber_;
    DataReader* reader_;
    Topic* topic_;
    TypeSupport type_;

    class SubListener : public DataReaderListener
    {
    public:

        SubListener() 
            : message_count_{0}, matched_{0}
        {}
        
        ~SubListener() override {}

        void on_subscription_matched(DataReader* reader, const SubscriptionMatchedStatus& info) override
        {
            if (reader == nullptr)
                return;

            if (info.current_count_change == 1)
            {
                matched_ += info.current_count_change;
                std::cout << "Subscriber matched. Total publishers: " << matched_ << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ += info.current_count_change;
                std::cout << "Subscriber unmatched. Total publishers: " << matched_ << std::endl;
            }

            if (matched_ < 0) matched_ = 0;
        }

        void on_data_available(DataReader* reader) override
        {
            if (reader == nullptr)
                return;
            
            SampleInfo info;
            sensor_msgs::msg::JointState joint_state;

            while (reader->take_next_sample(&joint_state, &info) == RETCODE_OK)
            {
                if (info.valid_data)
                {
                    int msg_id = ++message_count_;
                    if (msg_id % 100 != 0)
                        continue;

                    std::cout << "header:" << std::endl;
                    std::cout << "  stamp:" << std::endl;
                    std::cout << "    sec: " << joint_state.header().stamp().sec() << std::endl;
                    std::cout << "    nanosec: " << joint_state.header().stamp().nanosec() << std::endl;
                    std::cout << "  frame_id: ";
                    if (joint_state.header().frame_id().empty()) 
                        std::cout << "''" << std::endl;
                    else
                        std::cout << joint_state.header().frame_id() << std::endl;
                    
                    std::cout << "name:" << std::endl;
                    for (const auto& name : joint_state.name())
                    {
                        std::cout << "- " << name << std::endl;
                    }

                    std::cout << "position:" << std::endl;
                    for (const auto& position : joint_state.position())
                    {
                        if (position == 0.0)
                            std::cout << "- 0.0" << std::endl;
                        else
                            std::cout << "- " << position << std::endl;
                    }

                    std::cout << "velocity:" << std::endl;
                    for (const auto& velocity : joint_state.velocity())
                    {
                        if (velocity == 0.0)
                            std::cout << "- 0.0" << std::endl;
                        else
                            std::cout << "- " << velocity << std::endl;
                    }

                    std::cout << "effort:" << std::endl;
                    for (const auto& effort : joint_state.effort())
                    {
                        if (effort == 0.0)
                            std::cout << "- 0.0" << std::endl;
                        else
                            std::cout << "- " << effort << std::endl;
                    }
        
                    std::cout << "---" << std::endl;
                }
            }
        }

    private:
        std::atomic_int message_count_;
        std::atomic_int matched_;
    }
    listener_;

public:

    JointStateSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new sensor_msgs::msg::JointStatePubSubType())
    {
    }

    virtual ~JointStateSubscriber()
    {
        cleanup();
    }

    void cleanup()
    {
        if (reader_ != nullptr)
        {
            if (subscriber_ != nullptr)
            {
                subscriber_->delete_datareader(reader_);
            }
            reader_ = nullptr;
        }
        
        if (topic_ != nullptr)
        {
            if (participant_ != nullptr)
            {
                participant_->delete_topic(topic_);
            }
            topic_ = nullptr;
        }
        
        if (subscriber_ != nullptr)
        {
            if (participant_ != nullptr)
            {
                participant_->delete_subscriber(subscriber_);
            }
            subscriber_ = nullptr;
        }
        
        if (participant_ != nullptr)
        {
            DomainParticipantFactory::get_instance()->delete_participant(participant_);
            participant_ = nullptr;
        }

        std::cout << "Cleanup completed." << std::endl;
    }

    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_subscriber");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);
        
        if (participant_ == nullptr)
        {
            std::cerr << "Failed to create participant" << std::endl;
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the subscriptions Topic
        topic_ = participant_->create_topic("rt/joint_states", type_.get_type_name(), TOPIC_QOS_DEFAULT);
        if (topic_ == nullptr)
        {
            std::cerr << "Failed to create topic" << std::endl;
            cleanup();
            return false;
        }

        // Create the Subscriber
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);
        if (subscriber_ == nullptr)
        {
            std::cerr << "Failed to create subscriber" << std::endl;
            cleanup();
            return false;
        }

        // Create the DataReader
        DataReaderQos reader_qos;
        reader_qos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        reader_qos.history().kind = KEEP_LAST_HISTORY_QOS;
        reader_qos.history().depth = 1;
        reader_qos.resource_limits().max_samples = 1;
        reader_qos.resource_limits().allocated_samples = 1;
        reader_ = subscriber_->create_datareader(topic_, reader_qos, &listener_);
        if (reader_ == nullptr)
        {
            std::cerr << "Failed to create datareader" << std::endl;
            cleanup();
            return false;
        }

        return true;
    }

    void run()
    {
        std::cout << "Waiting for data..." << std::endl;
        
        while (running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

};

int main(int argc, char** argv)
{
    std::signal(SIGINT, sigint_handler);
    
    JointStateSubscriber* sub = new JointStateSubscriber();
    
    if (!sub->init())
    {
        std::cerr << "Failed to initialize subscriber!" << std::endl;
        return 1;
    }

    sub->run();
    
    delete sub;
    return 0;
}