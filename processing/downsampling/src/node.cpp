#include "node.hpp"

#include <chrono>
#include <cinttypes>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

namespace downsampling
{
Subscriber::Subscriber(const std::string &name, const std::string &input)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))

{
    // Create a subscription on the input topic which prints on receipt of new messages.
    sub_ = this->create_subscription<std_msgs::msg::Int32>(input, 10, [](std_msgs::msg::Int32::UniquePtr msg) {
        printf(" Received message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
               reinterpret_cast<std::uintptr_t>(msg.get()));
    });
}

Publisher::Publisher(const std::string &name, const std::string &output)
    : Node(name, rclcpp::NodeOptions().use_intra_process_comms(false))
{
    // Create a publisher on the output topic.
    pub_ = this->create_publisher<std_msgs::msg::Int32>(output, 10);
    std::weak_ptr<std::remove_pointer<decltype(pub_.get())>::type> captured_pub = pub_;
    // Create a timer which publishes on the output topic at ~1Hz.
    auto callback = [captured_pub]() -> void {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr)
        {
            return;
        }
        static int32_t count = 0;
        std_msgs::msg::Int32::UniquePtr msg(new std_msgs::msg::Int32());
        msg->data = count++;
        printf("Published message with value: %d, and address: 0x%" PRIXPTR "\n", msg->data,
               reinterpret_cast<std::uintptr_t>(msg.get()));
        pub_ptr->publish(std::move(msg));
    };
    timer_ = this->create_wall_timer(1s, callback);
}
} // namespace downsampling

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;

    std::string subscriber_node_name = "downsampling_subscriber";
    std::string subscriber_node_topic = "number";
    std::shared_ptr<downsampling::Subscriber> subscriber =
        std::make_shared<downsampling::Subscriber>(subscriber_node_name, subscriber_node_topic);
    executor.add_node(subscriber);

    std::string publisher_node_name = "downsampling_publisher";
    std::string publisher_node_topic = "number";
    std::shared_ptr<downsampling::Publisher> publisher =
        std::make_shared<downsampling::Publisher>(publisher_node_name, publisher_node_topic);
    executor.add_node(publisher);

    executor.spin();

    rclcpp::shutdown();

    return 0;
}