#ifndef OWDS_COMPAT_ROS_H
#define OWDS_COMPAT_ROS_H

#if OWDS_ROS_VERSION == 1
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <ros/package.h>

// Commonly used built-in interfaces
#include <std_msgs/String.h>

// User-defined message interfaces
// #include <owds/MyMessage.h>

// User-defined service interfaces
// #include <owds/MyService.h>

namespace std_msgs_compat = std_msgs;

#elif OWDS_ROS_VERSION == 2
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// Commonly used built-in interfaces
#include <std_msgs/msg/string.hpp>

// User-defined message interfaces
// #include <owds/MyMessage.hpp>

// User-defined service interfaces
// #include <owds/MyService.hpp>

namespace std_msgs_compat = std_msgs::msg;

namespace owds::msg
{
}

namespace owds::srv
{
}

#endif

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace owds::compat
{
#if OWDS_ROS_VERSION == 1

  using namespace ::owds;

  template<typename T>
  using RawRequestType = typename T::Request;

  template<typename T>
  using RawResponseType = typename T::Response;

  template<typename T>
  using RequestType = typename T::Request;

  template<typename T>
  using ResponseType = typename T::Response;

  template<typename T, typename Request_ = typename T::Request>
  inline auto makeRequest() { return Request_(); }

  template<typename T, typename Response_ = typename T::Response>
  inline auto makeResponse() { return Response_(); }

  // todo: RequestType, ResponseType

#elif OWDS_ROS_VERSION == 2
    using namespace ::owds::msg;
    using namespace ::owds::srv;

    template <typename T>
    using RawRequestType = typename T::Request;

    template <typename T>
    using RawResponseType = typename T::Response;

    template <typename T>
    using RequestType = std::shared_ptr<typename T::Request>;

    template <typename T>
    using ResponseType = std::shared_ptr<typename T::Response>;

    template <typename T, typename Request_ = typename T::Request>
    inline auto makeRequest() { return std::make_shared<Request_>(); }

    template <typename T, typename Response_ = typename T::Response>
    inline auto makeResponse() { return std::make_shared<Response_>(); }

    // template <typename T, typename Result_ = typename T::>
#endif

    namespace owds_ros
    {
#if OWDS_ROS_VERSION == 1
        inline std::string getShareDirectory(const std::string& name) {
            return ros::package::getPath(name);
        }

    template<typename T>
    using ServiceWrapper = T;

    template<typename T>
    using MessageWrapper = typename T::ConstPtr;

    using Rate = ros::Rate;
    using RosTime = ros::Time;

    template<typename T>
    T* getServicePointer(T& service) { return &service; }

#elif OWDS_ROS_VERSION == 2
        inline std::string getShareDirectory(const std::string& name)
        {
            return ament_index_cpp::get_package_share_directory(name);
        }

        template <typename T>
        using ServiceWrapper = typename T::SharedPtr; // std::shared_ptr<T>;

        template <typename T>
        using MessageWrapper = typename T::ConstSharedPtr;

        using Rate = rclcpp::Rate;
        using RosTime = rclcpp::Time;

        using namespace ::owds::msg;
        using namespace ::owds::srv;

        template <typename T>
        T& getServicePointer(T& service) { return service; }
#endif

        template <typename T>
        class Publisher;

        template <typename T>
        class Subscriber;

        template <typename T>
        class Service;

        template <typename T>
        class Client;

        class Time : public RosTime
        {
        public:
            Time(uint32_t sec, uint32_t nsec) : RosTime((int32_t)sec, (int32_t)nsec)
            {
            }

            explicit Time(int64_t t) : RosTime((uint32_t)t, (uint32_t)((t - std::floor(t)) * 1'000'000'000.))
            {
            }

            Time(const RosTime& time) : RosTime(time)
            {
            } // do not put it as explicit

            uint32_t seconds() const
            {
#if OWDS_ROS_VERSION == 1
        return sec;
#elif OWDS_ROS_VERSION == 2
                return (uint32_t)RosTime::seconds();
#endif
            }

            uint32_t nanoseconds() const
            {
#if OWDS_ROS_VERSION == 1
        return nsec;
#elif OWDS_ROS_VERSION == 2
                return RosTime::nanoseconds();
#endif
            }
        };

        class Node
        {
        public:
            template <typename T>
            friend class Publisher;

            template <typename T>
            friend class Subscriber;

            template <typename T>
            friend class Service;

            template <typename T>
            friend class Client;

            Node(Node& other) = delete;
            Node(Node&& other) = delete;
            ~Node() = default;

            static Node& get();
            static bool ok();

            static void init(int argc, char** argv, const std::string& node_name);
            static void shutdown();

            void spin();

            Time currentTime();

        private:
            explicit Node(const std::string& node_name);

            const std::string name_;

#if OWDS_ROS_VERSION == 1
      ros::NodeHandle handle_;
      ros::CallbackQueue callback_queue_;
#elif OWDS_ROS_VERSION == 2
            rclcpp::Node::SharedPtr handle_;
            std::thread ros_thread_;
#endif

            bool running_;
        };

        template <typename T>
        class Publisher
        {
        public:
            Publisher(const std::string& topic_name, std::size_t queue_size)
            {
                auto& node = Node::get();

#if OWDS_ROS_VERSION == 1
        handle_ = node.handle_.advertise<T>(topic_name, queue_size);
#elif OWDS_ROS_VERSION == 2
                (void)queue_size;
                handle_ = node.handle_->create_publisher<T>(topic_name, 10);
#endif
            }

            void publish(const T& message)
            {
#if OWDS_ROS_VERSION == 1
        handle_.publish(message);
#elif OWDS_ROS_VERSION == 2
                handle_->publish(message);
#endif
            }

            size_t getNumSubscribers()
            {
#if OWDS_ROS_VERSION == 1
        return handle_.getNumSubscribers();
#elif OWDS_ROS_VERSION == 2
                return handle_->get_subscription_count();
#endif
            }

        private:
#if OWDS_ROS_VERSION == 1
      ros::Publisher handle_;
#elif OWDS_ROS_VERSION == 2
            typename rclcpp::Publisher<T>::SharedPtr handle_;
#endif
        };

        template <typename T>
        class Subscriber
        {
        public:
            template <typename Ta, typename Tb>
            Subscriber(const std::string& topic_name, std::size_t queue_size, Ta&& callback, Tb&& ptr)
            {
                auto& node = Node::get();

#if OWDS_ROS_VERSION == 1
        handle_ = node.handle_.subscribe(topic_name, queue_size, callback, ptr);
#elif OWDS_ROS_VERSION == 2
                (void)queue_size;
                handle_ = node.handle_->create_subscription<T>(topic_name, 10,
                                                               std::bind(std::forward<Ta>(callback), ptr,
                                                                         std::placeholders::_1));
#endif
            }

        private:
#if OWDS_ROS_VERSION == 1
      ros::Subscriber handle_;
#elif OWDS_ROS_VERSION == 2
            typename rclcpp::Subscription<T>::SharedPtr handle_;
#endif
        };

        template <typename T>
        class Service
        {
        public:
            template <typename Ta>
            Service(const std::string& service_name, Ta&& callback)
            {
                auto& node = Node::get();

#if OWDS_ROS_VERSION == 1
        handle_ = node.handle_.advertiseService(service_name, callback);
#elif OWDS_ROS_VERSION == 2
                handle_ = node.handle_->create_service<T>(service_name,
                                                          [&](compat::owds_ros::ServiceWrapper<typename T::Request> req,
                                                              compat::owds_ros::ServiceWrapper<typename T::Response>
                                                              res)
                                                          {
                                                              callback(req, res);
                                                          });
                // handle_ = node.handle_->create_service<T>(service_name, callback);
#endif
            }

            template <typename Ta, typename Tb>
            Service(const std::string& service_name, Ta&& callback, Tb&& ptr)
            {
                auto& node = Node::get();

#if OWDS_ROS_VERSION == 1
        handle_ = node.handle_.advertiseService(service_name, callback, ptr);
#elif OWDS_ROS_VERSION == 2
                handle_ = node.handle_->create_service<T>(service_name,
                                                          [ptr, callback](
                                                          compat::owds_ros::ServiceWrapper<typename T::Request> req,
                                                          compat::owds_ros::ServiceWrapper<typename T::Response> res)
                                                          {
                                                              (ptr->*callback)(req, res);
                                                          });
                // handle_ = node.handle_->create_service<T>(service_name, std::bind(std::forward<Ta>(callback), ptr, std::placeholders::_1, std::placeholders::_2));
#endif
            }

        private:
#if OWDS_ROS_VERSION == 1
      ros::ServiceServer handle_;
#elif OWDS_ROS_VERSION == 2
            typename rclcpp::Service<T>::SharedPtr handle_;
#endif
        };

        template <typename T>
        class Client
        {
        public:
            enum class Status_e
            {
                ros_status_successful,
                ros_status_successful_with_retry,
                ros_status_failure
            };

            explicit Client(const std::string& service_name) : name_(service_name)
            {
                auto& node = Node::get();

#if OWDS_ROS_VERSION == 1
        handle_ = node.handle_.serviceClient<T>(service_name, true);
#elif OWDS_ROS_VERSION == 2
                handle_ = node.handle_->create_client<T>(service_name);
#endif
            }

            Status_e call(const owds::compat::RequestType<T>& req, owds::compat::ResponseType<T>& res)
            {
                using namespace std::chrono_literals;
                auto status = Status_e::ros_status_failure;

#if OWDS_ROS_VERSION == 1
        T srv;
        srv.request = req;
        if(!handle_.call(srv))
        {
          auto& node = Node::get();
          handle_ = node.handle_.serviceClient<T>(name_, true);
          if(handle_.call(srv))
          {
            status = Status_e::ros_status_successful_with_retry;
            res = srv.response;
          }
        }
        else
        {
          status = Status_e::ros_status_successful;
          res = srv.response;
        }

#elif OWDS_ROS_VERSION == 2
                if (!handle_->wait_for_service(5s))
                {
                    return status;
                }

                auto future = handle_->async_send_request(req);

                if (future.wait_for(5s) == std::future_status::ready)
                {
                    status = Status_e::ros_status_successful;
                    res = future.get();
                }
#endif
                return status;
            }

            bool wait(double timeout)
            {
#if OWDS_ROS_VERSION == 1
        return handle_.waitForExistence(ros::Duration(timeout));
#elif OWDS_ROS_VERSION == 2
                return handle_->wait_for_service(std::chrono::duration<double>(timeout));
#endif
            }

        private:
            std::string name_;
#if OWDS_ROS_VERSION == 1
      ros::ServiceClient handle_;
#elif OWDS_ROS_VERSION == 2
            typename rclcpp::Client<T>::SharedPtr handle_;
#endif
        };
    } // namespace owds_ros
} // namespace owds::compat

#endif // OWDS_COMPAT_ROS_H
