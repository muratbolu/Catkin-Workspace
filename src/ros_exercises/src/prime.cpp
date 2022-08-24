#include <ros_exercises/prime.h>

prime_application::prime_application(): calculator_rate(500),
                                        printer_rate(1),
                                        publisher_rate(500) {

    prime_pub = node.advertise<ros_exercises::primes_list>("prime_topic", 1000);
    calculator_thread = std::thread(&prime_application::calculator_function, this);
    printer_thread = std::thread(&prime_application::printer_function, this);
    publisher_thread = std::thread(&prime_application::publisher_function, this);

}

prime_application::~prime_application() {

    calculator_thread.join();
    printer_thread.join();
    publisher_thread.join();

}

void prime_application::calculator_function() {

    for (; ros::ok();) {
        if (mutex.try_lock()) {
            primes_lst.data.push_back(calculator.next_prime());
            mutex.unlock();
        }
        calculator_rate.sleep();
    }

}

void prime_application::printer_function() {

    for (; ros::ok();) {
        if (mutex.try_lock()) {
            ROS_INFO("%s", (print_array::to_string(primes_lst)).c_str());
            mutex.unlock();
        }
        printer_rate.sleep();
    }

}

void prime_application::publisher_function() {

    for (; ros::ok();) {
        if (mutex.try_lock()) {
            prime_pub.publish(primes_lst);
            mutex.unlock();
        }
        publisher_rate.sleep();
    }

}
