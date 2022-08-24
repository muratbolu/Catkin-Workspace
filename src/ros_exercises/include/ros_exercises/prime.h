#include <ros/ros.h>
#include <ros_exercises/primes_list.h>
#include <ros_exercises/print_array.h>
#include <vector>
#include <thread>
#include <mutex>

class primes {

    private:
        long unsigned int index;
        std::vector<long unsigned int> primes_vector;

    public:
        primes() {
            index = 1;
            primes_vector.push_back(2);
        }
        long unsigned int next_prime() {
            if (index == 1) {
                index++;
                return 2;
            }
            else {
                bool flag;
                for (index++;; index++) {
                    flag = true;
                    for (int i = 0; primes_vector[i]*primes_vector[i] <= index; i++) {
                        if (!(index % primes_vector[i])) {
                            flag = false;
                            break;
                        }
                    }
                    if (flag) break;
                }
                primes_vector.push_back(index);
                return index;
            }
        }
};

class prime_application {

    private:
        std::thread calculator_thread;
        std::thread printer_thread;
        std::thread publisher_thread;
        std::mutex mutex;
        primes calculator;
        ros::NodeHandle node;
        ros::Publisher prime_pub;
        ros::Rate calculator_rate;
        ros::Rate printer_rate;
        ros::Rate publisher_rate;
        ros_exercises::primes_list primes_lst;

    public:
        prime_application();
        ~prime_application();
        void calculator_function();
        void printer_function();
        void publisher_function();

};
