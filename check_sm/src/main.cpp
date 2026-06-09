#include <rclcpp/rclcpp.hpp>
#include <check_sm/check_sm.hpp>

int main(int argc, char **argv)
{
    // 1. Inicializa o ambiente ROS 2
    rclcpp::init(argc, argv);

    // 2. Roda a máquina de estados (SMACC2 cuida do node e do spin internamente)
    smacc2::run<check_sm::CheckSm>();

    // 3. Encerra o ROS 2 de forma limpa quando a máquina parar
    rclcpp::shutdown();
    return 0;
}