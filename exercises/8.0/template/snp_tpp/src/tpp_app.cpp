#include <snp_tpp/tpp_widget.h>

#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

void handleSignal(int)
{
  QApplication::instance()->quit();
}

int main(int argc, char* argv[])
{
  try
  {
    rclcpp::init(argc, argv);

    QApplication app(argc, argv);

    // Handle signals to terminate the Qt Application
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);

    auto node = std::make_shared<rclcpp::Node>("snp_tpp_app");

    boost_plugin_loader::PluginLoader loader;
    loader.search_libraries.insert(NOETHER_GUI_PLUGINS);
    loader.search_libraries.insert(SNP_TPP_GUI_PLUGINS);
    loader.search_libraries_env = NOETHER_GUI_PLUGIN_LIBS_ENV;
    loader.search_paths_env = NOETHER_GUI_PLUGIN_PATHS_ENV;

    snp_tpp::TPPWidget w(node, std::move(loader));
    w.show();

    // Move the ROS spinning into a separate thread since the call to `spin` is synchronous
    std::thread t{ [node]() { rclcpp::spin(node); } };

    // Run the Qt application, which is also sychronous
    auto ret = app.exec();

    // Shut down ROS to terminate call to `spin`
    rclcpp::shutdown();

    // Wait for the thread to finish
    t.join();

    return ret;
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << std::endl;
    return -1;
  }
}
