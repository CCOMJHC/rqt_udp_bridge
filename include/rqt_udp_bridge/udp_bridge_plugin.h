#ifndef RQT_UDP_BRIDGE_UDP_BRIDGE_PLUGIN_H
#define RQT_UDP_BRIDGE_UDP_BRIDGE_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_udp_bridge_plugin.h>
#include <ros/ros.h>
#include <udp_bridge/ChannelStatisticsArray.h>
#include <udp_bridge/BridgeInfo.h>

class QLabel;

namespace rqt_udp_bridge
{
    
class UDPBridgePlugin: public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  UDPBridgePlugin();
    
  void initPlugin(qt_gui_cpp::PluginContext& context) override;
  void shutdownPlugin() override;
  void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
  void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;
    
protected:
  std::string selectedRemote();
  std::string selectedLocalTopic();

protected slots:
  void updateNodeList();
    
  void selectNode(const QString& node);

  void onNodeChanged(int index);
  void addRemote();
  void advertise();

  void channelStatisticsCallback(const udp_bridge::ChannelStatisticsArray& msg);
  void bridgeInfoCallback(const udp_bridge::BridgeInfo& msg);

  void updateRemotes();

  void selectedRemoteChanged();
  void selectedLocalTopicChanged();

  void updateStatistics();

private:
  Ui::UDPBridgeWidget ui_;
  QWidget* widget_ = nullptr;

  ros::Subscriber channel_statistics_subscriber_;
  ros::Subscriber bridge_info_subscriber_;

  QString arg_node_;

  std::map<std::string, ros::ServiceClient> service_clients_;

  udp_bridge::ChannelStatisticsArray channel_statistics_array_;
  udp_bridge::BridgeInfo bridge_info_;
};

} // namespace rqt_udp_bridge

#endif
