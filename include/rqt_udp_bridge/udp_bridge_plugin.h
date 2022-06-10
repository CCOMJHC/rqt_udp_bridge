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
    
protected slots:
  void updateTopicList();
    
  void selectTopic(const QString& topic);

  void onTopicChanged(int index);
  
  void channelStatisticsCallback(const udp_bridge::ChannelStatisticsArray& msg);
  void bridgeInfoCallback(const udp_bridge::BridgeInfo& msg);

private:
  Ui::UDPBridgeWidget ui_;
  QWidget* widget_ = nullptr;

  ros::Subscriber channel_statistics_subscriber_;
  ros::Subscriber bridge_info_subscriber_;

  QString arg_topic_;
};

} // namespace rqt_udp_bridge

#endif
