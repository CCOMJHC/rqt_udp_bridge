#ifndef RQT_UDP_BRIDGE_UDP_BRIDGE_PLUGIN_H
#define RQT_UDP_BRIDGE_UDP_BRIDGE_PLUGIN_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_udp_bridge_plugin.h>
#include <ros/ros.h>
#include <udp_bridge/TopicStatisticsArray.h>
#include <udp_bridge/BridgeInfo.h>
#include <rqt_udp_bridge/bridge_node.h>

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

private slots:
  void updateNodeList();
  void selectNode(const QString& node);
  void onNodeChanged(int index);

  void addRemote();
  void subscribe(bool remote_advertise=false);

  void currentRemoteChanged(const QModelIndex& index, const QModelIndex& previous_index);
  void currentLocalTopicChanged(const QModelIndex& index, const QModelIndex& previous_index);
  void currentRemoteTopicChanged(const QModelIndex& index, const QModelIndex& previous_index);


private:
  using RemoteConnectionID = std::pair<std::string, std::string>;
  using TopicRemoteConnection = std::pair<std::string, RemoteConnectionID>;

  RemoteConnectionID getRemoteConnection(const QModelIndex& index);
  TopicRemoteConnection getTopicRemoteConnection(const QModelIndex& index);

  Ui::UDPBridgeWidget ui_;
  QWidget* widget_ = nullptr;

  std::string node_namespace_;

  QString arg_node_;

  std::string active_remote_;
  std::string active_connection_;
  std::string active_local_topic_;
  std::string active_remote_topic_;

  BridgeNode* bridge_node_ = nullptr;

  QMetaObject::Connection remote_topic_changed_connection_;
};

} // namespace rqt_udp_bridge

#endif
