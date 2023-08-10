#ifndef RQT_UDP_BRIDGE_BRIDGE_NODE_H
#define RQT_UDP_BRIDGE_BRIDGE_NODE_H

#include <QObject>
#include <QStandardItemModel>
#include <ros/ros.h>
#include <udp_bridge/BridgeInfo.h>
#include <udp_bridge/TopicStatisticsArray.h>
#include <udp_bridge/AddRemote.h>
#include <udp_bridge/Subscribe.h>

namespace rqt_udp_bridge
{

class BridgeNode: public QObject
{
  Q_OBJECT
public:
  BridgeNode(QObject* parent = nullptr);
  ~BridgeNode();

  /// Set the path to where the bridge_info and topic_statistics topics are located.
  /// @param local True if represents udp_bridge running locally.
  void setTopicsPrefix(ros::NodeHandle& node_handle, std::string node_namespace, bool local);

  void clear();

  QStandardItemModel* topicsModel();
  QStandardItemModel* remotesModel();
  QStandardItemModel* remoteTopicsModel(const std::string &remote);

  bool addRemote(udp_bridge::AddRemote & add_remote);
  bool remoteAdvertise(udp_bridge::Subscribe & subscribe);
  bool remoteSubscribe(udp_bridge::Subscribe & subscribe);

  QStringList topics();
  QStringList remotes();
  QStringList connections(const std::string &remote);
  QStringList remoteTopics(const std::string &remote);

private slots:
  void bridgeInfoUpdated();
  void topicStatisticsUpdated();

private:
  void bridgeInfoCallback(const udp_bridge::BridgeInfo::ConstPtr& bridge_info);
  void topicStatisticsCallback(const udp_bridge::TopicStatisticsArray::ConstPtr& topic_statistics_array);

  /// name of the bridge node as reported by BridgeInfo message
  std::string name_;

  /// namespace used to subscribe to information topics
  std::string node_namespace_;
  ros::NodeHandle node_handle_;

  std::map<std::string, BridgeNode*> remotes_;

  /// Flag indicating if this represents a locally running udp_bridge node.
  bool local_ = false;

  ros::Subscriber bridge_info_subscriber_;
  ros::Subscriber topic_statistics_subscriber_;

  ros::ServiceClient add_remote_service_;
  ros::ServiceClient remote_advertise_service_;
  ros::ServiceClient remote_subscribe_service_;

  QStandardItemModel topics_model_;
  QStandardItemModel remotes_model_;

  udp_bridge::BridgeInfo bridge_info_;
  std::mutex bridge_info_mutex_;

  udp_bridge::TopicStatisticsArray topic_statistics_array_;
  std::mutex topic_statistics_array_mutex_;
};

} // namespace rqt_udp_plugin

#endif
