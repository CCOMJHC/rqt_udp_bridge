#include "rqt_udp_bridge/udp_bridge_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <marine_msgs/KeyValue.h>
#include <QLineEdit>
#include <QLabel>

namespace rqt_udp_bridge
{
    
UDPBridgePlugin::UDPBridgePlugin():rqt_gui_cpp::Plugin()
{
  setObjectName("SonardyneRanger");
}
 
void UDPBridgePlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  
  widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  
  context.addWidget(widget_);
  
  updateTopicList();
  ui_.topicsComboBox->setCurrentIndex(ui_.topicsComboBox->findText(""));
  connect(ui_.topicsComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onTopicChanged(int)));

  ui_.refreshTopicsPushButton->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refreshTopicsPushButton, SIGNAL(pressed()), this, SLOT(updateTopicList()));
  
  // set topic name if passed in as argument
  const QStringList& argv = context.argv();
  if (!argv.empty()) {
      arg_topic_ = argv[0];
      selectTopic(arg_topic_);
  }
}

void UDPBridgePlugin::shutdownPlugin()
{
  channel_statistics_subscriber_.shutdown();
  bridge_info_subscriber_.shutdown();
}

void UDPBridgePlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString topic = ui_.topicsComboBox->currentText();
  instance_settings.setValue("topic", topic);
}

void UDPBridgePlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  QString topic = instance_settings.value("topic", "").toString();
  // don't overwrite topic name passed as command line argument
  if (!arg_topic_.isEmpty())
  {
    arg_topic_ = "";
  }
  else
  {
    selectTopic(topic);
  }
}

void UDPBridgePlugin::updateTopicList()
{
  QString selected = ui_.topicsComboBox->currentText();
  ros::master::V_TopicInfo topic_info;
  ros::master::getTopics(topic_info);
    
  QList<QString> topics;
  for(const auto t: topic_info)
    if (t.datatype == "udp_bridge/ChannelStatisticsArray")
    {
      std::string ci("/channel_info");
      if(t.name.length() > ci.length())
        if(t.name.substr(t.name.size()-ci.size()) == ci)
          topics.append(t.name.substr(0,t.name.size()-ci.size()).c_str());
    }
        
  topics.append("");
  qSort(topics);
  ui_.topicsComboBox->clear();
  for (QList<QString>::const_iterator it = topics.begin(); it != topics.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.topicsComboBox->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectTopic(selected);
}

void UDPBridgePlugin::selectTopic(const QString& topic)
{
  int index = ui_.topicsComboBox->findText(topic);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(topic);
    label.replace(" ", "/");
    ui_.topicsComboBox->addItem(label, QVariant(topic));
    index = ui_.topicsComboBox->findText(topic);
  }
  ui_.topicsComboBox->setCurrentIndex(index);
}

void UDPBridgePlugin::onTopicChanged(int index)
{
  channel_statistics_subscriber_.shutdown();
  bridge_info_subscriber_.shutdown();
    
  QString topic = ui_.topicsComboBox->itemData(index).toString();
  if(!topic.isEmpty())
  {
    channel_statistics_subscriber_ = getNodeHandle().subscribe(topic.toStdString()+"/channel_info", 1, &UDPBridgePlugin::channelStatisticsCallback, this);
    bridge_info_subscriber_ = getNodeHandle().subscribe(topic.toStdString()+"/bridge_info", 1, &UDPBridgePlugin::bridgeInfoCallback, this);
  }
}

void UDPBridgePlugin::channelStatisticsCallback(const udp_bridge::ChannelStatisticsArray& msg)
{
  std::cerr << "stats!" << std::endl;
  for(auto cs: msg.channels)
    std::cerr << " " << cs.source_topic << "\t" << cs.destination_host << "\t" << cs.compressed_bytes_per_second << " bytes/sec" << std::endl;

}

void UDPBridgePlugin::bridgeInfoCallback(const udp_bridge::BridgeInfo& msg)
{
  std::cerr << "bridge info!" << std::endl;
  for(auto t: msg.topics)
  {
    std::cerr << " " << t.topic << std::endl;
    for(auto r: t.remotes)
      std::cerr << "   " << r.remote.name << " (" << r.remote.connection << ") -> " << r.destination_topic << std::endl;
  }

}



} // namespace rqt_udp_bridge

PLUGINLIB_EXPORT_CLASS(rqt_udp_bridge::UDPBridgePlugin, rqt_gui_cpp::Plugin)
