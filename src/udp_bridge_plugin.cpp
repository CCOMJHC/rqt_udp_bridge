#include "rqt_udp_bridge/udp_bridge_plugin.h"
#include "ui_add_remote_dialog.h"

#include <udp_bridge/ListRemotes.h>
#include <udp_bridge/AddRemote.h>
#include <udp_bridge/Subscribe.h>

#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <QLineEdit>
#include <QLabel>
#include <QMessageBox>

namespace rqt_udp_bridge
{
    
UDPBridgePlugin::UDPBridgePlugin():rqt_gui_cpp::Plugin()
{
  setObjectName("UDPBridge");
}
 
void UDPBridgePlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  
  widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  
  context.addWidget(widget_);
  
  updateNodeList();
  ui_.nodesComboBox->setCurrentIndex(ui_.nodesComboBox->findText(""));
  connect(ui_.nodesComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onNodeChanged(int)));

  ui_.refreshNodesPushButton->setIcon(QIcon::fromTheme("view-refresh"));
  connect(ui_.refreshNodesPushButton, SIGNAL(pressed()), this, SLOT(updateNodeList()));


  connect(ui_.addRemotePushButton, SIGNAL(pressed()), this, SLOT(addRemote()));

  connect(ui_.remotesListWidget, SIGNAL(itemSelectionChanged()), this, SLOT(selectedRemoteChanged()) );

  connect(ui_.localTopicsTableWidget, SIGNAL(itemSelectionChanged()), this, SLOT(selectedLocalTopicChanged()));

  connect(ui_.advertisePushButton, SIGNAL(pressed()), this, SLOT(advertise()));

  QStringList local_topics_header;
  local_topics_header.append("topic");
  ui_.localTopicsTableWidget->setHorizontalHeaderLabels(local_topics_header);

  // set node name if passed in as argument
  const QStringList& argv = context.argv();
  if (!argv.empty()) {
      arg_node_ = argv[0];
      selectNode(arg_node_);
  }
}

void UDPBridgePlugin::shutdownPlugin()
{
  channel_statistics_subscriber_.shutdown();
  bridge_info_subscriber_.shutdown();
}

void UDPBridgePlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  QString node = ui_.nodesComboBox->currentText();
  instance_settings.setValue("node", node);
}

void UDPBridgePlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  QString node = instance_settings.value("node", "").toString();
  // don't overwrite topic name passed as command line argument
  if (!arg_node_.isEmpty())
  {
    arg_node_ = "";
  }
  else
  {
    selectNode(node);
  }
}

void UDPBridgePlugin::updateNodeList()
{
  std::vector<std::string> potential_nodes;

  XmlRpc::XmlRpcValue request, response, payload;
  request[0] = ros::this_node::getName();
  if(ros::master::execute("getSystemState", request, response, payload, false))
  {
    if (payload.size() >= 3)
    {
      auto services = payload[2];
      for(int i = 0; i < services.size(); i++)
      {
        std::string service_name = services[i][0];
        std::string lr("/list_remotes");
        if(service_name.length() > lr.length())
          if(service_name.substr(service_name.size()-lr.size()) == lr)
            potential_nodes.push_back(service_name.substr(0,service_name.size()-lr.size()));
      }
    }
  }
  QList<QString> nodes;

  for(auto node: potential_nodes)
  {
    auto service_client = ros::service::createClient<udp_bridge::ListRemotes>(node+"/list_remotes");
    if(service_client.exists())
      nodes.append(node.c_str());
  }

  nodes.append("");
  qSort(nodes);

  QString selected = ui_.nodesComboBox->currentText();

  ui_.nodesComboBox->clear();
  for (QList<QString>::const_iterator it = nodes.begin(); it != nodes.end(); it++)
  {
    QString label(*it);
    label.replace(" ", "/");
    ui_.nodesComboBox->addItem(label, QVariant(*it));
  }

  // restore previous selection
  selectNode(selected);
}

void UDPBridgePlugin::selectNode(const QString& node)
{
  int index = ui_.nodesComboBox->findText(node);
  if (index == -1)
  {
    // add topic name to list if not yet in
    QString label(node);
    label.replace(" ", "/");
    ui_.nodesComboBox->addItem(label, QVariant(node));
    index = ui_.nodesComboBox->findText(node);
  }
  ui_.nodesComboBox->setCurrentIndex(index);
}

void UDPBridgePlugin::onNodeChanged(int index)
{
  channel_statistics_subscriber_.shutdown();
  channel_statistics_array_.channels.clear();
  bridge_info_subscriber_.shutdown();
  bridge_info_ = udp_bridge::BridgeInfo();
  service_clients_.clear();
  ui_.localTopicsTableWidget->clearContents();
  ui_.localTopicsTableWidget->setRowCount(0);
    
  QString node = ui_.nodesComboBox->itemData(index).toString();
  if(!node.isEmpty())
  {
    channel_statistics_subscriber_ = getNodeHandle().subscribe(node.toStdString()+"/channel_info", 1, &UDPBridgePlugin::channelStatisticsCallback, this);
    bridge_info_subscriber_ = getNodeHandle().subscribe(node.toStdString()+"/bridge_info", 1, &UDPBridgePlugin::bridgeInfoCallback, this);
    service_clients_["list_remotes"] = ros::service::createClient<udp_bridge::ListRemotes>(node.toStdString()+"/list_remotes");
    service_clients_["add_remote"] = ros::service::createClient<udp_bridge::AddRemote>(node.toStdString()+"/add_remote");
    service_clients_["remote_advertise"] = ros::service::createClient<udp_bridge::Subscribe>(node.toStdString()+"/remote_advertise");
    service_clients_["remote_subscribe"] = ros::service::createClient<udp_bridge::Subscribe>(node.toStdString()+"/remote_subscribe");
  }
  updateRemotes();
}

void UDPBridgePlugin::channelStatisticsCallback(const udp_bridge::ChannelStatisticsArray& msg)
{
  channel_statistics_array_ = msg;
  updateStatistics();
}

std::string UDPBridgePlugin::selectedRemote()
{
  auto items = ui_.remotesListWidget->selectedItems();
  if(!items.empty())
    return items.front()->text().toStdString();
  return std::string();
}

std::string UDPBridgePlugin::selectedLocalTopic()
{
  auto items = ui_.localTopicsTableWidget->selectedItems();
  for(auto item: items)
  {
    std::cerr << item->row() << ", " << item->column() << " " << item->text().toStdString() << std::endl;
    if(item->column() == 0)
      return item->text().toStdString();
  }
  return std::string();
}

void UDPBridgePlugin::updateStatistics()
{
  std::string selected_remote = selectedRemote();
  for(auto cs: channel_statistics_array_.channels)
    std::cerr << " " << cs.source_topic << "\t" << cs.remote << "\t" << cs.compressed_bytes_per_second << " bytes/sec" << std::endl;

}

void UDPBridgePlugin::bridgeInfoCallback(const udp_bridge::BridgeInfo& msg)
{
  for(auto t: msg.topics)
  {
    auto items = ui_.localTopicsTableWidget->findItems(t.topic.c_str(),Qt::MatchExactly);
    if(items.empty())
    {
      ui_.localTopicsTableWidget->setRowCount(ui_.localTopicsTableWidget->rowCount()+1);
      auto* i = new QTableWidgetItem(QString(t.topic.c_str()));
      ui_.localTopicsTableWidget->setItem(ui_.localTopicsTableWidget->rowCount()-1, 0, i);
    }
  }
}

void UDPBridgePlugin::addRemote()
{
  if(service_clients_["add_remote"].exists())
  {
    Ui::AddRemoteDialog addRemoteDialogUI;
    QDialog addRemoteDialog;
    addRemoteDialogUI.setupUi(&addRemoteDialog);
    if(addRemoteDialog.exec())
    {
      udp_bridge::AddRemote add_remote;
      add_remote.request.address = addRemoteDialogUI.addressLineEdit->text().toStdString();
      add_remote.request.name = addRemoteDialogUI.nameLineEdit->text().toStdString();
      add_remote.request.return_address = addRemoteDialogUI.returnAddressLineEdit->text().toStdString();
      add_remote.request.port = addRemoteDialogUI.portLineEdit->text().toInt();
      service_clients_["add_remote"].call(add_remote);
      updateRemotes();
    }
  }
  else
  {
    QMessageBox::warning(widget_, "UDPBridge add remote", "The add_remote service is not avaiable.");
  }
}

void UDPBridgePlugin::advertise()
{
  auto remote = selectedRemote();
  auto local_topic = selectedLocalTopic();
  if(!remote.empty() && !local_topic.empty())
  {
    udp_bridge::Subscribe s;
    s.request.remote = remote;
    s.request.source_topic = local_topic;
    s.request.destination_topic = ui_.advertiseRemoteTopicLineEdit->text().toStdString();
    s.request.period = ui_.advertisePeriodLineEdit->text().toDouble();
    s.request.queue_size = ui_.advertiseQueueSizeSpinBox->value();
    service_clients_["remote_advertise"].call(s);
  }
}

void UDPBridgePlugin::updateRemotes()
{
  ui_.remotesListWidget->clear();
  ui_.remotesListWidget->addItem("");
  if(service_clients_.find("list_remotes") != service_clients_.end() && service_clients_["list_remotes"].exists())
  {
    udp_bridge::ListRemotes list_remotes;
    if(service_clients_["list_remotes"].call(list_remotes))
    {
      for(auto remote: list_remotes.response.remotes)
      {
        ui_.remotesListWidget->addItem(remote.name.c_str());
      }
    }
  }
}

void UDPBridgePlugin::selectedRemoteChanged()
{
  selectedLocalTopicChanged();
}

void UDPBridgePlugin::selectedLocalTopicChanged()
{
  updateStatistics();

  auto remote = selectedRemote();
  auto local_topic = selectedLocalTopic();

  ui_.advertiseRemoteTopicLineEdit->clear();
  ui_.advertisePeriodLineEdit->setText("0.0");
  ui_.advertiseQueueSizeSpinBox->setValue(1);

  for(auto t: bridge_info_.topics)
  {
    if(t.topic == local_topic)
      for(auto r: t.remotes)
        if(r.remote == remote)
        {
          ui_.advertiseRemoteTopicLineEdit->setText(r.destination_topic.c_str());
          ui_.advertisePeriodLineEdit->setText(QString::number(r.period));
        }
  }
  ui_.advertisePushButton->setEnabled(!remote.empty() && !local_topic.empty());
}

} // namespace rqt_udp_bridge

PLUGINLIB_EXPORT_CLASS(rqt_udp_bridge::UDPBridgePlugin, rqt_gui_cpp::Plugin)
