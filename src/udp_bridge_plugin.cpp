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

  connect(ui_.remotesTableWidget, SIGNAL(itemSelectionChanged()), this, SLOT(selectedRemoteChanged()) );
  connect(ui_.localTopicsTableWidget, SIGNAL(itemSelectionChanged()), this, SLOT(selectedLocalTopicChanged()));
  connect(ui_.remoteTopicsTableWidget, SIGNAL(itemSelectionChanged()), this, SLOT(selectedRemoteTopicChanged()) );

  connect(ui_.advertisePushButton, SIGNAL(pressed()), this, SLOT(advertise()));
  connect(ui_.subscribePushButton, SIGNAL(pressed()), this, SLOT(subscribe()));

  connect(this, SIGNAL(bridgeInfoUpdated()), this, SLOT(updateTables()));
  connect(this, SIGNAL(channelStatisticsUpdated()), this, SLOT(updateStatistics()));

  QStringList local_topics_header;
  local_topics_header.append("topic");
  local_topics_header.append("data rate");
  ui_.localTopicsTableWidget->setHorizontalHeaderLabels(local_topics_header);

  QStringList remotes_header;
  remotes_header.append("remote");
  remotes_header.append("overhead sent");
  remotes_header.append("total sent");
  remotes_header.append("received");
  ui_.remotesTableWidget->setColumnCount(4);
  ui_.remotesTableWidget->setHorizontalHeaderLabels(remotes_header);

  QStringList remote_topics_header;
  remote_topics_header.append("topic");
  remote_topics_header.append("data rate");
  ui_.remoteTopicsTableWidget->setHorizontalHeaderLabels(remote_topics_header);

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
  remote_bridge_info_subsciber_.shutdown();
  remote_channel_statistics_subscriber_.shutdown();
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
  active_remote_.clear();
  active_local_topic_.clear();
  active_remote_topic_.clear();
  channel_statistics_subscriber_.shutdown();
  bridge_info_subscriber_.shutdown();
  remote_bridge_info_subsciber_.shutdown();
  remote_channel_statistics_subscriber_.shutdown();
  service_clients_.clear();
  ui_.localTopicsTableWidget->clearContents();
  ui_.localTopicsTableWidget->setRowCount(0);
  ui_.remotesTableWidget->clearContents();
  ui_.remotesTableWidget->setRowCount(0);
  ui_.advertisePushButton->setEnabled(false);
  ui_.remoteTopicsTableWidget->clearContents();
  ui_.remoteTopicsTableWidget->setRowCount(0);
  ui_.subscribePushButton->setEnabled(false);
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);
    channel_statistics_array_ = udp_bridge::ChannelStatisticsArray();
    bridge_info_ = udp_bridge::BridgeInfo();
    remote_bridge_info_ = udp_bridge::BridgeInfo();
    remote_channel_statistics_array_ = udp_bridge::ChannelStatisticsArray();
  }


  QString node = ui_.nodesComboBox->itemData(index).toString();
  node_namespace_ = node.toStdString();
  if(!node.isEmpty())
  {
    channel_statistics_subscriber_ = getNodeHandle().subscribe(node_namespace_+"/channel_info", 1, &UDPBridgePlugin::channelStatisticsCallback, this);
    bridge_info_subscriber_ = getNodeHandle().subscribe(node_namespace_+"/bridge_info", 1, &UDPBridgePlugin::bridgeInfoCallback, this);
    service_clients_["list_remotes"] = ros::service::createClient<udp_bridge::ListRemotes>(node_namespace_+"/list_remotes");
    service_clients_["add_remote"] = ros::service::createClient<udp_bridge::AddRemote>(node_namespace_+"/add_remote");
    service_clients_["remote_advertise"] = ros::service::createClient<udp_bridge::Subscribe>(node_namespace_+"/remote_advertise");
    service_clients_["remote_subscribe"] = ros::service::createClient<udp_bridge::Subscribe>(node_namespace_+"/remote_subscribe");
  }
}

void UDPBridgePlugin::channelStatisticsCallback(const udp_bridge::ChannelStatisticsArray& msg)
{
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);
    channel_statistics_array_ = msg;
  }
  emit channelStatisticsUpdated();
}

std::string UDPBridgePlugin::selectedRemote()
{
  auto items = ui_.remotesTableWidget->selectedItems();
  for(auto item: items)
    if(item->column() == 0)
      return item->text().toStdString();
  return std::string();
}

std::string UDPBridgePlugin::selectedLocalTopic()
{
  auto items = ui_.localTopicsTableWidget->selectedItems();
  for(auto item: items)
  {
    if(item->column() == 0)
      return item->text().toStdString();
  }
  return std::string();
}

std::string UDPBridgePlugin::selectedRemoteTopic()
{
  auto items = ui_.remoteTopicsTableWidget->selectedItems();
  for(auto item: items)
  {
    if(item->column() == 0)
      return item->text().toStdString();
  }
  return std::string();
}

void UDPBridgePlugin::updateStatistics()
{
  std::map<std::string, std::pair<double,double> > totals;
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);
    for(auto cs: channel_statistics_array_.channels)
    {
      if(totals.find(cs.destination_node) == totals.end())
        totals[cs.destination_node] = std::make_pair<double,double>(0.0,0.0);
      totals[cs.destination_node].second += cs.compressed_bytes_per_second;
      if(cs.source_topic.empty()) // overhead
        totals[cs.destination_node].first += cs.compressed_bytes_per_second;
    }
  }

  for(int row = 0; row < ui_.remotesTableWidget->rowCount(); row++)
  {
    auto item = ui_.remotesTableWidget->item(row, 0);
    if(item)
    {
      auto overhead_item = ui_.remotesTableWidget->item(row, 1);
      if(!overhead_item)
      {
        overhead_item = new QTableWidgetItem;
        ui_.remotesTableWidget->setItem(row, 1, overhead_item);
      }

      auto total_item = ui_.remotesTableWidget->item(row, 2);
      if(!total_item)
      {
        total_item = new QTableWidgetItem;
        ui_.remotesTableWidget->setItem(row, 2, total_item);
      }

      if(totals.find(item->text().toStdString()) != totals.end())
      {
        overhead_item->setText(QString::number(totals[item->text().toStdString()].first)+" bytes/sec");
        total_item->setText(QString::number(totals[item->text().toStdString()].second)+" bytes/sec");
      }
      else
      {
        overhead_item->setText("");
        total_item->setText("");
      }
    }
  }
  ui_.remotesTableWidget->resizeColumnsToContents();

  auto selected_remote = selectedRemote();
  std::map<std::string, double> rates;
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);

    for(auto cs: channel_statistics_array_.channels)
    {
      if(selected_remote.empty() || cs.destination_node == selected_remote)
      {
        if(rates.find(cs.source_topic) == rates.end())
          rates[cs.source_topic] = 0.0;
        rates[cs.source_topic] += cs.compressed_bytes_per_second;
      }
    }
  }

  for(int row = 0; row < ui_.localTopicsTableWidget->rowCount(); row++)
  {
    auto item = ui_.localTopicsTableWidget->item(row, 0);
    if(item)
    {
      auto rate_item = ui_.localTopicsTableWidget->item(row, 1);
      if(!rate_item)
      {
        rate_item = new QTableWidgetItem;
        ui_.localTopicsTableWidget->setItem(row, 1, rate_item);
      }
      if(rates.find(item->text().toStdString()) != rates.end())
        rate_item->setText(QString::number(rates[item->text().toStdString()])+" bytes/sec");
      else
        rate_item->setText("");
    }
  }
  ui_.localTopicsTableWidget->resizeColumnsToContents();

  std::map<std::string, double> remote_rates;
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);

    for(auto cs: remote_channel_statistics_array_.channels)
    {
      if(bridge_info_.name == cs.destination_node)
      {
        remote_rates[cs.source_topic] = cs.compressed_bytes_per_second;
      }
    }
  }

  for(int row = 0; row < ui_.remoteTopicsTableWidget->rowCount(); row++)
  {
    auto item = ui_.remoteTopicsTableWidget->item(row, 0);
    if(item)
    {
      auto rate_item = ui_.remoteTopicsTableWidget->item(row, 1);
      if(!rate_item)
      {
        rate_item = new QTableWidgetItem;
        ui_.remoteTopicsTableWidget->setItem(row, 1, rate_item);
      }
      if(remote_rates.find(item->text().toStdString()) != remote_rates.end())
        rate_item->setText(QString::number(remote_rates[item->text().toStdString()])+" bytes/sec");
      else
        rate_item->setText("");
    }
  }
  ui_.remoteTopicsTableWidget->resizeColumnsToContents();

}

void UDPBridgePlugin::bridgeInfoCallback(const udp_bridge::BridgeInfo& msg)
{
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);
    bridge_info_ = msg;
  }
  emit bridgeInfoUpdated();
}

void UDPBridgePlugin::remoteBridgeInfoCallback(const udp_bridge::BridgeInfo& msg)
{
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);
    remote_bridge_info_ = msg;
  }
  emit bridgeInfoUpdated();
}

void UDPBridgePlugin::remoteChannelStatisticsCallback(const udp_bridge::ChannelStatisticsArray& msg)
{
  {
    std::lock_guard<std::mutex> lock(data_update_mutex_);
    remote_channel_statistics_array_ = msg;
  }
  emit channelStatisticsUpdated();
}

void UDPBridgePlugin::updateTables()
{
  updating_tables_ = true;
  ui_.localTopicsTableWidget->clearContents();
  ui_.localTopicsTableWidget->setRowCount(bridge_info_.topics.size());
  ui_.localTopicsTableWidget->setSortingEnabled(false);
  
  for(int i = 0; i < bridge_info_.topics.size(); i++)
  {
    auto* item = new QTableWidgetItem(QString(bridge_info_.topics[i].topic.c_str()));
    ui_.localTopicsTableWidget->setItem(i, 0, item);
  }

  ui_.localTopicsTableWidget->setSortingEnabled(true);
  ui_.localTopicsTableWidget->sortByColumn(0, Qt::AscendingOrder);

  bool local_topic_changed = false;
  if(active_local_topic_.empty())
    ui_.localTopicsTableWidget->clearSelection();
  else
  {
    auto items = ui_.localTopicsTableWidget->findItems(active_local_topic_.c_str(), Qt::MatchExactly);
    bool found = false;
    for(auto item: items)
      if(item->column() == 0)
      {
        ui_.localTopicsTableWidget->selectRow(item->row());
        found = true;
        break;
      }
    if(!found)
    {
      ui_.localTopicsTableWidget->clearSelection();
      local_topic_changed = true;
    }
  }

  ui_.remotesTableWidget->clearContents();
  ui_.remotesTableWidget->setRowCount(bridge_info_.remotes.size());
  ui_.remotesTableWidget->setSortingEnabled(false);

  bool remote_changed = false;

  for(int i = 0; i < bridge_info_.remotes.size(); i++)
  {
    auto* item = new QTableWidgetItem(QString(bridge_info_.remotes[i].name.c_str()));
    ui_.remotesTableWidget->setItem(i, 0, item);
    double received_bytes_per_second = 0.0;
    for(auto connection: bridge_info_.remotes[i].connections)
      received_bytes_per_second += connection.received_bytes_per_second;
    item = new QTableWidgetItem(QString::number(received_bytes_per_second)+" bytes/sec");
    ui_.remotesTableWidget->setItem(i, 3, item);
  }

  ui_.remotesTableWidget->setSortingEnabled(true);
  ui_.remotesTableWidget->sortByColumn(0, Qt::AscendingOrder);

  if(active_remote_.empty())
    ui_.remotesTableWidget->clearSelection();
  else
  {
    auto items = ui_.remotesTableWidget->findItems(active_remote_.c_str(), Qt::MatchExactly);
    bool found = false;
    for(auto item: items)
      if(item->column() == 0)
      {
        ui_.remotesTableWidget->selectRow(item->row());
        found = true;
        break;
      }
    if(!found)
    {
      ui_.remotesTableWidget->clearSelection();
      remote_changed = true;
    }
  }

  ui_.remoteTopicsTableWidget->clearContents();
  ui_.remoteTopicsTableWidget->setRowCount(remote_bridge_info_.topics.size());
  ui_.remoteTopicsTableWidget->setSortingEnabled(false);

  for(int i = 0; i < remote_bridge_info_.topics.size(); i++)
  {
    auto* item = new QTableWidgetItem(QString(remote_bridge_info_.topics[i].topic.c_str()));
    ui_.remoteTopicsTableWidget->setItem(i, 0, item);
  }

  ui_.remoteTopicsTableWidget->setSortingEnabled(true);
  ui_.remoteTopicsTableWidget->sortByColumn(0, Qt::AscendingOrder);

  bool remote_topic_changed = false;
  if(active_remote_topic_.empty())
    ui_.remoteTopicsTableWidget->clearSelection();
  else
  {
    auto items = ui_.remoteTopicsTableWidget->findItems(active_remote_topic_.c_str(), Qt::MatchExactly);
    bool found = false;
    for(auto item: items)
      if(item->column() == 0)
      {
        ui_.remoteTopicsTableWidget->selectRow(item->row());
        found = true;
        break;
      }
    if(!found)
    {
      ui_.remoteTopicsTableWidget->clearSelection();
      remote_topic_changed = true;
    }
  }


  updating_tables_ = false;

  if(remote_changed)
    selectedRemoteChanged();
  else if (local_topic_changed)
    selectedLocalTopicChanged();
  if(remote_topic_changed)
    selectedRemoteTopicChanged();

  updateStatistics();
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
      bool ok;
      uint16_t return_port = addRemoteDialogUI.returnPortLineEdit->text().toInt(&ok);
      if(ok)
        add_remote.request.return_port = return_port;
      service_clients_["add_remote"].call(add_remote);
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

void UDPBridgePlugin::subscribe()
{
  auto remote = selectedRemote();
  auto remote_topic = selectedRemoteTopic();
  if(!remote.empty() && !remote_topic.empty())
  {
    udp_bridge::Subscribe s;
    s.request.remote = remote;
    s.request.source_topic = remote_topic;
    s.request.destination_topic = ui_.subscribeLocalTopicLineEdit->text().toStdString();
    s.request.period = ui_.subscribePeriodLineEdit->text().toDouble();
    s.request.queue_size = ui_.advertiseQueueSizeSpinBox->value();
    service_clients_["remote_subscribe"].call(s);
  }
}

void UDPBridgePlugin::selectedRemoteChanged()
{
  if(updating_tables_)
    return;
  auto selected = selectedRemote();
  bool found = false;
  for(auto remote: bridge_info_.remotes)
  {
    if(remote.name == selected)
    {
      // ui_.remoteHostLineEdit->setText(remote.host.c_str());
      // ui_.remoteIPAddressLineEdit->setText(remote.ip_address.c_str());
      // ui_.remotePortLineEdit->setText(QString::number(remote.port));
      // ui_.remoteReturnIPAddressLineEdit->setText(remote.return_host.c_str());
      // ui_.remoteReturnPortLineEdit->setText(QString::number(remote.return_port));

      std::string remote_info_topic = node_namespace_+"/remotes/"+remote.topic_name+"/bridge_info";
      if(remote_bridge_info_subsciber_.getTopic() != remote_info_topic)
      {
        remote_bridge_info_subsciber_.shutdown();
        {
          std::lock_guard<std::mutex> lock(data_update_mutex_);
          remote_bridge_info_ = udp_bridge::BridgeInfo();
          ui_.remoteTopicsTableWidget->clearContents();
          ui_.remoteTopicsTableWidget->setRowCount(0);
          ui_.subscribePushButton->setEnabled(false);
        }
        remote_bridge_info_subsciber_ = getNodeHandle().subscribe(remote_info_topic, 1, &UDPBridgePlugin::remoteBridgeInfoCallback, this);
      }
      std::string remote_stats_topic = node_namespace_+"/remotes/"+remote.topic_name+"/channel_statistics";
      if(remote_channel_statistics_subscriber_.getTopic() != remote_stats_topic)
      {
        remote_channel_statistics_subscriber_.shutdown();
        {
          std::lock_guard<std::mutex> lock(data_update_mutex_);
          remote_channel_statistics_array_ = udp_bridge::ChannelStatisticsArray();
        }
        remote_channel_statistics_subscriber_ = getNodeHandle().subscribe(remote_stats_topic, 1, &UDPBridgePlugin::remoteChannelStatisticsCallback, this);
      }

      found = true;
      break;
    }
  }
  if(!found)
  {
    ui_.remoteHostLineEdit->clear();
    ui_.remoteIPAddressLineEdit->clear();
    ui_.remotePortLineEdit->clear();
    remote_bridge_info_subsciber_.shutdown();
    remote_channel_statistics_subscriber_.shutdown();

    std::lock_guard<std::mutex> lock(data_update_mutex_);
    remote_bridge_info_ = udp_bridge::BridgeInfo();
    remote_channel_statistics_array_ = udp_bridge::ChannelStatisticsArray();
  }

  selectedLocalTopicChanged();
}

void UDPBridgePlugin::selectedLocalTopicChanged()
{
  if(updating_tables_)
    return;

  updateStatistics();

  auto remote = selectedRemote();
  auto local_topic = selectedLocalTopic();

  if(active_remote_ != remote || active_local_topic_ != local_topic)
  {
    ui_.advertiseRemoteTopicLineEdit->clear();
    ui_.advertisePeriodLineEdit->setText("0.0");
    ui_.advertiseQueueSizeSpinBox->setValue(1);

    bool subscribed = false;

    for(auto t: bridge_info_.topics)
    {
      if(t.topic == local_topic)
      {
        for(auto r: t.remotes)
          if(r.remote == remote)
          {
            ui_.advertiseRemoteTopicLineEdit->setText(r.destination_topic.c_str());
            ui_.advertisePeriodLineEdit->setText(QString::number(r.period));
            subscribed = true;
            break;
          }
        break;
      }
    }
    if(subscribed)
      ui_.advertisePushButton->setText("Update");
    else
      ui_.advertisePushButton->setText("Advertise");
    ui_.advertisePushButton->setEnabled(!remote.empty() && !local_topic.empty());

    active_remote_ = remote;
    active_local_topic_ = local_topic;
  }
}

void UDPBridgePlugin::selectedRemoteTopicChanged()
{
  if(updating_tables_)
    return;

  auto remote = selectedRemote();
  auto remote_topic = selectedRemoteTopic();

  if(active_remote_topic_ != remote_topic)
  {
    ui_.subscribeLocalTopicLineEdit->clear();
    ui_.subscribePeriodLineEdit->setText("0.0");
    ui_.subscribeQueueSizeSpinBox->setValue(1);

    bool subscibed = false;

    for(auto t: remote_bridge_info_.topics)
    {
      if(t.topic == remote_topic)
      {
        for(auto r: t.remotes)
          if(r.remote == remote_bridge_info_.name)
          {
            ui_.subscribeLocalTopicLineEdit->setText(r.destination_topic.c_str());
            ui_.subscribePeriodLineEdit->setText(QString::number(r.period));
            subscibed = true;
            break;
          }
        break;
      }
    }
    
    if(subscibed)
      ui_.subscribePushButton->setText("Update");
    else
      ui_.subscribePushButton->setText("Subscribe");
    ui_.subscribePushButton->setEnabled(!remote.empty() && !remote_topic.empty());
    
    active_remote_topic_ = remote_topic;
  }
  
}


} // namespace rqt_udp_bridge

PLUGINLIB_EXPORT_CLASS(rqt_udp_bridge::UDPBridgePlugin, rqt_gui_cpp::Plugin)
