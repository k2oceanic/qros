#include "qros/qros_parameter_client.h"
#include "qros/qros_node.h"

QROS_NS_HEAD

QRosParameterClient::QRosParameterClient(QObject *parent)
    : QRosObject()
{
  connect(this, &QRosParameterClient::nodeChanged, this, &QRosParameterClient::reconnect);
}

void QRosParameterClient::setWatchedNode(const QString &node)
{
  if (watched_node_ == node) return;
  watched_node_ = node;
  setAvailable(false);
  emit watchedNodeChanged();
  fetch();
}

void QRosParameterClient::setWatchedParam(const QString &param)
{
  if (watched_param_ == param) return;
  watched_param_ = param;
  emit watchedParamChanged();
  fetch();
}

void QRosParameterClient::setOverride(bool enable)
{
  if (override_ == enable) return;
  override_ = enable;
  emit overrideChanged();
}

void QRosParameterClient::reconnect()
{
  if (conn_params_result_) QObject::disconnect(conn_params_result_);
  if (conn_param_event_)   QObject::disconnect(conn_param_event_);
  if (conn_new_param_)     QObject::disconnect(conn_new_param_);
  if (conn_deleted_)       QObject::disconnect(conn_deleted_);

  QRosNode *n = getNode();
  if (!n) return;

  // Initial fetch response
  conn_params_result_ = connect(n, &QRosNode::parametersGetResult,
      this, [this](bool success, QString node_name, QVariantMap params, QString /*error*/) {
        if (node_name != watched_node_) return;
        setAvailable(success);
        if (success && params.contains(watched_param_))
          setValue(params[watched_param_]);
      });

  // new_parameters — node started or restarted, parameter freshly declared
  conn_new_param_ = connect(n->getParameterEvent(), &QRosParameterEvent::newParam,
      this, [this](QString node_name, QString param_name, QVariant value) {
        if (node_name != watched_node_ || param_name != watched_param_) return;
        emit newParam(value);
        if (override_ && value_.isValid()) {
          // UI overrides driver — push latched value back, ignore YAML default
          setAvailable(true);
          QRosNode *n = getNode();
          if (n) n->setExternalParameterAsync(watched_node_, watched_param_, value_);
        } else {
          // Driver owns state — accept the new value
          setAvailable(true);
          setValue(value);
        }
      });

  // changed_parameters — normal value update on a running node
  conn_param_event_ = connect(n->getParameterEvent(), &QRosParameterEvent::event,
      this, [this](QString node_name, QString param_name, QVariant value) {
        if (node_name != watched_node_ || param_name != watched_param_) return;
        setAvailable(true);
        setValue(value);
      });

  // deleted_parameters — node shut down, mark unavailable and latch value_
  conn_deleted_ = connect(n->getParameterEvent(), &QRosParameterEvent::deleted,
      this, [this](QString node_name, QString param_name) {
        if (node_name != watched_node_ || param_name != watched_param_) return;
        setAvailable(false);
      });

  fetch();
}

void QRosParameterClient::set(const QVariant &value)
{
  QRosNode *n = getNode();
  if (!n || watched_node_.isEmpty() || watched_param_.isEmpty()) return;
  setValue(value);  // optimistic update
  n->setExternalParameterAsync(watched_node_, watched_param_, value);
}

void QRosParameterClient::fetch()
{
  QRosNode *n = getNode();
  if (!n || watched_node_.isEmpty() || watched_param_.isEmpty()) return;
  n->getExternalParametersAsync(watched_node_, QStringList{watched_param_});
}

void QRosParameterClient::setAvailable(bool a)
{
  if (available_ == a) return;
  available_ = a;
  emit availableChanged();
}

void QRosParameterClient::setValue(const QVariant &v)
{
  if (value_ == v) return;
  value_ = v;
  emit valueChanged();
}

QROS_NS_FOOT
