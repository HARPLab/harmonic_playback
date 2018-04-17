#include "ada_adjustable_playback/JoystickDisplayPanel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>

#include <boost/bind.hpp>

namespace ada_adjustable_playback {

JoystickDisplayPanel::JoystickDisplayPanel(QWidget * parent) :
		rviz::Panel(parent) {
	ROS_INFO("Building joystick display panel");
	// Next we lay out the "output topic" text entry field using a
	// QLabel and a QLineEdit in a QHBoxLayout.
	QHBoxLayout* topic_layout = new QHBoxLayout;
	topic_layout->addWidget(new QLabel("Joystick Topic:"));
	this->topic_editor = new QLineEdit;
	topic_layout->addWidget(this->topic_editor);
	ROS_INFO("Finished building topic layout");

	// Then create the control widget.
//	drive_widget_ = new DriveWidget;

	// Lay out the topic field above the control widget.
	this->joystick_widget = new JoystickWidget();
	QVBoxLayout* layout = new QVBoxLayout();
	layout->addLayout(topic_layout);
	layout->addWidget(joystick_widget);
	this->setLayout(layout);
	ROS_INFO("Finished building widget layout");

	this->joystick_widget->setEnabled(false);

	// Connect the joystick widget to the signal
	QObject::connect(this, SIGNAL( receivedJoystickData(sensor_msgs::Joy const &) ),
			this->joystick_widget, SLOT( updateJoystickData(sensor_msgs::Joy const &) ));
	QObject::connect( this->topic_editor, SIGNAL( editingFinished() ),
			this, SLOT( updateTopic() ));
	ROS_INFO("Finished building joystick display panel");
}

void JoystickDisplayPanel::updateTopic() {
	this->setTopic(this->topic_editor->text());
}

void JoystickDisplayPanel::setTopic(QString const & new_topic) {
	ROS_INFO_STREAM("Got new topic: " << new_topic.toStdString());
	// Only take action if the name has changed.
	if (new_topic != this->topic) {
		this->topic = new_topic;
		// If the topic is the empty string, don't publish anything.
		if (this->topic == "") {
			this->subscriber.shutdown();
		} else {
			this->subscriber = this->nh.subscribe<sensor_msgs::Joy>(this->topic.toStdString(), 1, boost::bind(&JoystickDisplayPanel::updateJoystickData, this, _1));
		}
		// rviz::Panel defines the configChanged() signal.  Emitting it
		// tells RViz that something in this panel has changed that will
		// affect a saved config file.  Ultimately this signal can cause
		// QWidget::setWindowModified(true) to be called on the top-level
		// rviz::VisualizationFrame, which causes a little asterisk ("*")
		// to show in the window's title bar indicating unsaved changes.
		Q_EMIT this->configChanged();
	}

	// Gray out the control widget when the output topic is empty.
	this->joystick_widget->setEnabled(this->topic != "");
}

void JoystickDisplayPanel::updateJoystickData(sensor_msgs::Joy::ConstPtr const & joy) {
	ROS_INFO_STREAM("Received joy message: " << joy->header.seq);
	Q_EMIT this->receivedJoystickData(*joy);
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void JoystickDisplayPanel::save(rviz::Config config) const {
	rviz::Panel::save(config);
	config.mapSetValue("Topic", this->topic);
}

// Load all configuration data for this panel from the given Config object.
void JoystickDisplayPanel::load(rviz::Config const & config) {
	rviz::Panel::load(config);
	QString topic;
	if (config.mapGetString("Topic", &topic)) {
		this->topic_editor->setText(topic);
		updateTopic();
	}
}

} // namespace ada_adjustable_playback

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ada_adjustable_playback::JoystickDisplayPanel, rviz::Panel )
