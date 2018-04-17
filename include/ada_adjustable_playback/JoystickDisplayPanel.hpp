#ifndef JOYSTICK_DISPLAY_PANEL_HPP
#define JOYSTICK_DISPLAY_PANEL_HPP

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rviz/panel.h>

#include "sensor_msgs/Joy.h"

#include "ada_adjustable_playback/JoystickDisplayWidget.hpp"

#endif // Q_MOC_RUN


#include <QLineEdit>

namespace ada_adjustable_playback {


class JoystickDisplayPanel : public rviz::Panel {
	Q_OBJECT

public:
	JoystickDisplayPanel(QWidget * parent = nullptr);

	virtual void load(rviz::Config const & config);
	virtual void save( rviz::Config config) const;


public Q_SLOTS:
	void updateTopic();

Q_SIGNALS:
	void receivedJoystickData(sensor_msgs::Joy const & joy);

protected:
	void updateJoystickData(sensor_msgs::Joy::ConstPtr const & joy);
	void setTopic(QString const & new_topic);

	JoystickWidget * joystick_widget;
	QLineEdit * topic_editor;
	QString topic;

	ros::NodeHandle nh;
	ros::Subscriber subscriber;


};



} // namespace ada_adjustable_playback




#endif // JOYSTICK_DISPLAY_PANEL_HPP
