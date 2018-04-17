/*
 * JoystickDisplayWidget.hpp
 *
 *  Created on: Apr 16, 2018
 *      Author: reubena
 */

#ifndef INCLUDE_ADA_ADJUSTABLE_PLAYBACK_JOYSTICKDISPLAYWIDGET_HPP_
#define INCLUDE_ADA_ADJUSTABLE_PLAYBACK_JOYSTICKDISPLAYWIDGET_HPP_

#ifndef Q_MOC_RUN

#include "sensor_msgs/Joy.h"

#endif // Q_MOC_RUN

#include <QWidget>

namespace ada_adjustable_playback {

class JoystickWidget: public QWidget {
	Q_OBJECT
public:

	JoystickWidget(QWidget * parent = nullptr);

	virtual void paintEvent(QPaintEvent * event);

	// Override sizeHint() to give the layout managers some idea of a
	// good size for this.
	virtual QSize sizeHint() const {
		return QSize(150, 200);
	}

public Q_SLOTS:
	void updateJoystickData(sensor_msgs::Joy const & joy);

private:
	float axis_x, axis_y;
	bool button_1, button_2;

};

} // namespace ada_adjustable_playback

#endif /* INCLUDE_ADA_ADJUSTABLE_PLAYBACK_JOYSTICKDISPLAYWIDGET_HPP_ */
