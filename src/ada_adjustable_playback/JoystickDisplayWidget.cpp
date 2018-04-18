
#include "ada_adjustable_playback/JoystickDisplayWidget.hpp"

#include <ros/ros.h>
#include <QPainter>

namespace ada_adjustable_playback {

JoystickWidget::JoystickWidget(QWidget * parent) :
		QWidget(parent), axis_x(0.), axis_y(0.), button_1(false), button_2(
				false) {
	ROS_INFO("Built widget");
	// empty
}

void JoystickWidget::paintEvent(QPaintEvent * parent) {
	// The background color and crosshair lines are drawn differently
	// depending on whether this widget is enabled or not.  This gives a
	// nice visual indication of whether the control is "live".
	QColor background;
	QColor crosshair;
	if (this->isEnabled()) {
		background = Qt::white;
		crosshair = Qt::black;
	} else {
		background = Qt::lightGray;
		crosshair = Qt::darkGray;
	}
	  // The main visual is a square, centered in the widget's area.  Here
	  // we compute the size of the square and the horizontal and vertical
	  // offsets of it.
	  int const w = this->width();
	  int const h = this->height();
	  int const size = (( w > h ) ? h : w) - 1;
	  int const axis_size = size * 3 / 4;
	  int const hpad_axis = ( w - axis_size ) / 2;
	  int const vpad_axis = ( h - size ) / 2;
	  int const h_orig = w / 2;
	  int const v_orig = vpad_axis + axis_size / 2;

	  QPainter painter( this );
	  painter.setBrush( background );
	  painter.setPen( crosshair );


	  // Draw the background square.
	  painter.drawRect( QRect( hpad_axis, vpad_axis, axis_size, axis_size ));

	  // Draw a cross-hair inside the square.
	  painter.drawLine( hpad_axis, v_orig, hpad_axis + axis_size, v_orig );
	  painter.drawLine( h_orig, vpad_axis, h_orig, vpad_axis + axis_size );

	  // Draw outlines of buttons
	  int const but_size = size / 5;
	  int const hpad_but = hpad_axis;
	  int const vpad_but = vpad_axis + axis_size + size / 20;

	  int const but_width = axis_size / 5;
	  int const but_height = but_size;
	  int const but_rad = but_height / 6;
	  painter.drawRoundedRect(hpad_but + but_width, vpad_but, but_width, but_height, but_rad, but_rad);
	  painter.drawRoundedRect(hpad_but + 3*but_width, vpad_but, but_width, but_height, but_rad, but_rad);

	  if (this->isEnabled()) {
		  // draw a line to the joystick point
		  int const pt_x = h_orig - this->axis_x * axis_size/2;
		  int const pt_y = v_orig - this->axis_y * axis_size/2;
		  painter.drawLine( h_orig,v_orig, pt_x, pt_y);
		  painter.setBrush( crosshair );
		  painter.drawEllipse( pt_x-3, pt_y-3, 6, 6);

		  // draw the filled buttons
		  if (this->button_1) {
			  painter.drawRoundedRect(hpad_but + but_width, vpad_but, but_width, but_height, but_rad, but_rad);
		  }
		  if (this->button_2) {
			  painter.drawRoundedRect(hpad_but + 3*but_width, vpad_but, but_width, but_height, but_rad, but_rad);
		  }
	  }

	  // TODO: Draw buttons
}

void JoystickWidget::updateJoystickData(sensor_msgs::Joy const & joy) {
	if (joy.axes.size() >= 1) {
		this->axis_x = joy.axes[0];
	}
	if (joy.axes.size() >= 2) {
		this->axis_y = joy.axes[1];
	}

	if (joy.buttons.size() >= 1) {
		this->button_1 = joy.buttons[0] > 0;
	}
	if (joy.buttons.size() >= 2) {
		this->button_2 = joy.buttons[1] > 0;
	}
	this->update();

	// TODO: Check if data is old and disable
}

} // namespace ada_adjustable_playback
