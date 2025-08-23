import sys
import serial
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QGroupBox, 
                             QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, 
                             QLabel, QPushButton, QComboBox, QTextEdit, 
                             QSpinBox, QFormLayout, QCheckBox, QScrollArea,
                             QLineEdit)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QTextCursor

class DraggablePlot(QWidget):
    positionChanged = pyqtSignal()
    dragFinished = pyqtSignal()  # New signal for drag completion

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)  # Smaller for better fit
        self.setMaximumSize(250, 250)
        self.x = 0
        self.y = 0
        self.dragging = False
        self.range = (-300, 300)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # Draw grid
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        w, h = self.width(), self.height()
        for i in range(1, 5):
            painter.drawLine(i * w // 5, 0, i * w // 5, h)
            painter.drawLine(0, i * h // 5, w, i * h // 5)

        # Draw axes
        painter.setPen(QPen(Qt.black, 2))
        center_x, center_y = w // 2, h // 2
        painter.drawLine(0, center_y, w, center_y)
        painter.drawLine(center_x, 0, center_x, h)

        # Draw point
        painter.setPen(QPen(Qt.red, 8))  # Smaller point
        scaled_x = center_x + (self.x / 300) * (w // 2)
        scaled_y = center_y - (self.y / 300) * (h // 2)
        painter.drawPoint(int(scaled_x), int(scaled_y))

        # Draw labels
        painter.setFont(QFont('Arial', 7))  # Smaller font
        painter.drawText(5, center_y - 5, "Y+")
        painter.drawText(center_x + 5, 12, "X+")
        painter.drawText(w - 18, center_y - 5, "Y-")
        painter.drawText(center_x + 5, h - 5, "X-")

    def mousePressEvent(self, event):
        center_x, center_y = self.width() // 2, self.height() // 2
        point_x = center_x + (self.x / 300) * (self.width() // 2)
        point_y = center_y - (self.y / 300) * (self.height() // 2)

        if (abs(event.x() - point_x) < 15 and 
            abs(event.y() - point_y) < 15):
            self.dragging = True

    def mouseMoveEvent(self, event):
        if not self.dragging:
            return

        center_x, center_y = self.width() // 2, self.height() // 2
        w, h = self.width(), self.height()

        raw_x = (event.x() - center_x) * 300 / (w // 2)
        raw_y = (center_y - event.y()) * 300 / (h // 2)

        self.x = max(min(raw_x, 300), -300)
        self.y = max(min(raw_y, 300), -300)

        self.update()
        self.positionChanged.emit()  # Still emit for UI updates

    def mouseReleaseEvent(self, event):
        if self.dragging:
            self.dragging = False
            self.dragFinished.emit()  # Emit when drag is complete

    def get_position(self):
        return self.x, self.y

    def set_position(self, x, y):
        self.x = max(min(x, 300), -300)
        self.y = max(min(y, 300), -300)
        self.update()
        self.positionChanged.emit()

class LegControl(QGroupBox):
    positionChanged = pyqtSignal()
    dragFinished = pyqtSignal()  # New signal for drag completion

    def __init__(self, title, quadrant, parent=None):
        super().__init__(title, parent)
        self.quadrant = quadrant
        self.setMaximumWidth(300)  # Limit width for better layout

        self.plot = DraggablePlot()
        self.plot.positionChanged.connect(self.on_position_changed)
        self.plot.dragFinished.connect(self.on_drag_finished)  # Connect drag finished

        self.z_slider = QSlider(Qt.Vertical)
        self.z_slider.setRange(-250, 250)
        self.z_slider.setValue(0)
        self.z_slider.setTickPosition(QSlider.TicksBothSides)
        self.z_slider.setTickInterval(50)
        self.z_slider.valueChanged.connect(self.on_position_changed)
        self.z_slider.sliderReleased.connect(self.on_slider_released)  # Connect slider release

        # Input boxes for direct value entry
        self.x_input = QSpinBox()
        self.x_input.setRange(-300, 300)
        self.x_input.valueChanged.connect(self.on_input_changed)
        self.x_input.setMaximumWidth(80)

        self.y_input = QSpinBox()
        self.y_input.setRange(-300, 300)
        self.y_input.valueChanged.connect(self.on_input_changed)
        self.y_input.setMaximumWidth(80)

        self.z_input = QSpinBox()
        self.z_input.setRange(-250, 250)
        self.z_input.valueChanged.connect(self.on_input_changed)
        self.z_input.setMaximumWidth(80)

        # Position display label
        self.position_label = QLabel("Position: X: 0, Y: 0, Z: 0")
        self.position_label.setWordWrap(True)

        # Set default positions based on quadrant
        if quadrant == 1:  # Front right
            default_x, default_y, default_z = 200, 0, -125
        elif quadrant == 2:  # Front left
            default_x, default_y, default_z = -200, 0, -125
        elif quadrant == 3:  # Rear left
            default_x, default_y, default_z = -150, -150, -125
        elif quadrant == 4:  # Rear right
            default_x, default_y, default_z = 150, -150, -125

        # Set initial values
        self.plot.x = default_x
        self.plot.y = default_y
        self.z_slider.setValue(default_z)
        self.x_input.setValue(default_x)
        self.y_input.setValue(default_y)
        self.z_input.setValue(default_z)

        # Input form layout
        input_form = QFormLayout()
        input_form.setFormAlignment(Qt.AlignLeft)
        input_form.addRow("X:", self.x_input)
        input_form.addRow("Y:", self.y_input)
        input_form.addRow("Z:", self.z_input)

        plot_layout = QHBoxLayout()
        plot_layout.addWidget(self.plot)
        plot_layout.addWidget(self.z_slider)

        main_layout = QVBoxLayout()
        main_layout.addLayout(plot_layout)
        main_layout.addWidget(self.position_label)
        main_layout.addLayout(input_form)

        self.setLayout(main_layout)

    def on_position_changed(self):
        x, y = self.plot.get_position()
        z = self.z_slider.value()

        # Update input boxes without triggering their change events
        self.x_input.blockSignals(True)
        self.y_input.blockSignals(True)
        self.z_input.blockSignals(True)

        self.x_input.setValue(int(x))
        self.y_input.setValue(int(y))
        self.z_input.setValue(z)

        self.x_input.blockSignals(False)
        self.y_input.blockSignals(False)
        self.z_input.blockSignals(False)

        self.position_label.setText(f"Position: X: {int(x)}, Y: {int(y)}, Z: {z}")
        self.positionChanged.emit()

    def on_input_changed(self):
        # Update plot and slider from input boxes
        x = self.x_input.value()
        y = self.y_input.value()
        z = self.z_input.value()

        self.plot.set_position(x, y)
        self.z_slider.setValue(z)

        self.position_label.setText(f"Position: X: {x}, Y: {y}, Z: {z}")
        self.positionChanged.emit()

    def on_drag_finished(self):
        """Handle drag completion"""
        self.dragFinished.emit()

    def on_slider_released(self):
        """Handle slider release"""
        self.dragFinished.emit()

    def get_position(self):
        x, y = self.plot.get_position()
        z = self.z_slider.value()
        return (x, y, z)

    def set_position(self, x, y, z):
        """Set position programmatically"""
        self.plot.set_position(x, y)
        self.z_slider.setValue(z)
        self.x_input.setValue(int(x))
        self.y_input.setValue(int(y))
        self.z_input.setValue(int(z))

    def get_command(self):
        x, y, z = self.get_position()
        return f"L {self.quadrant} {int(x)} {int(y)} {int(z)}"

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Leg Controller")
        self.setGeometry(50, 50, 1800, 900)  # Better size for 1080p

        # Create leg controls
        self.legs = [
            LegControl("Front Right", 1),
            LegControl("Front Left", 2),
            LegControl("Rear Left", 3),
            LegControl("Rear Right", 4)
        ]

        # Connect signals
        for leg in self.legs:
            leg.positionChanged.connect(self.handle_position_change)
            leg.dragFinished.connect(self.handle_drag_finished)  # Connect drag finished

        # Serial log
        self.serial_log = QTextEdit()
        self.serial_log.setReadOnly(True)
        self.serial_log.setFont(QFont("Courier", 9))  # Smaller font
        self.serial_log.setMaximumHeight(200)  # Limit height

        # Custom command input
        self.custom_command_input = QLineEdit()
        self.custom_command_input.setPlaceholderText("Enter custom command...")
        self.custom_command_input.returnPressed.connect(self.send_custom_command)  # Send on Enter press
        self.custom_command_input.setMaximumWidth(400)

        self.send_custom_button = QPushButton("Send Custom")
        self.send_custom_button.clicked.connect(self.send_custom_command)
        self.send_custom_button.setMaximumWidth(100)

        custom_command_layout = QHBoxLayout()
        custom_command_layout.addWidget(QLabel("Custom Command:"))
        custom_command_layout.addWidget(self.custom_command_input)
        custom_command_layout.addWidget(self.send_custom_button)
        custom_command_layout.addStretch()

        # Serial controls
        self.port_combo = QComboBox()
        self.port_combo.addItems(["/dev/ttyUSB1", "COM4"])
        self.port_combo.setMaximumWidth(100)

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setMaximumWidth(100)

        self.send_button = QPushButton("Send All Legs")
        self.send_button.clicked.connect(self.send_all_commands)
        self.send_button.setMaximumWidth(120)

        # Add "Send on update" checkbox
        self.send_on_update = QCheckBox("Send on update")
        self.send_on_update.setChecked(False)

        # Add clear log button
        self.clear_log_button = QPushButton("Clear Log")
        self.clear_log_button.clicked.connect(self.clear_serial_log)
        self.clear_log_button.setMaximumWidth(100)

        # Add open/close serial button
        self.serial_toggle = QPushButton("Open Serial")
        self.serial_toggle.clicked.connect(self.toggle_serial)
        self.serial_toggle.setMaximumWidth(100)
        self.serial_connection = None

        # Add reset button
        self.reset_button = QPushButton("Reset All to (0,0,0)")
        self.reset_button.clicked.connect(self.reset_all_positions)
        self.reset_button.setMaximumWidth(150)

        # Serial response timer
        self.response_timer = QTimer()
        self.response_timer.timeout.connect(self.check_serial_responses)
        self.response_timer.setInterval(100)

        # Create a scrollable area for the grid
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)

        # Grid container
        grid_container = QWidget()
        grid = QGridLayout()
        grid.setSpacing(10)  # Reduced spacing

        grid.addWidget(self.legs[1], 0, 0)
        grid.addWidget(self.legs[0], 0, 1)
        grid.addWidget(self.legs[2], 1, 0)
        grid.addWidget(self.legs[3], 1, 1)

        grid_container.setLayout(grid)
        scroll_area.setWidget(grid_container)

        # Serial controls layout
        serial_layout = QHBoxLayout()
        serial_layout.addWidget(QLabel("Port:"))
        serial_layout.addWidget(self.port_combo)
        serial_layout.addWidget(QLabel("Baud:"))
        serial_layout.addWidget(self.baud_combo)
        serial_layout.addWidget(self.serial_toggle)
        serial_layout.addWidget(self.send_button)
        serial_layout.addWidget(self.send_on_update)
        serial_layout.addWidget(self.reset_button)
        serial_layout.addWidget(self.clear_log_button)
        serial_layout.addStretch()

        main_layout = QVBoxLayout()
        main_layout.addLayout(serial_layout)
        main_layout.addWidget(scroll_area, 1)  # Stretch factor 1 for scroll area
        main_layout.addLayout(custom_command_layout)  # Add custom command input
        main_layout.addWidget(QLabel("Serial Log:"))
        main_layout.addWidget(self.serial_log)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Status bar
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")

    def handle_position_change(self):
        """Handle position changes for UI updates only"""
        # This now only updates the UI, not sending commands
        pass

    def handle_drag_finished(self):
        """Handle drag completion - send command if enabled"""
        if not self.send_on_update.isChecked():
            return

        # The sender is the LegControl instance
        changed_leg = self.sender()
        if not changed_leg:
            return

        # Get the current command
        command = changed_leg.get_command()

        # Send immediately if serial is open
        if self.serial_connection and self.serial_connection.is_open:
            self.send_command(command)
            self.log_message(f"Sent: {command}", "out")

    def reset_all_positions(self):
        """Reset all leg positions to (0,0,0)"""
        for leg in self.legs:
            leg.set_position(0, 0, 0)

        self.log_message("All positions reset to (0,0,0)", "info")

    def send_custom_command(self):
        """Send custom command from input field"""
        command = self.custom_command_input.text().strip()
        if not command:
            return

        if self.serial_connection and self.serial_connection.is_open:
            self.send_command(command)
            self.log_message(f"Sent custom: {command}", "out")
            self.custom_command_input.clear()  # Clear input after sending
        else:
            self.log_message("Error: Serial port not open", "error")

    def log_message(self, message, direction="in"):
        """Add message to serial log with timestamp and direction"""
        timestamp = self.get_timestamp()
        if direction == "out":
            prefix = f"[SENT {timestamp}]"
            color = "blue"
        elif direction == "info":
            prefix = f"[INFO {timestamp}]"
            color = "purple"
        elif direction == "error":
            prefix = f"[ERROR {timestamp}]"
            color = "red"
        else:  # in
            prefix = f"[RECV {timestamp}]"
            color = "green"

        html = f'<font color="{color}">{prefix} {message}</font>'
        self.serial_log.append(html)
        self.serial_log.moveCursor(QTextCursor.End)

    def get_timestamp(self):
        """Get current timestamp for logging"""
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def clear_serial_log(self):
        """Clear the serial log"""
        self.serial_log.clear()

    def send_command(self, command):
        """Send a single command to the serial port"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.log_message("Error: Serial port not open", "error")
            return

        try:
            full_command = command + "\n"
            self.serial_connection.write(full_command.encode())
        except Exception as e:
            self.log_message(f"Send error: {str(e)}", "error")

    def send_all_commands(self):
        """Send commands for all legs immediately"""
        if not self.serial_connection or not self.serial_connection.is_open:
            self.log_message("Error: Serial port not open", "error")
            return

        for leg in self.legs:
            command = leg.get_command()
            self.send_command(command)
            self.log_message(f"Sent: {command}", "out")

    def toggle_serial(self):
        """Open or close the serial connection"""
        if self.serial_connection and self.serial_connection.is_open:
            self.close_serial()
            self.serial_toggle.setText("Open Serial")
            self.status_bar.showMessage("Serial port closed")
        else:
            self.open_serial()
            self.serial_toggle.setText("Close Serial")

    def open_serial(self):
        """Open the serial connection"""
        port = self.port_combo.currentText()
        baud = int(self.baud_combo.currentText())

        try:
            self.serial_connection = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.log_message(f"Serial port opened: {port} @ {baud} baud", "info")
            self.status_bar.showMessage(f"Connected to {port} @ {baud} baud")
            self.response_timer.start()

        except Exception as e:
            self.log_message(f"Failed to open serial port: {str(e)}", "error")
            self.status_bar.showMessage(f"Error: {str(e)}")
            self.serial_connection = None

    def close_serial(self):
        """Close the serial connection"""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.response_timer.stop()
                self.serial_connection.close()
                self.log_message("Serial port closed", "info")
                self.status_bar.showMessage("Serial port closed")
            except Exception as e:
                self.log_message(f"Error closing serial: {str(e)}", "error")
        self.serial_connection = None

    def check_serial_responses(self):
        """Check for incoming serial responses"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return

        try:
            while self.serial_connection.in_waiting > 0:
                response = self.serial_connection.readline().decode().strip()
                if response:
                    self.log_message(response, "in")
        except Exception as e:
            self.log_message(f"Read error: {str(e)}", "error")

    def closeEvent(self, event):
        """Clean up when window is closed"""
        self.close_serial()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())