import sys
import serial
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QGroupBox, 
                             QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, 
                             QLabel, QPushButton, QComboBox, QTextEdit, 
                             QSpinBox, QFormLayout, QCheckBox, QScrollArea,
                             QLineEdit)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QTextCursor

# Configuration variables for easy modification
X_RANGE = (-250, 250)
Y_RANGE = (-250, 250)
Z_RANGE = (-250, 250)

class DraggablePlot(QWidget):
    positionChanged = pyqtSignal()
    dragFinished = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.setMaximumSize(250, 250)
        self.x = 0
        self.y = 0
        self.dragging = False
        self.range = X_RANGE  # Use configured range

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
        painter.setPen(QPen(Qt.red, 8))
        range_x = X_RANGE[1] - X_RANGE[0]
        range_y = Y_RANGE[1] - Y_RANGE[0]
        scaled_x = center_x + (self.x / range_x) * w
        scaled_y = center_y - (self.y / range_y) * h
        painter.drawPoint(int(scaled_x), int(scaled_y))

        # Draw labels
        painter.setFont(QFont('Arial', 7))
        painter.drawText(5, center_y - 5, "X-")
        painter.drawText(center_x + 5, 12, "Y+")
        painter.drawText(w - 18, center_y - 5, "X+")
        painter.drawText(center_x + 5, h - 5, "X-")

    def mousePressEvent(self, event):
        center_x, center_y = self.width() // 2, self.height() // 2
        range_x = X_RANGE[1] - X_RANGE[0]
        range_y = Y_RANGE[1] - Y_RANGE[0]
        point_x = center_x + (self.x / range_x) * self.width()
        point_y = center_y - (self.y / range_y) * self.height()

        if (abs(event.x() - point_x) < 15 and 
            abs(event.y() - point_y) < 15):
            self.dragging = True

    def mouseMoveEvent(self, event):
        if not self.dragging:
            return

        center_x, center_y = self.width() // 2, self.height() // 2
        w, h = self.width(), self.height()
        range_x = X_RANGE[1] - X_RANGE[0]
        range_y = Y_RANGE[1] - Y_RANGE[0]

        raw_x = (event.x() - center_x) * range_x / w
        raw_y = (center_y - event.y()) * range_y / h

        self.x = max(min(raw_x, X_RANGE[1]), X_RANGE[0])
        self.y = max(min(raw_y, Y_RANGE[1]), Y_RANGE[0])

        self.update()
        self.positionChanged.emit()

    def mouseReleaseEvent(self, event):
        if self.dragging:
            self.dragging = False
            self.dragFinished.emit()

    def get_position(self):
        return self.x, self.y

    def set_position(self, x, y):
        self.x = max(min(x, X_RANGE[1]), X_RANGE[0])
        self.y = max(min(y, Y_RANGE[1]), Y_RANGE[0])
        self.update()
        self.positionChanged.emit()

class LegControl(QGroupBox):
    positionChanged = pyqtSignal()
    dragFinished = pyqtSignal()

    def __init__(self, title, quadrant, parent=None):
        super().__init__(title, parent)
        self.quadrant = quadrant
        self.setMaximumWidth(300)
        self.external_update = False  # Flag to prevent recursion

        self.plot = DraggablePlot()
        self.plot.positionChanged.connect(self.on_position_changed)
        self.plot.dragFinished.connect(self.on_drag_finished)

        self.z_slider = QSlider(Qt.Vertical)
        self.z_slider.setRange(Z_RANGE[0], Z_RANGE[1])
        self.z_slider.setValue(0)
        self.z_slider.setTickPosition(QSlider.TicksBothSides)
        self.z_slider.setTickInterval(50)
        self.z_slider.valueChanged.connect(self.on_position_changed)
        self.z_slider.sliderReleased.connect(self.on_slider_released)

        # Input boxes for direct value entry
        self.x_input = QSpinBox()
        self.x_input.setRange(X_RANGE[0], X_RANGE[1])
        self.x_input.valueChanged.connect(self.on_input_changed)
        self.x_input.setMaximumWidth(80)

        self.y_input = QSpinBox()
        self.y_input.setRange(Y_RANGE[0], Y_RANGE[1])
        self.y_input.valueChanged.connect(self.on_input_changed)
        self.y_input.setMaximumWidth(80)

        self.z_input = QSpinBox()
        self.z_input.setRange(Z_RANGE[0], Z_RANGE[1])
        self.z_input.valueChanged.connect(self.on_input_changed)
        self.z_input.setMaximumWidth(80)

        # Position display label
        self.position_label = QLabel("Position: X: 0, Y: 0, Z: 0")
        self.position_label.setWordWrap(True)

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
        
        # Only emit if not an external update
        if not self.external_update:
            self.positionChanged.emit()

    def on_input_changed(self):
        # Update plot and slider from input boxes
        x = self.x_input.value()
        y = self.y_input.value()
        z = self.z_input.value()

        self.plot.blockSignals(True)
        self.z_slider.blockSignals(True)
        
        self.plot.set_position(x, y)
        self.z_slider.setValue(z)
        
        self.plot.blockSignals(False)
        self.z_slider.blockSignals(False)

        self.position_label.setText(f"Position: X: {x}, Y: {y}, Z: {z}")
        
        # Only emit if not an external update
        if not self.external_update:
            self.positionChanged.emit()
            self.dragFinished.emit()

    def on_drag_finished(self):
        self.dragFinished.emit()

    def on_slider_released(self):
        self.dragFinished.emit()

    def get_position(self):
        x, y = self.plot.get_position()
        z = self.z_slider.value()
        return (x, y, z)

    def set_position(self, x, y, z, external=False):
        """Set position programmatically with optional external flag"""
        self.external_update = external
        self.plot.set_position(x, y)
        self.z_slider.setValue(z)
        self.x_input.setValue(int(x))
        self.y_input.setValue(int(y))
        self.z_input.setValue(int(z))
        self.external_update = False

    def get_command(self):
        x, y, z = self.get_position()
        return f"L {self.quadrant} {int(x)} {int(y)} {int(z)}"

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Leg Controller")
        self.setGeometry(50, 50, 1800, 900)

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
            leg.dragFinished.connect(self.handle_drag_finished)

        # Serial log
        self.serial_log = QTextEdit()
        self.serial_log.setReadOnly(True)
        self.serial_log.setFont(QFont("Courier", 9))
        self.serial_log.setMaximumHeight(200)

        # Custom command input - replaced QLineEdit with QTextEdit
        self.custom_command_input = QTextEdit()
        self.custom_command_input.setMaximumHeight(60)  # Start with single line height
        self.custom_command_input.setPlaceholderText("Enter custom command...")
        
        # Multiline checkbox
        self.multiline_checkbox = QCheckBox("Multiline")
        self.multiline_checkbox.stateChanged.connect(self.toggle_multiline_mode)
        self.multiline_checkbox.setMaximumWidth(80)
        
        # Connect custom command input return key handling
        self.custom_command_input.keyPressEvent = self.custom_command_key_press

        self.send_custom_button = QPushButton("Send Custom")
        self.send_custom_button.clicked.connect(self.send_custom_command)
        self.send_custom_button.setMaximumWidth(100)

        custom_command_layout = QHBoxLayout()
        custom_command_layout.addWidget(QLabel("Custom Command:"))
        custom_command_layout.addWidget(self.custom_command_input)
        custom_command_layout.addWidget(self.multiline_checkbox)
        custom_command_layout.addWidget(self.send_custom_button)

        # Serial controls
        self.port_combo = QComboBox()
        self.port_combo.addItems(["COM4", "/dev/rfcomm0", "/dev/ttyUSB0"])
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
        grid.setSpacing(10)

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
        main_layout.addWidget(scroll_area, 1)
        main_layout.addLayout(custom_command_layout)
        main_layout.addWidget(QLabel("Serial Log:"))
        main_layout.addWidget(self.serial_log)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        # Status bar
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")

    def toggle_multiline_mode(self, state):
        """Toggle between single-line and multi-line mode for custom command input"""
        if state == Qt.Checked:
            # Enable multiline mode
            self.custom_command_input.setMaximumHeight(120)  # Allow more height
            self.custom_command_input.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        else:
            # Disable multiline mode
            self.custom_command_input.setMaximumHeight(60)  # Back to single line height
            self.custom_command_input.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def custom_command_key_press(self, event):
        """Handle key presses in the custom command input"""
        if (event.key() == Qt.Key_Return and 
            not self.multiline_checkbox.isChecked() and 
            not (event.modifiers() & Qt.ShiftModifier)):
            # Send command on Enter (unless Shift is held or multiline is enabled)
            self.send_custom_command()
            event.accept()
        else:
            # Default behavior (including newlines when multiline is enabled)
            QTextEdit.keyPressEvent(self.custom_command_input, event)

    def handle_position_change(self):
        pass

    def handle_drag_finished(self):
        if not self.send_on_update.isChecked():
            return

        changed_leg = self.sender()
        if not changed_leg:
            return

        command = changed_leg.get_command()

        if self.serial_connection and self.serial_connection.is_open:
            self.send_command(command)
            self.log_message(f"Sent: {command}", "out")

    def reset_all_positions(self):
        for leg in self.legs:
            leg.set_position(0, 0, 0)

        self.log_message("All positions reset to (0,0,0)", "info")
        
        # Send reset commands if "Send on update" is enabled
        if self.send_on_update.isChecked():
            self.send_all_commands()

    def send_custom_command(self):
        command = self.custom_command_input.toPlainText().strip()
        if not command:
            return

        if self.serial_connection and self.serial_connection.is_open:
            self.send_command(command)
            self.log_message(f"Sent custom: {command}", "out")
            # Don't clear the input, just select it for easy editing
            self.custom_command_input.selectAll()
        else:
            self.log_message("Error: Serial port not open", "error")

    def log_message(self, message, direction="in"):
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
        else:
            prefix = f"[RECV {timestamp}]"
            color = "green"

        html = f'<font color="{color}">{prefix} {message}</font>'
        self.serial_log.append(html)
        self.serial_log.moveCursor(QTextCursor.End)

    def get_timestamp(self):
        from datetime import datetime
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def clear_serial_log(self):
        self.serial_log.clear()

    def send_command(self, command):
        if not self.serial_connection or not self.serial_connection.is_open:
            self.log_message("Error: Serial port not open", "error")
            return

        try:
            full_command = command + "\n"
            self.serial_connection.write(full_command.encode())
        except Exception as e:
            self.log_message(f"Send error: {str(e)}", "error")

    def send_all_commands(self):
        if not self.serial_connection or not self.serial_connection.is_open:
            self.log_message("Error: Serial port not open", "error")
            return

        for leg in self.legs:
            command = leg.get_command()
            self.send_command(command)
            self.log_message(f"Sent: {command}", "out")

    def toggle_serial(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.close_serial()
            self.serial_toggle.setText("Open Serial")
            self.status_bar.showMessage("Serial port closed")
        else:
            self.open_serial()
            self.serial_toggle.setText("Close Serial")

    def open_serial(self):
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
        if not self.serial_connection or not self.serial_connection.is_open:
            return

        try:
            while self.serial_connection.in_waiting > 0:
                response = self.serial_connection.readline().decode().strip()
                if response:
                    self.log_message(response, "in")
                    
                    # Check for LEG_UPDATE messages
                    if response.startswith("LEG_UPDATE"):
                        try:
                            parts = response.split()
                            quadrant = int(parts[1])
                            x = int(float(parts[2]))
                            y = int(float(parts[3]))
                            z = int(float(parts[4]))
                            
                            # Update the corresponding leg without sending serial command
                            if 1 <= quadrant <= 4:
                                self.legs[quadrant-1].set_position(x, y, z, external=True)
                                self.log_message(f"Updated leg {quadrant} to ({x}, {y}, {z})", "info")
                        except (ValueError, IndexError) as e:
                            self.log_message(f"Error parsing LEG_UPDATE: {str(e)}", "error")
        except Exception as e:
            self.log_message(f"Read error: {str(e)}", "error")

    def closeEvent(self, event):
        self.close_serial()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())