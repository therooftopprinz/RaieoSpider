import sys
import serial
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QGroupBox, 
                             QVBoxLayout, QHBoxLayout, QGridLayout, QSlider, 
                             QLabel, QPushButton, QComboBox, QTextEdit, 
                             QSpinBox, QFormLayout, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QTextCursor

class DraggablePlot(QWidget):
    positionChanged = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(250, 250)
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
        painter.setPen(QPen(Qt.red, 10))
        scaled_x = center_x + (self.x / 300) * (w // 2)
        scaled_y = center_y - (self.y / 300) * (h // 2)
        painter.drawPoint(int(scaled_x), int(scaled_y))
        
        # Draw labels
        painter.setFont(QFont('Arial', 8))
        painter.drawText(5, center_y - 5, "Y+")
        painter.drawText(center_x + 5, 15, "X+")
        painter.drawText(w - 20, center_y - 5, "Y-")
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
        self.positionChanged.emit()
    
    def mouseReleaseEvent(self, event):
        self.dragging = False
    
    def get_position(self):
        return self.x, self.y

class LegControl(QGroupBox):
    positionChanged = pyqtSignal()
    
    def __init__(self, title, quadrant, parent=None):
        super().__init__(title, parent)
        self.quadrant = quadrant
        
        self.plot = DraggablePlot()
        self.plot.positionChanged.connect(self.on_position_changed)
        
        self.z_slider = QSlider(Qt.Vertical)
        self.z_slider.setRange(-250, 250)
        self.z_slider.setValue(0)
        self.z_slider.setTickPosition(QSlider.TicksBothSides)
        self.z_slider.setTickInterval(50)
        self.z_slider.valueChanged.connect(self.on_position_changed)
        
        self.position_label = QLabel("Raw: X: 0, Y: 0, Z: 0")
        self.adjusted_label = QLabel("Adjusted: X: 0, Y: 0, Z: 0")
        
        self.offset_x = QSpinBox()
        self.offset_x.setRange(-500, 500)
        self.offset_x.valueChanged.connect(self.on_position_changed)
        
        self.offset_y = QSpinBox()
        self.offset_y.setRange(-500, 500)
        self.offset_y.valueChanged.connect(self.on_position_changed)
        
        self.offset_z = QSpinBox()
        self.offset_z.setRange(-500, 500)
        self.offset_z.valueChanged.connect(self.on_position_changed)
        
        # Set default offsets
        if quadrant == 1:  # Front right
            self.offset_x.setValue(100)
            self.offset_y.setValue(0)
            self.offset_z.setValue(-100)
        elif quadrant == 2 or quadrant == 3:  # Front left & Rear left
            self.offset_x.setValue(-100)
            self.offset_y.setValue(0)
            self.offset_z.setValue(-100)
        elif quadrant == 4:  # Rear right
            self.offset_x.setValue(100)
            self.offset_y.setValue(0)
            self.offset_z.setValue(-100)
        
        offset_form = QFormLayout()
        offset_form.addRow("X Offset:", self.offset_x)
        offset_form.addRow("Y Offset:", self.offset_y)
        offset_form.addRow("Z Offset:", self.offset_z)
        
        plot_layout = QHBoxLayout()
        plot_layout.addWidget(self.plot)
        plot_layout.addWidget(self.z_slider)
        
        main_layout = QVBoxLayout()
        main_layout.addLayout(plot_layout)
        main_layout.addWidget(self.position_label)
        main_layout.addWidget(self.adjusted_label)
        main_layout.addLayout(offset_form)
        
        self.setLayout(main_layout)
    
    def on_position_changed(self):
        x, y = self.plot.get_position()
        z = self.z_slider.value()
        
        self.position_label.setText(f"Raw: X: {int(x)}, Y: {int(y)}, Z: {z}")
        
        adj_x = x + self.offset_x.value()
        adj_y = y + self.offset_y.value()
        adj_z = z + self.offset_z.value()
        
        self.adjusted_label.setText(f"Adjusted: X: {int(adj_x)}, Y: {int(adj_y)}, Z: {int(adj_z)}")
        
        self.positionChanged.emit()
    
    def get_position(self):
        x, y = self.plot.get_position()
        z = self.z_slider.value()
        return (x + self.offset_x.value(), 
                y + self.offset_y.value(), 
                z + self.offset_z.value())
    
    def get_command(self):
        x, y, z = self.get_position()
        return f"L {self.quadrant} {int(x)} {int(y)} {int(z)}"

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Leg Controller")
        self.setGeometry(100, 100, 1200, 900)
        
        # Create leg controls
        self.legs = [
            LegControl("Front Right", 1),
            LegControl("Front Left", 2),
            LegControl("Rear Left", 3),
            LegControl("Rear Right", 4)
        ]
        
        # Connect each leg's positionChanged signal
        for leg in self.legs:
            leg.positionChanged.connect(self.handle_position_change)
        
        # Serial log
        self.serial_log = QTextEdit()
        self.serial_log.setReadOnly(True)
        self.serial_log.setFont(QFont("Courier", 10))
        self.serial_log.setMinimumHeight(200)
        
        # Serial controls
        self.port_combo = QComboBox()
        self.port_combo.addItems(["COM1", "COM2", "COM3", "COM4", "COM5"])
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("115200")
        
        self.send_button = QPushButton("Send All Legs")
        self.send_button.clicked.connect(self.send_all_commands)
        
        # Add "Send on update" checkbox
        self.send_on_update = QCheckBox("Send on update")
        self.send_on_update.setChecked(False)
        
        # Add clear log button
        self.clear_log_button = QPushButton("Clear Log")
        self.clear_log_button.clicked.connect(self.clear_serial_log)
        
        # Add open/close serial button
        self.serial_toggle = QPushButton("Open Serial")
        self.serial_toggle.clicked.connect(self.toggle_serial)
        self.serial_connection = None
        
        # Serial response timer
        self.response_timer = QTimer()
        self.response_timer.timeout.connect(self.check_serial_responses)
        self.response_timer.setInterval(100)  # Check every 100ms
        
        serial_layout = QHBoxLayout()
        serial_layout.addWidget(QLabel("Serial Port:"))
        serial_layout.addWidget(self.port_combo)
        serial_layout.addWidget(QLabel("Baud Rate:"))
        serial_layout.addWidget(self.baud_combo)
        serial_layout.addWidget(self.serial_toggle)
        serial_layout.addWidget(self.send_button)
        serial_layout.addWidget(self.send_on_update)
        serial_layout.addWidget(self.clear_log_button)
        serial_layout.addStretch()
        
        # Main layout
        grid = QGridLayout()
        grid.addWidget(self.legs[1], 0, 0)
        grid.addWidget(self.legs[0], 0, 1)
        grid.addWidget(self.legs[2], 1, 0)
        grid.addWidget(self.legs[3], 1, 1)
        
        main_layout = QVBoxLayout()
        main_layout.addLayout(serial_layout)
        main_layout.addLayout(grid)
        main_layout.addWidget(QLabel("Serial Log:"))
        main_layout.addWidget(self.serial_log)
        
        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)
        
        # Status bar
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("Ready")
    
    def handle_position_change(self):
        """Handle position changes without rate limiting"""
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
        
        # Auto-scroll to bottom
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
            # Add newline to command
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
                timeout=0.1  # Non-blocking read
            )
            self.log_message(f"Serial port opened: {port} @ {baud} baud", "info")
            self.status_bar.showMessage(f"Connected to {port} @ {baud} baud")
            
            # Start checking for responses
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
            # Read all available data
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