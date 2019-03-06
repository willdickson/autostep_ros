from __future__ import print_function
import sys
from PyQt5 import QtCore
from PyQt5 import QtGui 
from PyQt5 import QtWidgets
from main_window_ui import Ui_MainWindow

from autostep_proxy import AutostepProxy


class AutostepProxyApp(QtWidgets.QMainWindow, Ui_MainWindow):

    def __init__(self, parent=None):
        super(AutostepProxyApp,self).__init__(parent)
        self.setupUi(self)
        self.initialize_ui()

        self.tracking_mode_disable_list = [ 
                self.currentPositionGroupBox,
                self.setToGroupBox, 
                self.drivePowerGroupBox,
                self.jogPositionGroupBox,
                self.moveToPositionGroupBox,
                self.runGroupBox,
                self.stopGroupBox,
                self.runGroupBox,
                self.stopGroupBox,
                ]

        self.busy_wait_disable_list = [ 
                self.setToGroupBox, 
                self.drivePowerGroupBox,
                self.jogPositionGroupBox,
                self.moveToPositionGroupBox,
                self.runGroupBox,
                self.trackingModeGroupBox,
                ]

        self.autostep_proxy = AutostepProxy()
        self.enable_drive_power()
        self.drivePowerCheckBox.setChecked(True)
        self.set_tracking_mode(False)
        self.trackingModeCheckBox.setChecked(False)
        self.positionPollCheckBox.setChecked(True)
        self.running = False

    def initialize_ui(self):
        self.set_lcd_colors()
        self.timer = QtCore.QTimer()
        self.timer_period = 100
        self.timer.start(self.timer_period)

        self.setToLineEdit.setText(str(0))
        self.setToLineEditValidator = QtGui.QDoubleValidator()
        self.setToLineEdit.setValidator(self.setToLineEditValidator)

        self.jogStepLineEdit.setText(str(10))
        self.jogStepLineEditValidator = QtGui.QDoubleValidator()
        self.jogStepLineEdit.setValidator(self.jogStepLineEditValidator)

        self.moveToLineEdit.setText(str(0))
        self.moveToLineEditValidator = QtGui.QDoubleValidator()
        self.moveToLineEdit.setValidator(self.moveToLineEditValidator)

        self.runLineEdit.setText(str(50))
        self.runLineEditValidator = QtGui.QDoubleValidator()
        self.runLineEdit.setValidator(self.runLineEditValidator)

        self.connect_ui_actions()

    def set_lcd_colors(self):
        palette = self.currentPositionLcdNumber.palette()
        palette.setColor(palette.WindowText, QtGui.QColor(0, 200, 255))
        palette.setColor(palette.Light, QtGui.QColor(0, 0, 0))
        palette.setColor(palette.Dark, QtGui.QColor(0, 0, 0))
        palette.setColor(palette.Background, QtGui.QColor(0, 0, 0))
        self.currentPositionLcdNumber.setPalette(palette)

    def connect_ui_actions(self):
        self.timer.timeout.connect(self.on_timer)
        self.setToPushButton.clicked.connect(self.on_setTo_button_clicked)
        self.jogPosPushButton.clicked.connect(self.on_jog_pos_button_clicked)
        self.jogNegPushButton.clicked.connect(self.on_jog_neg_button_clicked)
        self.moveToPushButton.clicked.connect(self.on_move_to_button_clicked)
        self.drivePowerCheckBox.stateChanged.connect(self.on_drive_power_changed)
        self.trackingModeCheckBox.stateChanged.connect(self.on_tracking_mode_changed)
        self.stopPushButton.clicked.connect(self.on_stop_button_clicked)
        self.runPushButton.clicked.connect(self.on_run_button_clicked)

    def on_setTo_button_clicked(self):
        value = float(self.setToLineEdit.text())
        self.set_position(value)

    def on_jog_pos_button_clicked(self):
        value = float(self.jogStepLineEdit.text())
        self.jog_position(value)

    def on_jog_neg_button_clicked(self):
        value = -float(self.jogStepLineEdit.text())
        self.jog_position(value)

    def on_move_to_button_clicked(self):
        value = float(self.moveToLineEdit.text())
        self.move_to_position(value)

    def on_drive_power_changed(self, value):
        if value == QtCore.Qt.Checked:
            self.enable_drive_power()
        else:
            self.disable_drive_power()

    def on_tracking_mode_changed(self, value):
        if value == QtCore.Qt.Checked:
            self.tracking_mode_enabled = True
            self.autostep_proxy.enable_tracking_mode()
        else:
            self.autostep_proxy.disable_tracking_mode()
            self.tracking_mode_enabled = False
        self.set_widget_enabled_for_tracking_mode()


    def on_stop_button_clicked(self):
        self.autostep_proxy.soft_stop()
        self.running = False

    def on_run_button_clicked(self):
        value = float(self.runLineEdit.text())
        self.running = True
        self.autostep_proxy.run(value)

    def on_timer(self):
        position_str = ''
        if self.position_poll_enabled():
            position = self.autostep_proxy.get_position()
            if position is not None:
                position = round(position,1)
                position_str = '{:0.1f}'.format(position)
        self.currentPositionLcdNumber.display(position_str)

        proxy_is_busy = self.autostep_proxy.is_busy()
        self.set_widget_enabled_for_busy(proxy_is_busy or self.running)

    def set_widget_enabled_for_tracking_mode(self):
        if self.tracking_mode_enabled:
            for widget in self.tracking_mode_disable_list:
                self.timer.stop()
                self.currentPositionLcdNumber.display('')
                widget.setEnabled(False)
        else:
            for widget in self.tracking_mode_disable_list:
                widget.setEnabled(True)
                self.timer.start(self.timer_period)

    def enable_drive_power(self):
        self.autostep_proxy.enable()

    def disable_drive_power(self):
        self.autostep_proxy.release()

    def set_position(self,value):
        self.autostep_proxy.set_position(value)

    def move_to_position(self,value):
        self.autostep_proxy.move_to(value)

    def jog_position(self,value):
        self.autostep_proxy.set_move_mode('jog')
        self.autostep_proxy.move_by(value)

    def set_tracking_mode(self, value):
        if value:
            self.autostep_proxy.enable_tracking_mode()
            self.tracking_mode_enabled = True
        else:
            self.autostep_proxy.disable_tracking_mode()
            self.tracking_mode_enabled = False 

    def set_widget_enabled_for_busy(self, is_busy):
        if is_busy:
            for widget in self.busy_wait_disable_list:
                widget.setEnabled(False)
        else:
            for widget in self.busy_wait_disable_list:
                widget.setEnabled(True)

    def position_poll_enabled(self):
        return self.positionPollCheckBox.isChecked()



def app_main():
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = AutostepProxyApp()
    mainWindow.show()
    app.exec_()


# -------------------------------------------------------------------------------------------------
if __name__ == '__main__':

    app_main()


