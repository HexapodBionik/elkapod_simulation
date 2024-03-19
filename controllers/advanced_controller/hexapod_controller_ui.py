# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'hexapod_controller.ui'
##
## Created by: Qt User Interface Compiler version 6.6.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QComboBox, QDial, QGraphicsView,
    QHBoxLayout, QLabel, QMainWindow, QSizePolicy,
    QSlider, QSpacerItem, QVBoxLayout, QWidget)

class Ui_HexapodController(object):
    def setupUi(self, HexapodController):
        if not HexapodController.objectName():
            HexapodController.setObjectName(u"HexapodController")
        HexapodController.resize(554, 603)
        self.central_widget = QWidget(HexapodController)
        self.central_widget.setObjectName(u"central_widget")
        self.verticalLayout_2 = QVBoxLayout(self.central_widget)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.leg_spacing_widget = QWidget(self.central_widget)
        self.leg_spacing_widget.setObjectName(u"leg_spacing_widget")
        self.leg_spacing_widget.setEnabled(True)
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.leg_spacing_widget.sizePolicy().hasHeightForWidth())
        self.leg_spacing_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout_2 = QHBoxLayout(self.leg_spacing_widget)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.leg_spacing_view = QGraphicsView(self.leg_spacing_widget)
        self.leg_spacing_view.setObjectName(u"leg_spacing_view")
        self.leg_spacing_view.setEnabled(True)
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.leg_spacing_view.sizePolicy().hasHeightForWidth())
        self.leg_spacing_view.setSizePolicy(sizePolicy1)
        self.leg_spacing_view.setMinimumSize(QSize(220, 80))
        self.leg_spacing_view.setMaximumSize(QSize(220, 80))

        self.horizontalLayout_2.addWidget(self.leg_spacing_view)

        self.leg_spacing_slider = QSlider(self.leg_spacing_widget)
        self.leg_spacing_slider.setObjectName(u"leg_spacing_slider")
        self.leg_spacing_slider.setEnabled(True)
        self.leg_spacing_slider.setMouseTracking(True)
        self.leg_spacing_slider.setMinimum(40)
        self.leg_spacing_slider.setMaximum(200)
        self.leg_spacing_slider.setValue(120)
        self.leg_spacing_slider.setOrientation(Qt.Horizontal)

        self.horizontalLayout_2.addWidget(self.leg_spacing_slider)


        self.verticalLayout_2.addWidget(self.leg_spacing_widget)

        self.height_widget = QWidget(self.central_widget)
        self.height_widget.setObjectName(u"height_widget")
        sizePolicy.setHeightForWidth(self.height_widget.sizePolicy().hasHeightForWidth())
        self.height_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout = QHBoxLayout(self.height_widget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.height_view = QGraphicsView(self.height_widget)
        self.height_view.setObjectName(u"height_view")
        self.height_view.setMinimumSize(QSize(220, 80))
        self.height_view.setMaximumSize(QSize(220, 80))

        self.horizontalLayout.addWidget(self.height_view)

        self.height_slider = QSlider(self.height_widget)
        self.height_slider.setObjectName(u"height_slider")
        self.height_slider.setMaximum(230)
        self.height_slider.setValue(100)
        self.height_slider.setOrientation(Qt.Horizontal)

        self.horizontalLayout.addWidget(self.height_slider)


        self.verticalLayout_2.addWidget(self.height_widget)

        self.velocity_widget = QWidget(self.central_widget)
        self.velocity_widget.setObjectName(u"velocity_widget")
        sizePolicy.setHeightForWidth(self.velocity_widget.sizePolicy().hasHeightForWidth())
        self.velocity_widget.setSizePolicy(sizePolicy)
        self.horizontalLayout_4 = QHBoxLayout(self.velocity_widget)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.widget_3 = QWidget(self.velocity_widget)
        self.widget_3.setObjectName(u"widget_3")
        self.verticalLayout_3 = QVBoxLayout(self.widget_3)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalSpacer_4 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_4)

        self.vdir_dial = QDial(self.widget_3)
        self.vdir_dial.setObjectName(u"vdir_dial")
        self.vdir_dial.setMinimumSize(QSize(80, 80))
        self.vdir_dial.setMaximumSize(QSize(80, 16777215))
        self.vdir_dial.setMaximum(360)
        self.vdir_dial.setValue(180)
        self.vdir_dial.setWrapping(True)
        self.vdir_dial.setNotchesVisible(True)

        self.verticalLayout_3.addWidget(self.vdir_dial)

        self.verticalSpacer_5 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3.addItem(self.verticalSpacer_5)

        self.label = QLabel(self.widget_3)
        self.label.setObjectName(u"label")
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_3.addWidget(self.label)


        self.horizontalLayout_4.addWidget(self.widget_3)

        self.widget_2 = QWidget(self.velocity_widget)
        self.widget_2.setObjectName(u"widget_2")
        self.verticalLayout = QVBoxLayout(self.widget_2)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalSpacer_2 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_2)

        self.vval_slider = QSlider(self.widget_2)
        self.vval_slider.setObjectName(u"vval_slider")
        self.vval_slider.setMaximum(100)
        self.vval_slider.setOrientation(Qt.Horizontal)

        self.verticalLayout.addWidget(self.vval_slider)

        self.verticalSpacer_3 = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer_3)

        self.label_2 = QLabel(self.widget_2)
        self.label_2.setObjectName(u"label_2")
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.verticalLayout.addWidget(self.label_2)


        self.horizontalLayout_4.addWidget(self.widget_2)


        self.verticalLayout_2.addWidget(self.velocity_widget)

        self.widget = QWidget(self.central_widget)
        self.widget.setObjectName(u"widget")
        self.horizontalLayout_3 = QHBoxLayout(self.widget)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.widget_4 = QWidget(self.widget)
        self.widget_4.setObjectName(u"widget_4")
        self.verticalLayout_4 = QVBoxLayout(self.widget_4)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.omega_dial = QDial(self.widget_4)
        self.omega_dial.setObjectName(u"omega_dial")
        self.omega_dial.setMinimumSize(QSize(80, 80))
        self.omega_dial.setMaximumSize(QSize(80, 80))
        self.omega_dial.setMinimum(-17)
        self.omega_dial.setMaximum(17)
        self.omega_dial.setNotchesVisible(True)

        self.verticalLayout_4.addWidget(self.omega_dial)

        self.label_3 = QLabel(self.widget_4)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_4.addWidget(self.label_3)


        self.horizontalLayout_3.addWidget(self.widget_4)

        self.widget_5 = QWidget(self.widget)
        self.widget_5.setObjectName(u"widget_5")
        self.verticalLayout_5 = QVBoxLayout(self.widget_5)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.yaw_dial = QDial(self.widget_5)
        self.yaw_dial.setObjectName(u"yaw_dial")
        self.yaw_dial.setMinimumSize(QSize(80, 80))
        self.yaw_dial.setMaximumSize(QSize(80, 80))
        self.yaw_dial.setMinimum(-120)
        self.yaw_dial.setMaximum(120)
        self.yaw_dial.setNotchesVisible(True)

        self.verticalLayout_5.addWidget(self.yaw_dial)

        self.yaw_label = QLabel(self.widget_5)
        self.yaw_label.setObjectName(u"yaw_label")
        self.yaw_label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_5.addWidget(self.yaw_label)


        self.horizontalLayout_3.addWidget(self.widget_5)

        self.widget_6 = QWidget(self.widget)
        self.widget_6.setObjectName(u"widget_6")
        self.verticalLayout_6 = QVBoxLayout(self.widget_6)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.pitch_dial = QDial(self.widget_6)
        self.pitch_dial.setObjectName(u"pitch_dial")
        self.pitch_dial.setMinimumSize(QSize(80, 80))
        self.pitch_dial.setMaximumSize(QSize(80, 80))
        self.pitch_dial.setMinimum(-250)
        self.pitch_dial.setMaximum(250)
        self.pitch_dial.setNotchesVisible(True)

        self.verticalLayout_6.addWidget(self.pitch_dial)

        self.label_4 = QLabel(self.widget_6)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setAlignment(Qt.AlignCenter)

        self.verticalLayout_6.addWidget(self.label_4)


        self.horizontalLayout_3.addWidget(self.widget_6)

        self.widget_7 = QWidget(self.widget)
        self.widget_7.setObjectName(u"widget_7")
        self.verticalLayout_7 = QVBoxLayout(self.widget_7)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.roll_dial = QDial(self.widget_7)
        self.roll_dial.setObjectName(u"roll_dial")
        self.roll_dial.setMinimumSize(QSize(80, 80))
        self.roll_dial.setMaximumSize(QSize(80, 80))
        self.roll_dial.setMinimum(-120)
        self.roll_dial.setMaximum(120)
        self.roll_dial.setNotchesVisible(True)

        self.verticalLayout_7.addWidget(self.roll_dial)

        self.label_5 = QLabel(self.widget_7)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setAlignment(Qt.AlignCenter)

        self.verticalLayout_7.addWidget(self.label_5)


        self.horizontalLayout_3.addWidget(self.widget_7)

        self.widget_8 = QWidget(self.widget)
        self.widget_8.setObjectName(u"widget_8")
        self.verticalLayout_8 = QVBoxLayout(self.widget_8)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.step_height_slider = QSlider(self.widget_8)
        self.step_height_slider.setObjectName(u"step_height_slider")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.step_height_slider.sizePolicy().hasHeightForWidth())
        self.step_height_slider.setSizePolicy(sizePolicy2)
        self.step_height_slider.setMinimumSize(QSize(0, 80))
        self.step_height_slider.setMaximumSize(QSize(16777215, 80))
        self.step_height_slider.setMaximum(150)
        self.step_height_slider.setOrientation(Qt.Horizontal)

        self.verticalLayout_8.addWidget(self.step_height_slider)

        self.label_6 = QLabel(self.widget_8)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setAlignment(Qt.AlignCenter)

        self.verticalLayout_8.addWidget(self.label_6)


        self.horizontalLayout_3.addWidget(self.widget_8)


        self.verticalLayout_2.addWidget(self.widget)

        self.widget_9 = QWidget(self.central_widget)
        self.widget_9.setObjectName(u"widget_9")
        self.horizontalLayout_5 = QHBoxLayout(self.widget_9)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.widget_10 = QWidget(self.widget_9)
        self.widget_10.setObjectName(u"widget_10")
        self.horizontalLayout_6 = QHBoxLayout(self.widget_10)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_7 = QLabel(self.widget_10)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_6.addWidget(self.label_7)

        self.gait_selection = QComboBox(self.widget_10)
        self.gait_selection.setObjectName(u"gait_selection")

        self.horizontalLayout_6.addWidget(self.gait_selection)


        self.horizontalLayout_5.addWidget(self.widget_10)

        self.widget_11 = QWidget(self.widget_9)
        self.widget_11.setObjectName(u"widget_11")
        self.horizontalLayout_7 = QHBoxLayout(self.widget_11)
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_8 = QLabel(self.widget_11)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_7.addWidget(self.label_8)

        self.mode_selection = QComboBox(self.widget_11)
        self.mode_selection.setObjectName(u"mode_selection")

        self.horizontalLayout_7.addWidget(self.mode_selection)


        self.horizontalLayout_5.addWidget(self.widget_11)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_5.addItem(self.horizontalSpacer)


        self.verticalLayout_2.addWidget(self.widget_9)

        self.verticalSpacer = QSpacerItem(20, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)

        HexapodController.setCentralWidget(self.central_widget)

        self.retranslateUi(HexapodController)

        self.gait_selection.setCurrentIndex(-1)


        QMetaObject.connectSlotsByName(HexapodController)
    # setupUi

    def retranslateUi(self, HexapodController):
        HexapodController.setWindowTitle(QCoreApplication.translate("HexapodController", u"Hexapod Controller", None))
        self.label.setText(QCoreApplication.translate("HexapodController", u"v direction", None))
        self.label_2.setText(QCoreApplication.translate("HexapodController", u"v value", None))
        self.label_3.setText(QCoreApplication.translate("HexapodController", u"omega", None))
        self.yaw_label.setText(QCoreApplication.translate("HexapodController", u"yaw", None))
        self.label_4.setText(QCoreApplication.translate("HexapodController", u"pitch", None))
        self.label_5.setText(QCoreApplication.translate("HexapodController", u"roll", None))
        self.label_6.setText(QCoreApplication.translate("HexapodController", u"step height", None))
        self.label_7.setText(QCoreApplication.translate("HexapodController", u"Gait", None))
        self.gait_selection.setCurrentText("")
        self.label_8.setText(QCoreApplication.translate("HexapodController", u"Mode", None))
    # retranslateUi

