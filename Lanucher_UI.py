# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'Lanucher_UI.ui'
##
## Created by: Qt User Interface Compiler version 6.10.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QAction, QBrush, QColor, QConicalGradient,
    QCursor, QFont, QFontDatabase, QGradient,
    QIcon, QImage, QKeySequence, QLinearGradient,
    QPainter, QPalette, QPixmap, QRadialGradient,
    QTransform)
from PySide6.QtWidgets import (QAbstractItemView, QAbstractSpinBox, QApplication, QCheckBox,
    QComboBox, QDoubleSpinBox, QFormLayout, QFrame,
    QGridLayout, QGroupBox, QHBoxLayout, QHeaderView,
    QLabel, QLineEdit, QMainWindow, QMenu,
    QMenuBar, QPlainTextEdit, QProgressBar, QPushButton,
    QRadioButton, QSizePolicy, QSlider, QSpacerItem,
    QSpinBox, QStatusBar, QTabWidget, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(1117, 940)
        font = QFont()
        font.setFamilies([u"MS Shell Dlg 2"])
        font.setPointSize(10)
        MainWindow.setFont(font)
        self.action_Load = QAction(MainWindow)
        self.action_Load.setObjectName(u"action_Load")
        self.action_Save = QAction(MainWindow)
        self.action_Save.setObjectName(u"action_Save")
        self.action_Exit = QAction(MainWindow)
        self.action_Exit.setObjectName(u"action_Exit")
        self.action_OpenSerial = QAction(MainWindow)
        self.action_OpenSerial.setObjectName(u"action_OpenSerial")
        self.action_SearchDev = QAction(MainWindow)
        self.action_SearchDev.setObjectName(u"action_SearchDev")
        self.action_Protocol = QAction(MainWindow)
        self.action_Protocol.setObjectName(u"action_Protocol")
        self.action_About = QAction(MainWindow)
        self.action_About.setObjectName(u"action_About")
        self.action = QAction(MainWindow)
        self.action.setObjectName(u"action")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout_Main = QHBoxLayout(self.centralwidget)
        self.horizontalLayout_Main.setSpacing(8)
        self.horizontalLayout_Main.setObjectName(u"horizontalLayout_Main")
        self.horizontalLayout_Main.setContentsMargins(8, 8, 8, 8)
        self.frame_Sidebar = QFrame(self.centralwidget)
        self.frame_Sidebar.setObjectName(u"frame_Sidebar")
        self.frame_Sidebar.setMinimumSize(QSize(260, 0))
        self.frame_Sidebar.setMaximumSize(QSize(260, 16777215))
        self.frame_Sidebar.setFrameShape(QFrame.Shape.StyledPanel)
        self.frame_Sidebar.setFrameShadow(QFrame.Shadow.Raised)
        self.verticalLayout_Side = QVBoxLayout(self.frame_Sidebar)
        self.verticalLayout_Side.setObjectName(u"verticalLayout_Side")
        self.grp_Serial = QGroupBox(self.frame_Sidebar)
        self.grp_Serial.setObjectName(u"grp_Serial")
        self.formLayout_Serial = QFormLayout(self.grp_Serial)
        self.formLayout_Serial.setObjectName(u"formLayout_Serial")
        self.lbl_Port = QLabel(self.grp_Serial)
        self.lbl_Port.setObjectName(u"lbl_Port")

        self.formLayout_Serial.setWidget(0, QFormLayout.ItemRole.LabelRole, self.lbl_Port)

        self.combo_Port = QComboBox(self.grp_Serial)
        self.combo_Port.addItem("")
        self.combo_Port.setObjectName(u"combo_Port")

        self.formLayout_Serial.setWidget(0, QFormLayout.ItemRole.FieldRole, self.combo_Port)

        self.lbl_CtrlVer = QLabel(self.grp_Serial)
        self.lbl_CtrlVer.setObjectName(u"lbl_CtrlVer")

        self.formLayout_Serial.setWidget(1, QFormLayout.ItemRole.LabelRole, self.lbl_CtrlVer)

        self.comboBox = QComboBox(self.grp_Serial)
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.comboBox.setObjectName(u"comboBox")

        self.formLayout_Serial.setWidget(1, QFormLayout.ItemRole.FieldRole, self.comboBox)

        self.lbl_CtrlLink = QLabel(self.grp_Serial)
        self.lbl_CtrlLink.setObjectName(u"lbl_CtrlLink")

        self.formLayout_Serial.setWidget(2, QFormLayout.ItemRole.LabelRole, self.lbl_CtrlLink)

        self.lbl_CtrlLinkState = QLabel(self.grp_Serial)
        self.lbl_CtrlLinkState.setObjectName(u"lbl_CtrlLinkState")
        font1 = QFont()
        font1.setFamilies([u"MS Shell Dlg 2"])
        font1.setPointSize(10)
        font1.setBold(True)
        self.lbl_CtrlLinkState.setFont(font1)
        self.lbl_CtrlLinkState.setStyleSheet(u"color: rgb(210, 38, 38);")
        self.lbl_CtrlLinkState.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.formLayout_Serial.setWidget(2, QFormLayout.ItemRole.FieldRole, self.lbl_CtrlLinkState)

        self.btn_OpenSerial = QPushButton(self.grp_Serial)
        self.btn_OpenSerial.setObjectName(u"btn_OpenSerial")
        self.btn_OpenSerial.setMinimumSize(QSize(0, 35))
        self.btn_OpenSerial.setCheckable(True)

        self.formLayout_Serial.setWidget(3, QFormLayout.ItemRole.SpanningRole, self.btn_OpenSerial)


        self.verticalLayout_Side.addWidget(self.grp_Serial)

        self.grp_Controller = QGroupBox(self.frame_Sidebar)
        self.grp_Controller.setObjectName(u"grp_Controller")
        self.grp_Controller.setMinimumSize(QSize(0, 0))
        self.gridLayout_3 = QGridLayout(self.grp_Controller)
        self.gridLayout_3.setObjectName(u"gridLayout_3")
        self.lbl_CtrlID = QLabel(self.grp_Controller)
        self.lbl_CtrlID.setObjectName(u"lbl_CtrlID")

        self.gridLayout_3.addWidget(self.lbl_CtrlID, 0, 0, 1, 1)

        self.lbl_CtrlID_Val = QLineEdit(self.grp_Controller)
        self.lbl_CtrlID_Val.setObjectName(u"lbl_CtrlID_Val")
        self.lbl_CtrlID_Val.setMinimumSize(QSize(90, 30))
        self.lbl_CtrlID_Val.setMaximumSize(QSize(16777215, 30))
        self.lbl_CtrlID_Val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.gridLayout_3.addWidget(self.lbl_CtrlID_Val, 0, 1, 1, 1)

        self.lbl_CtrlCh = QLabel(self.grp_Controller)
        self.lbl_CtrlCh.setObjectName(u"lbl_CtrlCh")

        self.gridLayout_3.addWidget(self.lbl_CtrlCh, 1, 0, 1, 1)

        self.spin_CtrlCh = QSpinBox(self.grp_Controller)
        self.spin_CtrlCh.setObjectName(u"spin_CtrlCh")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.spin_CtrlCh.sizePolicy().hasHeightForWidth())
        self.spin_CtrlCh.setSizePolicy(sizePolicy)
        self.spin_CtrlCh.setMinimumSize(QSize(55, 30))
        self.spin_CtrlCh.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_CtrlCh.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.spin_CtrlCh.setMaximum(83)
        self.spin_CtrlCh.setValue(23)

        self.gridLayout_3.addWidget(self.spin_CtrlCh, 1, 1, 1, 1)

        self.btn_SetCtrlID = QPushButton(self.grp_Controller)
        self.btn_SetCtrlID.setObjectName(u"btn_SetCtrlID")
        self.btn_SetCtrlID.setMinimumSize(QSize(0, 32))

        self.gridLayout_3.addWidget(self.btn_SetCtrlID, 2, 0, 1, 1)

        self.btn_SetCtrlCh = QPushButton(self.grp_Controller)
        self.btn_SetCtrlCh.setObjectName(u"btn_SetCtrlCh")
        self.btn_SetCtrlCh.setMinimumSize(QSize(0, 32))

        self.gridLayout_3.addWidget(self.btn_SetCtrlCh, 2, 1, 1, 1)

        self.btn_InitCtrl = QPushButton(self.grp_Controller)
        self.btn_InitCtrl.setObjectName(u"btn_InitCtrl")
        self.btn_InitCtrl.setMinimumSize(QSize(106, 32))

        self.gridLayout_3.addWidget(self.btn_InitCtrl, 3, 0, 1, 1)

        self.btn_QueryCtrl = QPushButton(self.grp_Controller)
        self.btn_QueryCtrl.setObjectName(u"btn_QueryCtrl")
        self.btn_QueryCtrl.setMinimumSize(QSize(0, 32))

        self.gridLayout_3.addWidget(self.btn_QueryCtrl, 3, 1, 1, 1)


        self.verticalLayout_Side.addWidget(self.grp_Controller)

        self.groupBox_SecondPWD = QGroupBox(self.frame_Sidebar)
        self.groupBox_SecondPWD.setObjectName(u"groupBox_SecondPWD")
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_SecondPWD)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.lineEdit_SecondPWD = QLineEdit(self.groupBox_SecondPWD)
        self.lineEdit_SecondPWD.setObjectName(u"lineEdit_SecondPWD")
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.lineEdit_SecondPWD.sizePolicy().hasHeightForWidth())
        self.lineEdit_SecondPWD.setSizePolicy(sizePolicy1)
        self.lineEdit_SecondPWD.setMinimumSize(QSize(0, 30))
        self.lineEdit_SecondPWD.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_6.addWidget(self.lineEdit_SecondPWD)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.pushButton_SecondPWD_random = QPushButton(self.groupBox_SecondPWD)
        self.pushButton_SecondPWD_random.setObjectName(u"pushButton_SecondPWD_random")

        self.horizontalLayout.addWidget(self.pushButton_SecondPWD_random)

        self.pushButton_SecondPWD_set = QPushButton(self.groupBox_SecondPWD)
        self.pushButton_SecondPWD_set.setObjectName(u"pushButton_SecondPWD_set")
        self.pushButton_SecondPWD_set.setMinimumSize(QSize(0, 30))

        self.horizontalLayout.addWidget(self.pushButton_SecondPWD_set)


        self.verticalLayout_6.addLayout(self.horizontalLayout)


        self.verticalLayout_Side.addWidget(self.groupBox_SecondPWD)

        self.grp_Devices = QGroupBox(self.frame_Sidebar)
        self.grp_Devices.setObjectName(u"grp_Devices")
        self.grp_Devices.setMinimumSize(QSize(0, 0))
        self.verticalLayout = QVBoxLayout(self.grp_Devices)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout_DevBtns = QHBoxLayout()
        self.horizontalLayout_DevBtns.setSpacing(2)
        self.horizontalLayout_DevBtns.setObjectName(u"horizontalLayout_DevBtns")
        self.btn_Search = QPushButton(self.grp_Devices)
        self.btn_Search.setObjectName(u"btn_Search")

        self.horizontalLayout_DevBtns.addWidget(self.btn_Search)

        self.btn_AddDevManual = QPushButton(self.grp_Devices)
        self.btn_AddDevManual.setObjectName(u"btn_AddDevManual")

        self.horizontalLayout_DevBtns.addWidget(self.btn_AddDevManual)


        self.verticalLayout.addLayout(self.horizontalLayout_DevBtns)

        self.table_Devices = QTableWidget(self.grp_Devices)
        if (self.table_Devices.columnCount() < 2):
            self.table_Devices.setColumnCount(2)
        __qtablewidgetitem = QTableWidgetItem()
        self.table_Devices.setHorizontalHeaderItem(0, __qtablewidgetitem)
        __qtablewidgetitem1 = QTableWidgetItem()
        self.table_Devices.setHorizontalHeaderItem(1, __qtablewidgetitem1)
        self.table_Devices.setObjectName(u"table_Devices")
        self.table_Devices.setStyleSheet(u"QTableWidget::item { text-align: center; }")
        self.table_Devices.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.table_Devices.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table_Devices.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table_Devices.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_Devices.setShowGrid(True)
        self.table_Devices.setColumnCount(2)
        self.table_Devices.horizontalHeader().setVisible(True)
        self.table_Devices.horizontalHeader().setDefaultSectionSize(80)
        self.table_Devices.horizontalHeader().setStretchLastSection(True)
        self.table_Devices.verticalHeader().setVisible(False)

        self.verticalLayout.addWidget(self.table_Devices)

        self.formLayout_2 = QFormLayout()
        self.formLayout_2.setObjectName(u"formLayout_2")
        self.btn_SelectAll = QPushButton(self.grp_Devices)
        self.btn_SelectAll.setObjectName(u"btn_SelectAll")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Fixed)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.btn_SelectAll.sizePolicy().hasHeightForWidth())
        self.btn_SelectAll.setSizePolicy(sizePolicy2)
        self.btn_SelectAll.setMinimumSize(QSize(106, 32))

        self.formLayout_2.setWidget(0, QFormLayout.ItemRole.LabelRole, self.btn_SelectAll)

        self.btn_SelectNone = QPushButton(self.grp_Devices)
        self.btn_SelectNone.setObjectName(u"btn_SelectNone")
        self.btn_SelectNone.setMinimumSize(QSize(0, 32))

        self.formLayout_2.setWidget(0, QFormLayout.ItemRole.FieldRole, self.btn_SelectNone)

        self.btn_SelectNone_2 = QPushButton(self.grp_Devices)
        self.btn_SelectNone_2.setObjectName(u"btn_SelectNone_2")
        sizePolicy2.setHeightForWidth(self.btn_SelectNone_2.sizePolicy().hasHeightForWidth())
        self.btn_SelectNone_2.setSizePolicy(sizePolicy2)
        self.btn_SelectNone_2.setMinimumSize(QSize(106, 32))

        self.formLayout_2.setWidget(1, QFormLayout.ItemRole.LabelRole, self.btn_SelectNone_2)

        self.btn_SelectNone_3 = QPushButton(self.grp_Devices)
        self.btn_SelectNone_3.setObjectName(u"btn_SelectNone_3")
        self.btn_SelectNone_3.setMinimumSize(QSize(0, 32))

        self.formLayout_2.setWidget(1, QFormLayout.ItemRole.FieldRole, self.btn_SelectNone_3)


        self.verticalLayout.addLayout(self.formLayout_2)

        self.label = QLabel(self.grp_Devices)
        self.label.setObjectName(u"label")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout.addWidget(self.label)


        self.verticalLayout_Side.addWidget(self.grp_Devices)

        self.verticalSpacer_Side = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_Side.addItem(self.verticalSpacer_Side)


        self.horizontalLayout_Main.addWidget(self.frame_Sidebar)

        self.verticalLayout_Main = QVBoxLayout()
        self.verticalLayout_Main.setSpacing(10)
        self.verticalLayout_Main.setObjectName(u"verticalLayout_Main")
        self.grp_Monitor = QGroupBox(self.centralwidget)
        self.grp_Monitor.setObjectName(u"grp_Monitor")
        self.grp_Monitor.setMinimumSize(QSize(0, 220))
        self.grp_Monitor.setMaximumSize(QSize(16777215, 16777215))
        self.verticalLayout_2 = QVBoxLayout(self.grp_Monitor)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.btn_GlobalStop_2 = QPushButton(self.grp_Monitor)
        self.btn_GlobalStop_2.setObjectName(u"btn_GlobalStop_2")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.btn_GlobalStop_2.sizePolicy().hasHeightForWidth())
        self.btn_GlobalStop_2.setSizePolicy(sizePolicy3)
        self.btn_GlobalStop_2.setMinimumSize(QSize(0, 32))
        self.btn_GlobalStop_2.setStyleSheet(u"QPushButton {\n"
"    background-color: rgb(200, 46, 46);\n"
"    color: white;\n"
"    font-weight: 700;\n"
"    border: 1px solid rgb(165, 38, 38);\n"
"    border-radius: 4px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(216, 58, 58);\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgb(164, 34, 34);\n"
"    border: 1px solid rgb(138, 26, 26);\n"
"    padding-top: 1px;\n"
"    padding-left: 1px;\n"
"}")

        self.horizontalLayout_2.addWidget(self.btn_GlobalStop_2)

        self.btn_GlobalStop = QPushButton(self.grp_Monitor)
        self.btn_GlobalStop.setObjectName(u"btn_GlobalStop")
        sizePolicy3.setHeightForWidth(self.btn_GlobalStop.sizePolicy().hasHeightForWidth())
        self.btn_GlobalStop.setSizePolicy(sizePolicy3)
        self.btn_GlobalStop.setMinimumSize(QSize(0, 32))
        self.btn_GlobalStop.setStyleSheet(u"QPushButton {\n"
"    background-color: rgb(225, 58, 58);\n"
"    color: white;\n"
"    font-weight: 700;\n"
"    border: 1px solid rgb(175, 40, 40);\n"
"    border-radius: 4px;\n"
"}\n"
"QPushButton:hover {\n"
"    background-color: rgb(238, 72, 72);\n"
"}\n"
"QPushButton:pressed {\n"
"    background-color: rgb(186, 41, 41);\n"
"    border: 1px solid rgb(146, 30, 30);\n"
"    padding-top: 1px;\n"
"    padding-left: 1px;\n"
"}")

        self.horizontalLayout_2.addWidget(self.btn_GlobalStop)


        self.verticalLayout_2.addLayout(self.horizontalLayout_2)

        self.table_SysMonitor = QTableWidget(self.grp_Monitor)
        if (self.table_SysMonitor.columnCount() < 5):
            self.table_SysMonitor.setColumnCount(5)
        __qtablewidgetitem2 = QTableWidgetItem()
        self.table_SysMonitor.setHorizontalHeaderItem(0, __qtablewidgetitem2)
        __qtablewidgetitem3 = QTableWidgetItem()
        self.table_SysMonitor.setHorizontalHeaderItem(1, __qtablewidgetitem3)
        __qtablewidgetitem4 = QTableWidgetItem()
        self.table_SysMonitor.setHorizontalHeaderItem(2, __qtablewidgetitem4)
        __qtablewidgetitem5 = QTableWidgetItem()
        self.table_SysMonitor.setHorizontalHeaderItem(3, __qtablewidgetitem5)
        __qtablewidgetitem6 = QTableWidgetItem()
        self.table_SysMonitor.setHorizontalHeaderItem(4, __qtablewidgetitem6)
        self.table_SysMonitor.setObjectName(u"table_SysMonitor")
        self.table_SysMonitor.setStyleSheet(u"QTableWidget::item { text-align: center; }")
        self.table_SysMonitor.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table_SysMonitor.setAlternatingRowColors(True)
        self.table_SysMonitor.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_SysMonitor.setRowCount(0)
        self.table_SysMonitor.setColumnCount(5)
        self.table_SysMonitor.horizontalHeader().setVisible(True)
        self.table_SysMonitor.horizontalHeader().setMinimumSectionSize(33)
        self.table_SysMonitor.horizontalHeader().setDefaultSectionSize(120)
        self.table_SysMonitor.horizontalHeader().setHighlightSections(True)
        self.table_SysMonitor.horizontalHeader().setStretchLastSection(True)
        self.table_SysMonitor.verticalHeader().setVisible(False)
        self.table_SysMonitor.verticalHeader().setMinimumSectionSize(24)
        self.table_SysMonitor.verticalHeader().setDefaultSectionSize(30)

        self.verticalLayout_2.addWidget(self.table_SysMonitor)

        self.lbl_MonitorHint = QLabel(self.grp_Monitor)
        self.lbl_MonitorHint.setObjectName(u"lbl_MonitorHint")
        self.lbl_MonitorHint.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_2.addWidget(self.lbl_MonitorHint)


        self.verticalLayout_Main.addWidget(self.grp_Monitor)

        self.tabWidget_Main = QTabWidget(self.centralwidget)
        self.tabWidget_Main.setObjectName(u"tabWidget_Main")
        sizePolicy3.setHeightForWidth(self.tabWidget_Main.sizePolicy().hasHeightForWidth())
        self.tabWidget_Main.setSizePolicy(sizePolicy3)
        self.tab_SimpleMotion = QWidget()
        self.tab_SimpleMotion.setObjectName(u"tab_SimpleMotion")
        self.verticalLayout_Gear = QVBoxLayout(self.tab_SimpleMotion)
        self.verticalLayout_Gear.setObjectName(u"verticalLayout_Gear")
        self.grp_Gear_Ctrl = QGroupBox(self.tab_SimpleMotion)
        self.grp_Gear_Ctrl.setObjectName(u"grp_Gear_Ctrl")
        self.horizontalLayout_Dial = QHBoxLayout(self.grp_Gear_Ctrl)
        self.horizontalLayout_Dial.setSpacing(10)
        self.horizontalLayout_Dial.setObjectName(u"horizontalLayout_Dial")
        self.verticalLayout_SpeedDial = QVBoxLayout()
        self.verticalLayout_SpeedDial.setObjectName(u"verticalLayout_SpeedDial")
        self.lbl_SpeedDial = QLabel(self.grp_Gear_Ctrl)
        self.lbl_SpeedDial.setObjectName(u"lbl_SpeedDial")
        self.lbl_SpeedDial.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_SpeedDial.addWidget(self.lbl_SpeedDial)

        self.dial_Speed = QSlider(self.grp_Gear_Ctrl)
        self.dial_Speed.setObjectName(u"dial_Speed")
        self.dial_Speed.setMinimumSize(QSize(250, 34))
        self.dial_Speed.setMinimum(0)
        self.dial_Speed.setMaximum(15)
        self.dial_Speed.setOrientation(Qt.Orientation.Horizontal)
        self.dial_Speed.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.dial_Speed.setTickInterval(1)

        self.verticalLayout_SpeedDial.addWidget(self.dial_Speed)

        self.horizontalLayout_SpeedHint = QHBoxLayout()
        self.horizontalLayout_SpeedHint.setObjectName(u"horizontalLayout_SpeedHint")
        self.lbl_SpeedMin = QLabel(self.grp_Gear_Ctrl)
        self.lbl_SpeedMin.setObjectName(u"lbl_SpeedMin")

        self.horizontalLayout_SpeedHint.addWidget(self.lbl_SpeedMin)

        self.horizontalSpacer_SpeedHint1 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_SpeedHint.addItem(self.horizontalSpacer_SpeedHint1)

        self.lbl_SpeedMid = QLabel(self.grp_Gear_Ctrl)
        self.lbl_SpeedMid.setObjectName(u"lbl_SpeedMid")

        self.horizontalLayout_SpeedHint.addWidget(self.lbl_SpeedMid)

        self.horizontalSpacer_SpeedHint2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_SpeedHint.addItem(self.horizontalSpacer_SpeedHint2)

        self.lbl_SpeedMax = QLabel(self.grp_Gear_Ctrl)
        self.lbl_SpeedMax.setObjectName(u"lbl_SpeedMax")

        self.horizontalLayout_SpeedHint.addWidget(self.lbl_SpeedMax)


        self.verticalLayout_SpeedDial.addLayout(self.horizontalLayout_SpeedHint)

        self.spin_GearSpeed = QSpinBox(self.grp_Gear_Ctrl)
        self.spin_GearSpeed.setObjectName(u"spin_GearSpeed")
        self.spin_GearSpeed.setMinimumSize(QSize(110, 30))
        self.spin_GearSpeed.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_GearSpeed.setMaximum(15)

        self.verticalLayout_SpeedDial.addWidget(self.spin_GearSpeed)


        self.horizontalLayout_Dial.addLayout(self.verticalLayout_SpeedDial)

        self.line_2 = QFrame(self.grp_Gear_Ctrl)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setFrameShape(QFrame.Shape.VLine)
        self.line_2.setFrameShadow(QFrame.Shadow.Sunken)

        self.horizontalLayout_Dial.addWidget(self.line_2)

        self.verticalLayout_TurnDial = QVBoxLayout()
        self.verticalLayout_TurnDial.setObjectName(u"verticalLayout_TurnDial")
        self.lbl_TurnDial = QLabel(self.grp_Gear_Ctrl)
        self.lbl_TurnDial.setObjectName(u"lbl_TurnDial")
        self.lbl_TurnDial.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_TurnDial.addWidget(self.lbl_TurnDial)

        self.dial_Turn = QSlider(self.grp_Gear_Ctrl)
        self.dial_Turn.setObjectName(u"dial_Turn")
        self.dial_Turn.setMinimumSize(QSize(250, 34))
        self.dial_Turn.setMinimum(1)
        self.dial_Turn.setMaximum(15)
        self.dial_Turn.setValue(8)
        self.dial_Turn.setOrientation(Qt.Orientation.Horizontal)
        self.dial_Turn.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.dial_Turn.setTickInterval(1)

        self.verticalLayout_TurnDial.addWidget(self.dial_Turn)

        self.horizontalLayout_TurnHint = QHBoxLayout()
        self.horizontalLayout_TurnHint.setObjectName(u"horizontalLayout_TurnHint")
        self.lbl_TurnLeft = QLabel(self.grp_Gear_Ctrl)
        self.lbl_TurnLeft.setObjectName(u"lbl_TurnLeft")

        self.horizontalLayout_TurnHint.addWidget(self.lbl_TurnLeft)

        self.horizontalSpacer_TurnHint1 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_TurnHint.addItem(self.horizontalSpacer_TurnHint1)

        self.lbl_TurnMid = QLabel(self.grp_Gear_Ctrl)
        self.lbl_TurnMid.setObjectName(u"lbl_TurnMid")

        self.horizontalLayout_TurnHint.addWidget(self.lbl_TurnMid)

        self.horizontalSpacer_TurnHint2 = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_TurnHint.addItem(self.horizontalSpacer_TurnHint2)

        self.lbl_TurnRight = QLabel(self.grp_Gear_Ctrl)
        self.lbl_TurnRight.setObjectName(u"lbl_TurnRight")

        self.horizontalLayout_TurnHint.addWidget(self.lbl_TurnRight)


        self.verticalLayout_TurnDial.addLayout(self.horizontalLayout_TurnHint)

        self.spin_GearTurn = QSpinBox(self.grp_Gear_Ctrl)
        self.spin_GearTurn.setObjectName(u"spin_GearTurn")
        self.spin_GearTurn.setMinimumSize(QSize(110, 30))
        self.spin_GearTurn.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_GearTurn.setMinimum(1)
        self.spin_GearTurn.setMaximum(15)
        self.spin_GearTurn.setValue(8)

        self.verticalLayout_TurnDial.addWidget(self.spin_GearTurn)


        self.horizontalLayout_Dial.addLayout(self.verticalLayout_TurnDial)


        self.verticalLayout_Gear.addWidget(self.grp_Gear_Ctrl)

        self.frame_Gear_Act = QFrame(self.tab_SimpleMotion)
        self.frame_Gear_Act.setObjectName(u"frame_Gear_Act")
        self.frame_Gear_Act.setFrameShape(QFrame.Shape.StyledPanel)
        self.horizontalLayout_Gear_Act = QHBoxLayout(self.frame_Gear_Act)
        self.horizontalLayout_Gear_Act.setObjectName(u"horizontalLayout_Gear_Act")
        self.chk_Sync_Gear = QCheckBox(self.frame_Gear_Act)
        self.chk_Sync_Gear.setObjectName(u"chk_Sync_Gear")

        self.horizontalLayout_Gear_Act.addWidget(self.chk_Sync_Gear)

        self.spin_Sync_Gear_Ms = QSpinBox(self.frame_Gear_Act)
        self.spin_Sync_Gear_Ms.setObjectName(u"spin_Sync_Gear_Ms")
        self.spin_Sync_Gear_Ms.setMinimum(20)
        self.spin_Sync_Gear_Ms.setMaximum(1000)
        self.spin_Sync_Gear_Ms.setValue(40)

        self.horizontalLayout_Gear_Act.addWidget(self.spin_Sync_Gear_Ms)

        self.horizontalSpacer_Gear = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_Gear_Act.addItem(self.horizontalSpacer_Gear)

        self.btn_SendGear = QPushButton(self.frame_Gear_Act)
        self.btn_SendGear.setObjectName(u"btn_SendGear")
        self.btn_SendGear.setMinimumSize(QSize(120, 32))

        self.horizontalLayout_Gear_Act.addWidget(self.btn_SendGear)

        self.btn_SetBootFromGear = QPushButton(self.frame_Gear_Act)
        self.btn_SetBootFromGear.setObjectName(u"btn_SetBootFromGear")
        self.btn_SetBootFromGear.setMinimumSize(QSize(0, 32))

        self.horizontalLayout_Gear_Act.addWidget(self.btn_SetBootFromGear)


        self.verticalLayout_Gear.addWidget(self.frame_Gear_Act)

        self.verticalSpacer_Gear = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_Gear.addItem(self.verticalSpacer_Gear)

        self.tabWidget_Main.addTab(self.tab_SimpleMotion, "")
        self.tab_Servo = QWidget()
        self.tab_Servo.setObjectName(u"tab_Servo")
        self.horizontalLayout_Servo_Main = QHBoxLayout(self.tab_Servo)
        self.horizontalLayout_Servo_Main.setObjectName(u"horizontalLayout_Servo_Main")
        self.verticalLayout_Servo_Ctrl = QVBoxLayout()
        self.verticalLayout_Servo_Ctrl.setObjectName(u"verticalLayout_Servo_Ctrl")
        self.grp_Servo_Pos = QGroupBox(self.tab_Servo)
        self.grp_Servo_Pos.setObjectName(u"grp_Servo_Pos")
        self.verticalLayout_SPos = QVBoxLayout(self.grp_Servo_Pos)
        self.verticalLayout_SPos.setObjectName(u"verticalLayout_SPos")
        self.layout_ServoMode = QHBoxLayout()
        self.layout_ServoMode.setObjectName(u"layout_ServoMode")
        self.radio_S1_Only = QRadioButton(self.grp_Servo_Pos)
        self.radio_S1_Only.setObjectName(u"radio_S1_Only")
        self.radio_S1_Only.setChecked(False)

        self.layout_ServoMode.addWidget(self.radio_S1_Only)

        self.radio_S2_Only = QRadioButton(self.grp_Servo_Pos)
        self.radio_S2_Only.setObjectName(u"radio_S2_Only")

        self.layout_ServoMode.addWidget(self.radio_S2_Only)

        self.radio_Dual_Sync = QRadioButton(self.grp_Servo_Pos)
        self.radio_Dual_Sync.setObjectName(u"radio_Dual_Sync")
        self.radio_Dual_Sync.setChecked(True)

        self.layout_ServoMode.addWidget(self.radio_Dual_Sync)


        self.verticalLayout_SPos.addLayout(self.layout_ServoMode)

        self.frame_S1 = QFrame(self.grp_Servo_Pos)
        self.frame_S1.setObjectName(u"frame_S1")
        self.frame_S1.setFrameShape(QFrame.Shape.NoFrame)
        self.horizontalLayout_S1 = QHBoxLayout(self.frame_S1)
        self.horizontalLayout_S1.setObjectName(u"horizontalLayout_S1")
        self.lbl_S1 = QLabel(self.frame_S1)
        self.lbl_S1.setObjectName(u"lbl_S1")
        self.lbl_S1.setMinimumSize(QSize(30, 0))

        self.horizontalLayout_S1.addWidget(self.lbl_S1)

        self.slider_S1 = QSlider(self.frame_S1)
        self.slider_S1.setObjectName(u"slider_S1")
        self.slider_S1.setMinimum(-900)
        self.slider_S1.setMaximum(900)
        self.slider_S1.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_S1.addWidget(self.slider_S1)

        self.spin_S1 = QDoubleSpinBox(self.frame_S1)
        self.spin_S1.setObjectName(u"spin_S1")
        self.spin_S1.setMinimum(-90.000000000000000)
        self.spin_S1.setMaximum(90.000000000000000)

        self.horizontalLayout_S1.addWidget(self.spin_S1)


        self.verticalLayout_SPos.addWidget(self.frame_S1)

        self.frame_S2 = QFrame(self.grp_Servo_Pos)
        self.frame_S2.setObjectName(u"frame_S2")
        self.frame_S2.setFrameShape(QFrame.Shape.NoFrame)
        self.horizontalLayout_S2 = QHBoxLayout(self.frame_S2)
        self.horizontalLayout_S2.setObjectName(u"horizontalLayout_S2")
        self.lbl_S2 = QLabel(self.frame_S2)
        self.lbl_S2.setObjectName(u"lbl_S2")
        self.lbl_S2.setMinimumSize(QSize(30, 0))

        self.horizontalLayout_S2.addWidget(self.lbl_S2)

        self.slider_S2 = QSlider(self.frame_S2)
        self.slider_S2.setObjectName(u"slider_S2")
        self.slider_S2.setMinimum(-900)
        self.slider_S2.setMaximum(900)
        self.slider_S2.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_S2.addWidget(self.slider_S2)

        self.spin_S2 = QDoubleSpinBox(self.frame_S2)
        self.spin_S2.setObjectName(u"spin_S2")
        self.spin_S2.setMinimum(-90.000000000000000)
        self.spin_S2.setMaximum(90.000000000000000)

        self.horizontalLayout_S2.addWidget(self.spin_S2)


        self.verticalLayout_SPos.addWidget(self.frame_S2)

        self.frame_Servo_Act = QFrame(self.grp_Servo_Pos)
        self.frame_Servo_Act.setObjectName(u"frame_Servo_Act")
        self.frame_Servo_Act.setFrameShape(QFrame.Shape.NoFrame)
        self.horizontalLayout_Servo_Act = QHBoxLayout(self.frame_Servo_Act)
        self.horizontalLayout_Servo_Act.setObjectName(u"horizontalLayout_Servo_Act")
        self.chk_Sync_Servo = QCheckBox(self.frame_Servo_Act)
        self.chk_Sync_Servo.setObjectName(u"chk_Sync_Servo")

        self.horizontalLayout_Servo_Act.addWidget(self.chk_Sync_Servo)

        self.spin_Sync_Servo_Ms = QSpinBox(self.frame_Servo_Act)
        self.spin_Sync_Servo_Ms.setObjectName(u"spin_Sync_Servo_Ms")
        self.spin_Sync_Servo_Ms.setMinimum(20)
        self.spin_Sync_Servo_Ms.setMaximum(1000)
        self.spin_Sync_Servo_Ms.setValue(40)

        self.horizontalLayout_Servo_Act.addWidget(self.spin_Sync_Servo_Ms)

        self.horizontalSpacer_Servo = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_Servo_Act.addItem(self.horizontalSpacer_Servo)

        self.btn_SendServo = QPushButton(self.frame_Servo_Act)
        self.btn_SendServo.setObjectName(u"btn_SendServo")

        self.horizontalLayout_Servo_Act.addWidget(self.btn_SendServo)

        self.btn_ServoQuickReset = QPushButton(self.frame_Servo_Act)
        self.btn_ServoQuickReset.setObjectName(u"btn_ServoQuickReset")

        self.horizontalLayout_Servo_Act.addWidget(self.btn_ServoQuickReset)


        self.verticalLayout_SPos.addWidget(self.frame_Servo_Act)


        self.verticalLayout_Servo_Ctrl.addWidget(self.grp_Servo_Pos)

        self.grp_Servo_Power = QGroupBox(self.tab_Servo)
        self.grp_Servo_Power.setObjectName(u"grp_Servo_Power")
        self.horizontalLayout_SPower = QHBoxLayout(self.grp_Servo_Power)
        self.horizontalLayout_SPower.setObjectName(u"horizontalLayout_SPower")
        self.chk_S1_Pwr = QCheckBox(self.grp_Servo_Power)
        self.chk_S1_Pwr.setObjectName(u"chk_S1_Pwr")
        self.chk_S1_Pwr.setChecked(True)

        self.horizontalLayout_SPower.addWidget(self.chk_S1_Pwr)

        self.chk_S2_Pwr = QCheckBox(self.grp_Servo_Power)
        self.chk_S2_Pwr.setObjectName(u"chk_S2_Pwr")
        self.chk_S2_Pwr.setChecked(True)

        self.horizontalLayout_SPower.addWidget(self.chk_S2_Pwr)

        self.btn_SetServoPower = QPushButton(self.grp_Servo_Power)
        self.btn_SetServoPower.setObjectName(u"btn_SetServoPower")
        self.btn_SetServoPower.setMinimumSize(QSize(0, 32))

        self.horizontalLayout_SPower.addWidget(self.btn_SetServoPower)


        self.verticalLayout_Servo_Ctrl.addWidget(self.grp_Servo_Power)

        self.verticalSpacer_ServoLeft = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_Servo_Ctrl.addItem(self.verticalSpacer_ServoLeft)


        self.horizontalLayout_Servo_Main.addLayout(self.verticalLayout_Servo_Ctrl)

        self.grp_Servo_Status = QGroupBox(self.tab_Servo)
        self.grp_Servo_Status.setObjectName(u"grp_Servo_Status")
        self.grp_Servo_Status.setMinimumSize(QSize(280, 0))
        self.verticalLayout_ServoStat = QVBoxLayout(self.grp_Servo_Status)
        self.verticalLayout_ServoStat.setObjectName(u"verticalLayout_ServoStat")
        self.btn_QueryServoAll = QPushButton(self.grp_Servo_Status)
        self.btn_QueryServoAll.setObjectName(u"btn_QueryServoAll")

        self.verticalLayout_ServoStat.addWidget(self.btn_QueryServoAll)

        self.btn_ResetFault_ServoStat = QPushButton(self.grp_Servo_Status)
        self.btn_ResetFault_ServoStat.setObjectName(u"btn_ResetFault_ServoStat")

        self.verticalLayout_ServoStat.addWidget(self.btn_ResetFault_ServoStat)

        self.lbl_S1_FaultFlag = QLabel(self.grp_Servo_Status)
        self.lbl_S1_FaultFlag.setObjectName(u"lbl_S1_FaultFlag")

        self.verticalLayout_ServoStat.addWidget(self.lbl_S1_FaultFlag)

        self.lbl_S1_FaultFlag_Val = QLabel(self.grp_Servo_Status)
        self.lbl_S1_FaultFlag_Val.setObjectName(u"lbl_S1_FaultFlag_Val")
        self.lbl_S1_FaultFlag_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_S1_FaultFlag_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_S1_FaultFlag_Val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_ServoStat.addWidget(self.lbl_S1_FaultFlag_Val)

        self.lbl_S2_FaultFlag = QLabel(self.grp_Servo_Status)
        self.lbl_S2_FaultFlag.setObjectName(u"lbl_S2_FaultFlag")

        self.verticalLayout_ServoStat.addWidget(self.lbl_S2_FaultFlag)

        self.lbl_S2_FaultFlag_Val = QLabel(self.grp_Servo_Status)
        self.lbl_S2_FaultFlag_Val.setObjectName(u"lbl_S2_FaultFlag_Val")
        self.lbl_S2_FaultFlag_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_S2_FaultFlag_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_S2_FaultFlag_Val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_ServoStat.addWidget(self.lbl_S2_FaultFlag_Val)

        self.lbl_S1_Curr = QLabel(self.grp_Servo_Status)
        self.lbl_S1_Curr.setObjectName(u"lbl_S1_Curr")

        self.verticalLayout_ServoStat.addWidget(self.lbl_S1_Curr)

        self.bar_S1_Curr = QProgressBar(self.grp_Servo_Status)
        self.bar_S1_Curr.setObjectName(u"bar_S1_Curr")
        self.bar_S1_Curr.setMaximum(4000)
        self.bar_S1_Curr.setValue(1500)

        self.verticalLayout_ServoStat.addWidget(self.bar_S1_Curr)

        self.lbl_S2_Curr = QLabel(self.grp_Servo_Status)
        self.lbl_S2_Curr.setObjectName(u"lbl_S2_Curr")

        self.verticalLayout_ServoStat.addWidget(self.lbl_S2_Curr)

        self.bar_S2_Curr = QProgressBar(self.grp_Servo_Status)
        self.bar_S2_Curr.setObjectName(u"bar_S2_Curr")
        self.bar_S2_Curr.setMaximum(4000)
        self.bar_S2_Curr.setValue(2500)

        self.verticalLayout_ServoStat.addWidget(self.bar_S2_Curr)

        self.verticalSpacer_ServoRight = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_ServoStat.addItem(self.verticalSpacer_ServoRight)


        self.horizontalLayout_Servo_Main.addWidget(self.grp_Servo_Status)

        self.tabWidget_Main.addTab(self.tab_Servo, "")
        self.tab_AdvMotion = QWidget()
        self.tab_AdvMotion.setObjectName(u"tab_AdvMotion")
        self.verticalLayout_CPG = QVBoxLayout(self.tab_AdvMotion)
        self.verticalLayout_CPG.setObjectName(u"verticalLayout_CPG")
        self.grp_CPG_Param = QGroupBox(self.tab_AdvMotion)
        self.grp_CPG_Param.setObjectName(u"grp_CPG_Param")
        self.verticalLayout_5 = QVBoxLayout(self.grp_CPG_Param)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(9, 9, 9, 9)
        self.lbl_Amp = QLabel(self.grp_CPG_Param)
        self.lbl_Amp.setObjectName(u"lbl_Amp")

        self.horizontalLayout_4.addWidget(self.lbl_Amp)

        self.slider_Amp = QSlider(self.grp_CPG_Param)
        self.slider_Amp.setObjectName(u"slider_Amp")
        self.slider_Amp.setMaximum(250)
        self.slider_Amp.setValue(150)
        self.slider_Amp.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_4.addWidget(self.slider_Amp)

        self.spin_Amp = QDoubleSpinBox(self.grp_CPG_Param)
        self.spin_Amp.setObjectName(u"spin_Amp")
        self.spin_Amp.setDecimals(1)
        self.spin_Amp.setMaximum(25.000000000000000)
        self.spin_Amp.setSingleStep(0.100000000000000)
        self.spin_Amp.setValue(15.000000000000000)

        self.horizontalLayout_4.addWidget(self.spin_Amp)


        self.verticalLayout_5.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(9, 9, 9, 9)
        self.lbl_Freq = QLabel(self.grp_CPG_Param)
        self.lbl_Freq.setObjectName(u"lbl_Freq")

        self.horizontalLayout_5.addWidget(self.lbl_Freq)

        self.slider_Freq = QSlider(self.grp_CPG_Param)
        self.slider_Freq.setObjectName(u"slider_Freq")
        self.slider_Freq.setMaximum(38)
        self.slider_Freq.setValue(12)
        self.slider_Freq.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_5.addWidget(self.slider_Freq)

        self.spin_Freq = QDoubleSpinBox(self.grp_CPG_Param)
        self.spin_Freq.setObjectName(u"spin_Freq")
        self.spin_Freq.setDecimals(1)
        self.spin_Freq.setMaximum(3.800000000000000)
        self.spin_Freq.setSingleStep(0.100000000000000)
        self.spin_Freq.setValue(1.200000000000000)

        self.horizontalLayout_5.addWidget(self.spin_Freq)


        self.verticalLayout_5.addLayout(self.horizontalLayout_5)

        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.horizontalLayout_6.setContentsMargins(9, 9, 9, 9)
        self.lbl_Bias = QLabel(self.grp_CPG_Param)
        self.lbl_Bias.setObjectName(u"lbl_Bias")

        self.horizontalLayout_6.addWidget(self.lbl_Bias)

        self.slider_Bias = QSlider(self.grp_CPG_Param)
        self.slider_Bias.setObjectName(u"slider_Bias")
        self.slider_Bias.setMinimum(-250)
        self.slider_Bias.setMaximum(250)
        self.slider_Bias.setValue(-150)
        self.slider_Bias.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_6.addWidget(self.slider_Bias)

        self.spin_Bias = QDoubleSpinBox(self.grp_CPG_Param)
        self.spin_Bias.setObjectName(u"spin_Bias")
        self.spin_Bias.setDecimals(1)
        self.spin_Bias.setMinimum(-25.000000000000000)
        self.spin_Bias.setMaximum(25.000000000000000)
        self.spin_Bias.setValue(-15.000000000000000)

        self.horizontalLayout_6.addWidget(self.spin_Bias)


        self.verticalLayout_5.addLayout(self.horizontalLayout_6)


        self.verticalLayout_CPG.addWidget(self.grp_CPG_Param)

        self.grp_CPG_Inter = QGroupBox(self.tab_AdvMotion)
        self.grp_CPG_Inter.setObjectName(u"grp_CPG_Inter")
        self.grp_CPG_Inter.setCheckable(True)
        self.grp_CPG_Inter.setChecked(False)
        self.horizontalLayout_CPG_Inter = QHBoxLayout(self.grp_CPG_Inter)
        self.horizontalLayout_CPG_Inter.setObjectName(u"horizontalLayout_CPG_Inter")
        self.lbl_N = QLabel(self.grp_CPG_Inter)
        self.lbl_N.setObjectName(u"lbl_N")

        self.horizontalLayout_CPG_Inter.addWidget(self.lbl_N)

        self.spin_N = QSpinBox(self.grp_CPG_Inter)
        self.spin_N.setObjectName(u"spin_N")
        self.spin_N.setMaximum(15)

        self.horizontalLayout_CPG_Inter.addWidget(self.spin_N)

        self.lbl_Coast = QLabel(self.grp_CPG_Inter)
        self.lbl_Coast.setObjectName(u"lbl_Coast")

        self.horizontalLayout_CPG_Inter.addWidget(self.lbl_Coast)

        self.spin_Coast = QDoubleSpinBox(self.grp_CPG_Inter)
        self.spin_Coast.setObjectName(u"spin_Coast")
        self.spin_Coast.setDecimals(1)
        self.spin_Coast.setMaximum(400.000000000000000)
        self.spin_Coast.setSingleStep(0.100000000000000)
        self.spin_Coast.setValue(3.500000000000000)

        self.horizontalLayout_CPG_Inter.addWidget(self.spin_Coast)


        self.verticalLayout_CPG.addWidget(self.grp_CPG_Inter)

        self.frame_CPG_Action = QFrame(self.tab_AdvMotion)
        self.frame_CPG_Action.setObjectName(u"frame_CPG_Action")
        self.frame_CPG_Action.setFrameShape(QFrame.Shape.StyledPanel)
        self.horizontalLayout_CPG_Act = QHBoxLayout(self.frame_CPG_Action)
        self.horizontalLayout_CPG_Act.setObjectName(u"horizontalLayout_CPG_Act")
        self.chk_Sync_CPG = QCheckBox(self.frame_CPG_Action)
        self.chk_Sync_CPG.setObjectName(u"chk_Sync_CPG")

        self.horizontalLayout_CPG_Act.addWidget(self.chk_Sync_CPG)

        self.spin_Sync_CPG_Ms = QSpinBox(self.frame_CPG_Action)
        self.spin_Sync_CPG_Ms.setObjectName(u"spin_Sync_CPG_Ms")
        self.spin_Sync_CPG_Ms.setMinimum(20)
        self.spin_Sync_CPG_Ms.setMaximum(1000)
        self.spin_Sync_CPG_Ms.setValue(40)

        self.horizontalLayout_CPG_Act.addWidget(self.spin_Sync_CPG_Ms)

        self.horizontalSpacer_CPG = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_CPG_Act.addItem(self.horizontalSpacer_CPG)

        self.btn_SendCPG = QPushButton(self.frame_CPG_Action)
        self.btn_SendCPG.setObjectName(u"btn_SendCPG")
        self.btn_SendCPG.setMinimumSize(QSize(120, 32))

        self.horizontalLayout_CPG_Act.addWidget(self.btn_SendCPG)

        self.btn_SetBootFromCPG = QPushButton(self.frame_CPG_Action)
        self.btn_SetBootFromCPG.setObjectName(u"btn_SetBootFromCPG")
        self.btn_SetBootFromCPG.setMinimumSize(QSize(0, 32))

        self.horizontalLayout_CPG_Act.addWidget(self.btn_SetBootFromCPG)


        self.verticalLayout_CPG.addWidget(self.frame_CPG_Action)

        self.verticalSpacer_CPG = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_CPG.addItem(self.verticalSpacer_CPG)

        self.tabWidget_Main.addTab(self.tab_AdvMotion, "")
        self.tab_Play = QWidget()
        self.tab_Play.setObjectName(u"tab_Play")
        self.verticalLayout_Play = QVBoxLayout(self.tab_Play)
        self.verticalLayout_Play.setObjectName(u"verticalLayout_Play")
        self.horizontalLayout_Play = QHBoxLayout()
        self.horizontalLayout_Play.setObjectName(u"horizontalLayout_Play")
        self.lbl_PlayId = QLabel(self.tab_Play)
        self.lbl_PlayId.setObjectName(u"lbl_PlayId")

        self.horizontalLayout_Play.addWidget(self.lbl_PlayId)

        self.spin_PlayId = QSpinBox(self.tab_Play)
        self.spin_PlayId.setObjectName(u"spin_PlayId")
        self.spin_PlayId.setMinimum(1)
        self.spin_PlayId.setMaximum(255)

        self.horizontalLayout_Play.addWidget(self.spin_PlayId)

        self.btn_PlayStart = QPushButton(self.tab_Play)
        self.btn_PlayStart.setObjectName(u"btn_PlayStart")

        self.horizontalLayout_Play.addWidget(self.btn_PlayStart)

        self.btn_PlayStop = QPushButton(self.tab_Play)
        self.btn_PlayStop.setObjectName(u"btn_PlayStop")

        self.horizontalLayout_Play.addWidget(self.btn_PlayStop)

        self.btn_SetBootFromPlay = QPushButton(self.tab_Play)
        self.btn_SetBootFromPlay.setObjectName(u"btn_SetBootFromPlay")

        self.horizontalLayout_Play.addWidget(self.btn_SetBootFromPlay)

        self.horizontalSpacer_Play = QSpacerItem(40, 20, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)

        self.horizontalLayout_Play.addItem(self.horizontalSpacer_Play)


        self.verticalLayout_Play.addLayout(self.horizontalLayout_Play)

        self.table_PlayDesc = QTableWidget(self.tab_Play)
        if (self.table_PlayDesc.columnCount() < 2):
            self.table_PlayDesc.setColumnCount(2)
        __qtablewidgetitem7 = QTableWidgetItem()
        self.table_PlayDesc.setHorizontalHeaderItem(0, __qtablewidgetitem7)
        __qtablewidgetitem8 = QTableWidgetItem()
        self.table_PlayDesc.setHorizontalHeaderItem(1, __qtablewidgetitem8)
        if (self.table_PlayDesc.rowCount() < 3):
            self.table_PlayDesc.setRowCount(3)
        __qtablewidgetitem9 = QTableWidgetItem()
        __qtablewidgetitem9.setTextAlignment(Qt.AlignCenter);
        self.table_PlayDesc.setItem(0, 0, __qtablewidgetitem9)
        __qtablewidgetitem10 = QTableWidgetItem()
        self.table_PlayDesc.setItem(0, 1, __qtablewidgetitem10)
        __qtablewidgetitem11 = QTableWidgetItem()
        __qtablewidgetitem11.setTextAlignment(Qt.AlignCenter);
        self.table_PlayDesc.setItem(1, 0, __qtablewidgetitem11)
        __qtablewidgetitem12 = QTableWidgetItem()
        self.table_PlayDesc.setItem(1, 1, __qtablewidgetitem12)
        __qtablewidgetitem13 = QTableWidgetItem()
        __qtablewidgetitem13.setTextAlignment(Qt.AlignCenter);
        self.table_PlayDesc.setItem(2, 0, __qtablewidgetitem13)
        __qtablewidgetitem14 = QTableWidgetItem()
        self.table_PlayDesc.setItem(2, 1, __qtablewidgetitem14)
        self.table_PlayDesc.setObjectName(u"table_PlayDesc")
        self.table_PlayDesc.setAlternatingRowColors(True)
        self.table_PlayDesc.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_PlayDesc.setColumnCount(2)
        self.table_PlayDesc.horizontalHeader().setStretchLastSection(True)
        self.table_PlayDesc.verticalHeader().setVisible(False)

        self.verticalLayout_Play.addWidget(self.table_PlayDesc)

        self.tabWidget_Main.addTab(self.tab_Play, "")
        self.tab_BootConfig = QWidget()
        self.tab_BootConfig.setObjectName(u"tab_BootConfig")
        self.horizontalLayout_Boot = QHBoxLayout(self.tab_BootConfig)
        self.horizontalLayout_Boot.setSpacing(10)
        self.horizontalLayout_Boot.setObjectName(u"horizontalLayout_Boot")
        self.verticalLayout_BootLeft = QVBoxLayout()
        self.verticalLayout_BootLeft.setObjectName(u"verticalLayout_BootLeft")
        self.grp_EditFishID = QGroupBox(self.tab_BootConfig)
        self.grp_EditFishID.setObjectName(u"grp_EditFishID")
        self.formLayout_EditFishID = QVBoxLayout(self.grp_EditFishID)
        self.formLayout_EditFishID.setObjectName(u"formLayout_EditFishID")
        self.horizontalLayout_EditFishID = QHBoxLayout()
        self.horizontalLayout_EditFishID.setSpacing(6)
        self.horizontalLayout_EditFishID.setObjectName(u"horizontalLayout_EditFishID")
        self.label_EditFishID = QLabel(self.grp_EditFishID)
        self.label_EditFishID.setObjectName(u"label_EditFishID")
        self.label_EditFishID.setMinimumSize(QSize(0, 30))

        self.horizontalLayout_EditFishID.addWidget(self.label_EditFishID)

        self.txt_F_ID = QLineEdit(self.grp_EditFishID)
        self.txt_F_ID.setObjectName(u"txt_F_ID")
        self.txt_F_ID.setMinimumSize(QSize(90, 30))
        self.txt_F_ID.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_EditFishID.addWidget(self.txt_F_ID)

        self.btn_EditFishID = QPushButton(self.grp_EditFishID)
        self.btn_EditFishID.setObjectName(u"btn_EditFishID")

        self.horizontalLayout_EditFishID.addWidget(self.btn_EditFishID)


        self.formLayout_EditFishID.addLayout(self.horizontalLayout_EditFishID)

        self.horizontalLayout_EditCh = QHBoxLayout()
        self.horizontalLayout_EditCh.setSpacing(6)
        self.horizontalLayout_EditCh.setObjectName(u"horizontalLayout_EditCh")
        self.label_EditCh = QLabel(self.grp_EditFishID)
        self.label_EditCh.setObjectName(u"label_EditCh")
        self.label_EditCh.setMinimumSize(QSize(0, 30))

        self.horizontalLayout_EditCh.addWidget(self.label_EditCh)

        self.spin_F_Ch = QSpinBox(self.grp_EditFishID)
        self.spin_F_Ch.setObjectName(u"spin_F_Ch")
        sizePolicy3.setHeightForWidth(self.spin_F_Ch.sizePolicy().hasHeightForWidth())
        self.spin_F_Ch.setSizePolicy(sizePolicy3)
        self.spin_F_Ch.setMinimumSize(QSize(90, 30))
        self.spin_F_Ch.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.spin_F_Ch.setButtonSymbols(QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.spin_F_Ch.setMaximum(83)
        self.spin_F_Ch.setValue(23)

        self.horizontalLayout_EditCh.addWidget(self.spin_F_Ch)

        self.btn_EditFishCh = QPushButton(self.grp_EditFishID)
        self.btn_EditFishCh.setObjectName(u"btn_EditFishCh")

        self.horizontalLayout_EditCh.addWidget(self.btn_EditFishCh)


        self.formLayout_EditFishID.addLayout(self.horizontalLayout_EditCh)

        self.horizontalLayout_EditPwd = QHBoxLayout()
        self.horizontalLayout_EditPwd.setSpacing(6)
        self.horizontalLayout_EditPwd.setObjectName(u"horizontalLayout_EditPwd")
        self.label_EditPwd = QLabel(self.grp_EditFishID)
        self.label_EditPwd.setObjectName(u"label_EditPwd")
        self.label_EditPwd.setMinimumSize(QSize(0, 30))

        self.horizontalLayout_EditPwd.addWidget(self.label_EditPwd)

        self.txt_F_Pwd = QLineEdit(self.grp_EditFishID)
        self.txt_F_Pwd.setObjectName(u"txt_F_Pwd")
        self.txt_F_Pwd.setMinimumSize(QSize(90, 30))
        self.txt_F_Pwd.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.horizontalLayout_EditPwd.addWidget(self.txt_F_Pwd)

        self.btn_EditFishPwd = QPushButton(self.grp_EditFishID)
        self.btn_EditFishPwd.setObjectName(u"btn_EditFishPwd")

        self.horizontalLayout_EditPwd.addWidget(self.btn_EditFishPwd)


        self.formLayout_EditFishID.addLayout(self.horizontalLayout_EditPwd)


        self.verticalLayout_BootLeft.addWidget(self.grp_EditFishID)

        self.verticalSpacer_BootLeft = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_BootLeft.addItem(self.verticalSpacer_BootLeft)


        self.horizontalLayout_Boot.addLayout(self.verticalLayout_BootLeft)

        self.verticalLayout_BootRight = QVBoxLayout()
        self.verticalLayout_BootRight.setObjectName(u"verticalLayout_BootRight")
        self.grp_Startup_Basic = QGroupBox(self.tab_BootConfig)
        self.grp_Startup_Basic.setObjectName(u"grp_Startup_Basic")
        sizePolicy4 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy4.setHorizontalStretch(0)
        sizePolicy4.setVerticalStretch(0)
        sizePolicy4.setHeightForWidth(self.grp_Startup_Basic.sizePolicy().hasHeightForWidth())
        self.grp_Startup_Basic.setSizePolicy(sizePolicy4)
        self.horizontalLayout_3 = QHBoxLayout(self.grp_Startup_Basic)
        self.horizontalLayout_3.setSpacing(6)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(9, -1, -1, -1)
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setHorizontalSpacing(6)
        self.gridLayout.setVerticalSpacing(2)
        self.label_BFirmwareVer = QLabel(self.grp_Startup_Basic)
        self.label_BFirmwareVer.setObjectName(u"label_BFirmwareVer")
        self.label_BFirmwareVer.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BFirmwareVer, 6, 0, 1, 1)

        self.spin_F_ServoBiasS1 = QDoubleSpinBox(self.grp_Startup_Basic)
        self.spin_F_ServoBiasS1.setObjectName(u"spin_F_ServoBiasS1")
        self.spin_F_ServoBiasS1.setMinimumSize(QSize(0, 30))
        self.spin_F_ServoBiasS1.setDecimals(1)
        self.spin_F_ServoBiasS1.setMinimum(-90.000000000000000)
        self.spin_F_ServoBiasS1.setMaximum(90.000000000000000)
        self.spin_F_ServoBiasS1.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.spin_F_ServoBiasS1, 2, 1, 1, 1)

        self.lbl_BProtocolVer_Val = QLabel(self.grp_Startup_Basic)
        self.lbl_BProtocolVer_Val.setObjectName(u"lbl_BProtocolVer_Val")
        self.lbl_BProtocolVer_Val.setMinimumSize(QSize(0, 30))
        self.lbl_BProtocolVer_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_BProtocolVer_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout.addWidget(self.lbl_BProtocolVer_Val, 5, 1, 1, 2)

        self.label_BProtocolVer = QLabel(self.grp_Startup_Basic)
        self.label_BProtocolVer.setObjectName(u"label_BProtocolVer")
        self.label_BProtocolVer.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BProtocolVer, 5, 0, 1, 1)

        self.label_BServoBiasS1 = QLabel(self.grp_Startup_Basic)
        self.label_BServoBiasS1.setObjectName(u"label_BServoBiasS1")
        self.label_BServoBiasS1.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BServoBiasS1, 2, 0, 1, 1)

        self.label_BOverCurrentTimeout = QLabel(self.grp_Startup_Basic)
        self.label_BOverCurrentTimeout.setObjectName(u"label_BOverCurrentTimeout")
        self.label_BOverCurrentTimeout.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BOverCurrentTimeout, 4, 0, 1, 1)

        self.lbl_BFirmwareVer_Val = QLabel(self.grp_Startup_Basic)
        self.lbl_BFirmwareVer_Val.setObjectName(u"lbl_BFirmwareVer_Val")
        self.lbl_BFirmwareVer_Val.setMinimumSize(QSize(0, 30))
        self.lbl_BFirmwareVer_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_BFirmwareVer_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout.addWidget(self.lbl_BFirmwareVer_Val, 6, 1, 1, 2)

        self.spin_F_OverCurrentTimeout = QSpinBox(self.grp_Startup_Basic)
        self.spin_F_OverCurrentTimeout.setObjectName(u"spin_F_OverCurrentTimeout")
        self.spin_F_OverCurrentTimeout.setMinimumSize(QSize(0, 30))
        self.spin_F_OverCurrentTimeout.setMaximum(15)
        self.spin_F_OverCurrentTimeout.setValue(2)

        self.gridLayout.addWidget(self.spin_F_OverCurrentTimeout, 4, 1, 1, 1)

        self.combo_F_Mode = QComboBox(self.grp_Startup_Basic)
        self.combo_F_Mode.addItem("")
        self.combo_F_Mode.addItem("")
        self.combo_F_Mode.addItem("")
        self.combo_F_Mode.addItem("")
        self.combo_F_Mode.addItem("")
        self.combo_F_Mode.addItem("")
        self.combo_F_Mode.setObjectName(u"combo_F_Mode")
        self.combo_F_Mode.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.combo_F_Mode, 0, 1, 1, 2)

        self.label_Warn_2 = QLabel(self.grp_Startup_Basic)
        self.label_Warn_2.setObjectName(u"label_Warn_2")
        self.label_Warn_2.setFont(font1)

        self.gridLayout.addWidget(self.label_Warn_2, 1, 2, 1, 1)

        self.label_BMode = QLabel(self.grp_Startup_Basic)
        self.label_BMode.setObjectName(u"label_BMode")
        self.label_BMode.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BMode, 0, 0, 1, 1)

        self.label_BReply = QLabel(self.grp_Startup_Basic)
        self.label_BReply.setObjectName(u"label_BReply")
        self.label_BReply.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BReply, 1, 0, 1, 1)

        self.chk_F_Reply = QCheckBox(self.grp_Startup_Basic)
        self.chk_F_Reply.setObjectName(u"chk_F_Reply")
        self.chk_F_Reply.setChecked(True)

        self.gridLayout.addWidget(self.chk_F_Reply, 1, 1, 1, 1)

        self.label_BServoBiasS2 = QLabel(self.grp_Startup_Basic)
        self.label_BServoBiasS2.setObjectName(u"label_BServoBiasS2")
        self.label_BServoBiasS2.setMinimumSize(QSize(0, 30))

        self.gridLayout.addWidget(self.label_BServoBiasS2, 3, 0, 1, 1)

        self.spin_F_ServoBiasS2 = QDoubleSpinBox(self.grp_Startup_Basic)
        self.spin_F_ServoBiasS2.setObjectName(u"spin_F_ServoBiasS2")
        self.spin_F_ServoBiasS2.setMinimumSize(QSize(0, 30))
        self.spin_F_ServoBiasS2.setDecimals(1)
        self.spin_F_ServoBiasS2.setMinimum(-90.000000000000000)
        self.spin_F_ServoBiasS2.setMaximum(90.000000000000000)
        self.spin_F_ServoBiasS2.setSingleStep(0.100000000000000)

        self.gridLayout.addWidget(self.spin_F_ServoBiasS2, 3, 1, 1, 1)

        self.label_2 = QLabel(self.grp_Startup_Basic)
        self.label_2.setObjectName(u"label_2")
        palette = QPalette()
        brush = QBrush(QColor(143, 143, 143, 228))
        brush.setStyle(Qt.BrushStyle.SolidPattern)
        palette.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, brush)
        palette.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, brush)
        self.label_2.setPalette(palette)

        self.gridLayout.addWidget(self.label_2, 2, 2, 1, 1)

        self.label_3 = QLabel(self.grp_Startup_Basic)
        self.label_3.setObjectName(u"label_3")
        palette1 = QPalette()
        palette1.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, brush)
        palette1.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, brush)
        self.label_3.setPalette(palette1)

        self.gridLayout.addWidget(self.label_3, 3, 2, 1, 1)

        self.label_4 = QLabel(self.grp_Startup_Basic)
        self.label_4.setObjectName(u"label_4")
        palette2 = QPalette()
        palette2.setBrush(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, brush)
        palette2.setBrush(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, brush)
        self.label_4.setPalette(palette2)

        self.gridLayout.addWidget(self.label_4, 4, 2, 1, 1)


        self.horizontalLayout_3.addLayout(self.gridLayout)


        self.verticalLayout_BootRight.addWidget(self.grp_Startup_Basic)

        self.layout_BootActions = QHBoxLayout()
        self.layout_BootActions.setObjectName(u"layout_BootActions")
        self.btn_FlashRead = QPushButton(self.tab_BootConfig)
        self.btn_FlashRead.setObjectName(u"btn_FlashRead")
        self.btn_FlashRead.setMinimumSize(QSize(0, 32))

        self.layout_BootActions.addWidget(self.btn_FlashRead)

        self.btn_FlashSave = QPushButton(self.tab_BootConfig)
        self.btn_FlashSave.setObjectName(u"btn_FlashSave")
        self.btn_FlashSave.setMinimumSize(QSize(0, 32))

        self.layout_BootActions.addWidget(self.btn_FlashSave)


        self.verticalLayout_BootRight.addLayout(self.layout_BootActions)

        self.verticalSpacer_BootRight = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_BootRight.addItem(self.verticalSpacer_BootRight)


        self.horizontalLayout_Boot.addLayout(self.verticalLayout_BootRight)

        self.tabWidget_Main.addTab(self.tab_BootConfig, "")
        self.tab_Config = QWidget()
        self.tab_Config.setObjectName(u"tab_Config")
        self.gridLayout_Cfg = QGridLayout(self.tab_Config)
        self.gridLayout_Cfg.setObjectName(u"gridLayout_Cfg")
        self.grp_Diag = QGroupBox(self.tab_Config)
        self.grp_Diag.setObjectName(u"grp_Diag")
        self.verticalLayout_4 = QVBoxLayout(self.grp_Diag)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.btn_ResetFault_6 = QPushButton(self.grp_Diag)
        self.btn_ResetFault_6.setObjectName(u"btn_ResetFault_6")
        self.btn_ResetFault_6.setMinimumSize(QSize(147, 32))

        self.verticalLayout_3.addWidget(self.btn_ResetFault_6)

        self.btn_ResetFault = QPushButton(self.grp_Diag)
        self.btn_ResetFault.setObjectName(u"btn_ResetFault")
        self.btn_ResetFault.setMinimumSize(QSize(147, 32))

        self.verticalLayout_3.addWidget(self.btn_ResetFault)

        self.btn_ResetFault_5 = QPushButton(self.grp_Diag)
        self.btn_ResetFault_5.setObjectName(u"btn_ResetFault_5")
        self.btn_ResetFault_5.setMinimumSize(QSize(147, 32))

        self.verticalLayout_3.addWidget(self.btn_ResetFault_5)

        self.btn_FactoryReset = QPushButton(self.grp_Diag)
        self.btn_FactoryReset.setObjectName(u"btn_FactoryReset")
        self.btn_FactoryReset.setMinimumSize(QSize(147, 32))

        self.verticalLayout_3.addWidget(self.btn_FactoryReset)


        self.verticalLayout_4.addLayout(self.verticalLayout_3)


        self.gridLayout_Cfg.addWidget(self.grp_Diag, 0, 1, 1, 1)

        self.grp_AutoReport = QGroupBox(self.tab_Config)
        self.grp_AutoReport.setObjectName(u"grp_AutoReport")
        self.verticalLayout_AutoReport = QVBoxLayout(self.grp_AutoReport)
        self.verticalLayout_AutoReport.setSpacing(10)
        self.verticalLayout_AutoReport.setObjectName(u"verticalLayout_AutoReport")
        self.lbl_ReportItems = QLabel(self.grp_AutoReport)
        self.lbl_ReportItems.setObjectName(u"lbl_ReportItems")

        self.verticalLayout_AutoReport.addWidget(self.lbl_ReportItems)

        self.gridLayout_ReportFlags = QGridLayout()
        self.gridLayout_ReportFlags.setObjectName(u"gridLayout_ReportFlags")
        self.gridLayout_ReportFlags.setHorizontalSpacing(18)
        self.gridLayout_ReportFlags.setVerticalSpacing(8)
        self.chk_Rpt_Servo = QCheckBox(self.grp_AutoReport)
        self.chk_Rpt_Servo.setObjectName(u"chk_Rpt_Servo")

        self.gridLayout_ReportFlags.addWidget(self.chk_Rpt_Servo, 1, 0, 1, 1)

        self.chk_Rpt_Motion = QCheckBox(self.grp_AutoReport)
        self.chk_Rpt_Motion.setObjectName(u"chk_Rpt_Motion")

        self.gridLayout_ReportFlags.addWidget(self.chk_Rpt_Motion, 0, 0, 1, 1)


        self.verticalLayout_AutoReport.addLayout(self.gridLayout_ReportFlags)

        self.line_5 = QFrame(self.grp_AutoReport)
        self.line_5.setObjectName(u"line_5")
        self.line_5.setFrameShape(QFrame.Shape.HLine)
        self.line_5.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_AutoReport.addWidget(self.line_5)

        self.layout_ReportTime = QHBoxLayout()
        self.layout_ReportTime.setSpacing(10)
        self.layout_ReportTime.setObjectName(u"layout_ReportTime")
        self.lbl_Ms = QLabel(self.grp_AutoReport)
        self.lbl_Ms.setObjectName(u"lbl_Ms")

        self.layout_ReportTime.addWidget(self.lbl_Ms)

        self.spin_Rpt_Ms = QSpinBox(self.grp_AutoReport)
        self.spin_Rpt_Ms.setObjectName(u"spin_Rpt_Ms")
        self.spin_Rpt_Ms.setMinimum(100)
        self.spin_Rpt_Ms.setMaximum(3600000)
        self.spin_Rpt_Ms.setSingleStep(100)
        self.spin_Rpt_Ms.setValue(1000)

        self.layout_ReportTime.addWidget(self.spin_Rpt_Ms)


        self.verticalLayout_AutoReport.addLayout(self.layout_ReportTime)

        self.layout_ReportAction = QHBoxLayout()
        self.layout_ReportAction.setSpacing(12)
        self.layout_ReportAction.setObjectName(u"layout_ReportAction")
        self.radio_Rpt_Off = QRadioButton(self.grp_AutoReport)
        self.radio_Rpt_Off.setObjectName(u"radio_Rpt_Off")
        self.radio_Rpt_Off.setChecked(True)

        self.layout_ReportAction.addWidget(self.radio_Rpt_Off)

        self.radio_Rpt_On = QRadioButton(self.grp_AutoReport)
        self.radio_Rpt_On.setObjectName(u"radio_Rpt_On")

        self.layout_ReportAction.addWidget(self.radio_Rpt_On)

        self.btn_SetAutoReport = QPushButton(self.grp_AutoReport)
        self.btn_SetAutoReport.setObjectName(u"btn_SetAutoReport")

        self.layout_ReportAction.addWidget(self.btn_SetAutoReport)


        self.verticalLayout_AutoReport.addLayout(self.layout_ReportAction)


        self.gridLayout_Cfg.addWidget(self.grp_AutoReport, 0, 0, 1, 1)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout_Cfg.addItem(self.verticalSpacer, 1, 0, 1, 1)

        self.verticalSpacer_ConfigRight = QSpacerItem(14, 148, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.gridLayout_Cfg.addItem(self.verticalSpacer_ConfigRight, 1, 1, 1, 1)

        self.tabWidget_Main.addTab(self.tab_Config, "")
        self.tab_3DMotion = QWidget()
        self.tab_3DMotion.setObjectName(u"tab_3DMotion")
        self.verticalLayout_3DMotion = QVBoxLayout(self.tab_3DMotion)
        self.verticalLayout_3DMotion.setObjectName(u"verticalLayout_3DMotion")
        self.horizontalLayout_3DTop = QHBoxLayout()
        self.horizontalLayout_3DTop.setObjectName(u"horizontalLayout_3DTop")
        self.grp_IMU6Axis = QGroupBox(self.tab_3DMotion)
        self.grp_IMU6Axis.setObjectName(u"grp_IMU6Axis")
        self.gridLayout_IMU6 = QGridLayout(self.grp_IMU6Axis)
        self.gridLayout_IMU6.setObjectName(u"gridLayout_IMU6")
        self.lbl_Gyro_Status_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_Gyro_Status_Val.setObjectName(u"lbl_Gyro_Status_Val")
        self.lbl_Gyro_Status_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_Gyro_Status_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_Gyro_Status_Val, 0, 1, 1, 1)

        self.lbl_IMU_GZ_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_GZ_Val.setObjectName(u"lbl_IMU_GZ_Val")
        self.lbl_IMU_GZ_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_IMU_GZ_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_GZ_Val, 3, 3, 1, 1)

        self.lbl_IMU_GX_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_GX_Val.setObjectName(u"lbl_IMU_GX_Val")
        self.lbl_IMU_GX_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_IMU_GX_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_GX_Val, 1, 3, 1, 1)

        self.lbl_IMU_AX_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_AX_Val.setObjectName(u"lbl_IMU_AX_Val")
        self.lbl_IMU_AX_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_IMU_AX_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_AX_Val, 1, 1, 1, 1)

        self.lbl_IMU_AZ_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_AZ_Val.setObjectName(u"lbl_IMU_AZ_Val")
        self.lbl_IMU_AZ_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_IMU_AZ_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_AZ_Val, 3, 1, 1, 1)

        self.lbl_IMU_AX = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_AX.setObjectName(u"lbl_IMU_AX")

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_AX, 1, 0, 1, 1)

        self.lbl_IMU_AY = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_AY.setObjectName(u"lbl_IMU_AY")

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_AY, 2, 0, 1, 1)

        self.lbl_IMU_GY_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_GY_Val.setObjectName(u"lbl_IMU_GY_Val")
        self.lbl_IMU_GY_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_IMU_GY_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_GY_Val, 2, 3, 1, 1)

        self.lbl_IMU_GZ = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_GZ.setObjectName(u"lbl_IMU_GZ")

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_GZ, 3, 2, 1, 1)

        self.pushButton_2 = QPushButton(self.grp_IMU6Axis)
        self.pushButton_2.setObjectName(u"pushButton_2")

        self.gridLayout_IMU6.addWidget(self.pushButton_2, 0, 2, 1, 1)

        self.pushButton_3 = QPushButton(self.grp_IMU6Axis)
        self.pushButton_3.setObjectName(u"pushButton_3")

        self.gridLayout_IMU6.addWidget(self.pushButton_3, 0, 3, 1, 1)

        self.lbl_Gyro_Status = QLabel(self.grp_IMU6Axis)
        self.lbl_Gyro_Status.setObjectName(u"lbl_Gyro_Status")

        self.gridLayout_IMU6.addWidget(self.lbl_Gyro_Status, 0, 0, 1, 1)

        self.lbl_IMU_GY = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_GY.setObjectName(u"lbl_IMU_GY")

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_GY, 2, 2, 1, 1)

        self.lbl_IMU_AZ = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_AZ.setObjectName(u"lbl_IMU_AZ")

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_AZ, 3, 0, 1, 1)

        self.lbl_IMU_GX = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_GX.setObjectName(u"lbl_IMU_GX")

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_GX, 1, 2, 1, 1)

        self.lbl_IMU_AY_Val = QLabel(self.grp_IMU6Axis)
        self.lbl_IMU_AY_Val.setObjectName(u"lbl_IMU_AY_Val")
        self.lbl_IMU_AY_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_IMU_AY_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_IMU6.addWidget(self.lbl_IMU_AY_Val, 2, 1, 1, 1)


        self.horizontalLayout_3DTop.addWidget(self.grp_IMU6Axis)

        self.grp_YawFiltered = QGroupBox(self.tab_3DMotion)
        self.grp_YawFiltered.setObjectName(u"grp_YawFiltered")
        self.grp_YawFiltered.setMinimumSize(QSize(220, 0))
        self.gridLayout_2 = QGridLayout(self.grp_YawFiltered)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.lbl_Attitude_Roll = QLabel(self.grp_YawFiltered)
        self.lbl_Attitude_Roll.setObjectName(u"lbl_Attitude_Roll")

        self.gridLayout_2.addWidget(self.lbl_Attitude_Roll, 1, 2, 1, 1)

        self.lbl_Attitude_Pitch_Val = QLabel(self.grp_YawFiltered)
        self.lbl_Attitude_Pitch_Val.setObjectName(u"lbl_Attitude_Pitch_Val")
        self.lbl_Attitude_Pitch_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_Attitude_Pitch_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_Attitude_Pitch_Val, 1, 1, 1, 1)

        self.lbl_YawFiltered_Hint = QLabel(self.grp_YawFiltered)
        self.lbl_YawFiltered_Hint.setObjectName(u"lbl_YawFiltered_Hint")

        self.gridLayout_2.addWidget(self.lbl_YawFiltered_Hint, 6, 2, 1, 1)

        self.lbl_LinearAccel = QLabel(self.grp_YawFiltered)
        self.lbl_LinearAccel.setObjectName(u"lbl_LinearAccel")

        self.gridLayout_2.addWidget(self.lbl_LinearAccel, 6, 0, 1, 1)

        self.lbl_DepthInfo_Val = QLabel(self.grp_YawFiltered)
        self.lbl_DepthInfo_Val.setObjectName(u"lbl_DepthInfo_Val")
        self.lbl_DepthInfo_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_DepthInfo_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_DepthInfo_Val, 9, 3, 1, 1)

        self.lbl_Attitude_Pitch = QLabel(self.grp_YawFiltered)
        self.lbl_Attitude_Pitch.setObjectName(u"lbl_Attitude_Pitch")

        self.gridLayout_2.addWidget(self.lbl_Attitude_Pitch, 1, 0, 1, 1)

        self.lbl_DepthInfo = QLabel(self.grp_YawFiltered)
        self.lbl_DepthInfo.setObjectName(u"lbl_DepthInfo")

        self.gridLayout_2.addWidget(self.lbl_DepthInfo, 9, 2, 1, 1)

        self.lbl_RealtimeFreq = QLabel(self.grp_YawFiltered)
        self.lbl_RealtimeFreq.setObjectName(u"lbl_RealtimeFreq")

        self.gridLayout_2.addWidget(self.lbl_RealtimeFreq, 5, 2, 1, 1)

        self.lbl_LinearAccel_Val = QLabel(self.grp_YawFiltered)
        self.lbl_LinearAccel_Val.setObjectName(u"lbl_LinearAccel_Val")
        self.lbl_LinearAccel_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_LinearAccel_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_LinearAccel_Val, 6, 1, 1, 1)

        self.lbl_YawFiltered_Val = QLabel(self.grp_YawFiltered)
        self.lbl_YawFiltered_Val.setObjectName(u"lbl_YawFiltered_Val")
        font2 = QFont()
        font2.setFamilies([u"Microsoft YaHei UI"])
        font2.setPointSize(10)
        font2.setBold(False)
        self.lbl_YawFiltered_Val.setFont(font2)
        self.lbl_YawFiltered_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_YawFiltered_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_YawFiltered_Val.setAlignment(Qt.AlignmentFlag.AlignLeading|Qt.AlignmentFlag.AlignLeft|Qt.AlignmentFlag.AlignVCenter)

        self.gridLayout_2.addWidget(self.lbl_YawFiltered_Val, 6, 3, 1, 1)

        self.lbl_DepthSensor_Status = QLabel(self.grp_YawFiltered)
        self.lbl_DepthSensor_Status.setObjectName(u"lbl_DepthSensor_Status")

        self.gridLayout_2.addWidget(self.lbl_DepthSensor_Status, 9, 0, 1, 1)

        self.lbl_RealtimeFreq_Val = QLabel(self.grp_YawFiltered)
        self.lbl_RealtimeFreq_Val.setObjectName(u"lbl_RealtimeFreq_Val")
        self.lbl_RealtimeFreq_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_RealtimeFreq_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_RealtimeFreq_Val, 5, 3, 1, 1)

        self.lbl_Attitude_Yaw = QLabel(self.grp_YawFiltered)
        self.lbl_Attitude_Yaw.setObjectName(u"lbl_Attitude_Yaw")

        self.gridLayout_2.addWidget(self.lbl_Attitude_Yaw, 5, 0, 1, 1)

        self.lbl_DepthSensor_Status_Val = QLabel(self.grp_YawFiltered)
        self.lbl_DepthSensor_Status_Val.setObjectName(u"lbl_DepthSensor_Status_Val")
        self.lbl_DepthSensor_Status_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_DepthSensor_Status_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_DepthSensor_Status_Val, 9, 1, 1, 1)

        self.lbl_Attitude_Roll_Val = QLabel(self.grp_YawFiltered)
        self.lbl_Attitude_Roll_Val.setObjectName(u"lbl_Attitude_Roll_Val")
        self.lbl_Attitude_Roll_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_Attitude_Roll_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_Attitude_Roll_Val, 1, 3, 1, 1)

        self.lbl_Attitude_Yaw_Val = QLabel(self.grp_YawFiltered)
        self.lbl_Attitude_Yaw_Val.setObjectName(u"lbl_Attitude_Yaw_Val")
        self.lbl_Attitude_Yaw_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_Attitude_Yaw_Val.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.lbl_Attitude_Yaw_Val, 5, 1, 1, 1)

        self.pushButton_4 = QPushButton(self.grp_YawFiltered)
        self.pushButton_4.setObjectName(u"pushButton_4")

        self.gridLayout_2.addWidget(self.pushButton_4, 8, 3, 1, 1)

        self.pushButton = QPushButton(self.grp_YawFiltered)
        self.pushButton.setObjectName(u"pushButton")

        self.gridLayout_2.addWidget(self.pushButton, 8, 2, 1, 1)

        self.label_5 = QLabel(self.grp_YawFiltered)
        self.label_5.setObjectName(u"label_5")

        self.gridLayout_2.addWidget(self.label_5, 8, 0, 1, 1)

        self.label_6 = QLabel(self.grp_YawFiltered)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setFrameShape(QFrame.Shape.StyledPanel)
        self.label_6.setFrameShadow(QFrame.Shadow.Sunken)

        self.gridLayout_2.addWidget(self.label_6, 8, 1, 1, 1)


        self.horizontalLayout_3DTop.addWidget(self.grp_YawFiltered)


        self.verticalLayout_3DMotion.addLayout(self.horizontalLayout_3DTop)

        self.grp_BLDC_Control = QGroupBox(self.tab_3DMotion)
        self.grp_BLDC_Control.setObjectName(u"grp_BLDC_Control")
        self.horizontalLayout_BLDC = QHBoxLayout(self.grp_BLDC_Control)
        self.horizontalLayout_BLDC.setObjectName(u"horizontalLayout_BLDC")
        self.radio_BLDC_Forward = QRadioButton(self.grp_BLDC_Control)
        self.radio_BLDC_Forward.setObjectName(u"radio_BLDC_Forward")
        self.radio_BLDC_Forward.setChecked(True)

        self.horizontalLayout_BLDC.addWidget(self.radio_BLDC_Forward)

        self.radio_BLDC_Reverse = QRadioButton(self.grp_BLDC_Control)
        self.radio_BLDC_Reverse.setObjectName(u"radio_BLDC_Reverse")

        self.horizontalLayout_BLDC.addWidget(self.radio_BLDC_Reverse)

        self.lbl_BLDC_Speed = QLabel(self.grp_BLDC_Control)
        self.lbl_BLDC_Speed.setObjectName(u"lbl_BLDC_Speed")

        self.horizontalLayout_BLDC.addWidget(self.lbl_BLDC_Speed)

        self.slider_BLDC_Speed = QSlider(self.grp_BLDC_Control)
        self.slider_BLDC_Speed.setObjectName(u"slider_BLDC_Speed")
        self.slider_BLDC_Speed.setMinimumSize(QSize(220, 0))
        self.slider_BLDC_Speed.setMaximum(100)
        self.slider_BLDC_Speed.setOrientation(Qt.Orientation.Horizontal)

        self.horizontalLayout_BLDC.addWidget(self.slider_BLDC_Speed)

        self.spin_BLDC_Speed = QSpinBox(self.grp_BLDC_Control)
        self.spin_BLDC_Speed.setObjectName(u"spin_BLDC_Speed")
        self.spin_BLDC_Speed.setMaximum(100)

        self.horizontalLayout_BLDC.addWidget(self.spin_BLDC_Speed)

        self.btn_BLDC_Apply = QPushButton(self.grp_BLDC_Control)
        self.btn_BLDC_Apply.setObjectName(u"btn_BLDC_Apply")

        self.horizontalLayout_BLDC.addWidget(self.btn_BLDC_Apply)


        self.verticalLayout_3DMotion.addWidget(self.grp_BLDC_Control)

        self.verticalSpacer_3DMotion = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_3DMotion.addItem(self.verticalSpacer_3DMotion)

        self.tabWidget_Main.addTab(self.tab_3DMotion, "")
        self.tab_Env = QWidget()
        self.tab_Env.setObjectName(u"tab_Env")
        self.horizontalLayout_Env = QHBoxLayout(self.tab_Env)
        self.horizontalLayout_Env.setSpacing(12)
        self.horizontalLayout_Env.setObjectName(u"horizontalLayout_Env")
        self.grp_SysEnv = QGroupBox(self.tab_Env)
        self.grp_SysEnv.setObjectName(u"grp_SysEnv")
        self.formLayout_Sys = QFormLayout(self.grp_SysEnv)
        self.formLayout_Sys.setObjectName(u"formLayout_Sys")
        self.formLayout_Sys.setLabelAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.formLayout_Sys.setHorizontalSpacing(10)
        self.formLayout_Sys.setVerticalSpacing(10)
        self.lbl_OS = QLabel(self.grp_SysEnv)
        self.lbl_OS.setObjectName(u"lbl_OS")

        self.formLayout_Sys.setWidget(0, QFormLayout.ItemRole.LabelRole, self.lbl_OS)

        self.lbl_OS_Val = QLabel(self.grp_SysEnv)
        self.lbl_OS_Val.setObjectName(u"lbl_OS_Val")
        self.lbl_OS_Val.setMinimumSize(QSize(0, 26))
        self.lbl_OS_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_OS_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_OS_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Sys.setWidget(0, QFormLayout.ItemRole.FieldRole, self.lbl_OS_Val)

        self.lbl_CPU = QLabel(self.grp_SysEnv)
        self.lbl_CPU.setObjectName(u"lbl_CPU")

        self.formLayout_Sys.setWidget(1, QFormLayout.ItemRole.LabelRole, self.lbl_CPU)

        self.lbl_CPU_Val = QLabel(self.grp_SysEnv)
        self.lbl_CPU_Val.setObjectName(u"lbl_CPU_Val")
        self.lbl_CPU_Val.setMinimumSize(QSize(0, 26))
        self.lbl_CPU_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_CPU_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_CPU_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Sys.setWidget(1, QFormLayout.ItemRole.FieldRole, self.lbl_CPU_Val)

        self.lbl_GPU_Basic = QLabel(self.grp_SysEnv)
        self.lbl_GPU_Basic.setObjectName(u"lbl_GPU_Basic")

        self.formLayout_Sys.setWidget(2, QFormLayout.ItemRole.LabelRole, self.lbl_GPU_Basic)

        self.lbl_GPU_Basic_Val = QLabel(self.grp_SysEnv)
        self.lbl_GPU_Basic_Val.setObjectName(u"lbl_GPU_Basic_Val")
        self.lbl_GPU_Basic_Val.setMinimumSize(QSize(0, 26))
        self.lbl_GPU_Basic_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_GPU_Basic_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_GPU_Basic_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Sys.setWidget(2, QFormLayout.ItemRole.FieldRole, self.lbl_GPU_Basic_Val)

        self.lbl_RAM = QLabel(self.grp_SysEnv)
        self.lbl_RAM.setObjectName(u"lbl_RAM")

        self.formLayout_Sys.setWidget(3, QFormLayout.ItemRole.LabelRole, self.lbl_RAM)

        self.lbl_RAM_Val = QLabel(self.grp_SysEnv)
        self.lbl_RAM_Val.setObjectName(u"lbl_RAM_Val")
        self.lbl_RAM_Val.setMinimumSize(QSize(0, 26))
        self.lbl_RAM_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_RAM_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_RAM_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Sys.setWidget(3, QFormLayout.ItemRole.FieldRole, self.lbl_RAM_Val)


        self.horizontalLayout_Env.addWidget(self.grp_SysEnv)

        self.grp_CompEnv = QGroupBox(self.tab_Env)
        self.grp_CompEnv.setObjectName(u"grp_CompEnv")
        self.verticalLayout_7 = QVBoxLayout(self.grp_CompEnv)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.formLayout_Comp = QFormLayout()
        self.formLayout_Comp.setObjectName(u"formLayout_Comp")
        self.formLayout_Comp.setLabelAlignment(Qt.AlignmentFlag.AlignRight|Qt.AlignmentFlag.AlignTrailing|Qt.AlignmentFlag.AlignVCenter)
        self.formLayout_Comp.setHorizontalSpacing(10)
        self.formLayout_Comp.setVerticalSpacing(10)
        self.lbl_GPU_Count = QLabel(self.grp_CompEnv)
        self.lbl_GPU_Count.setObjectName(u"lbl_GPU_Count")

        self.formLayout_Comp.setWidget(0, QFormLayout.ItemRole.LabelRole, self.lbl_GPU_Count)

        self.lbl_GPU_Count_Val = QLabel(self.grp_CompEnv)
        self.lbl_GPU_Count_Val.setObjectName(u"lbl_GPU_Count_Val")
        self.lbl_GPU_Count_Val.setMinimumSize(QSize(0, 26))
        self.lbl_GPU_Count_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_GPU_Count_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_GPU_Count_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Comp.setWidget(0, QFormLayout.ItemRole.FieldRole, self.lbl_GPU_Count_Val)

        self.lbl_CUDA = QLabel(self.grp_CompEnv)
        self.lbl_CUDA.setObjectName(u"lbl_CUDA")

        self.formLayout_Comp.setWidget(1, QFormLayout.ItemRole.LabelRole, self.lbl_CUDA)

        self.lbl_CUDA_Val = QLabel(self.grp_CompEnv)
        self.lbl_CUDA_Val.setObjectName(u"lbl_CUDA_Val")
        self.lbl_CUDA_Val.setMinimumSize(QSize(0, 26))
        self.lbl_CUDA_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_CUDA_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_CUDA_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Comp.setWidget(1, QFormLayout.ItemRole.FieldRole, self.lbl_CUDA_Val)

        self.lbl_Vision = QLabel(self.grp_CompEnv)
        self.lbl_Vision.setObjectName(u"lbl_Vision")

        self.formLayout_Comp.setWidget(2, QFormLayout.ItemRole.LabelRole, self.lbl_Vision)

        self.lbl_Vision_Val = QLabel(self.grp_CompEnv)
        self.lbl_Vision_Val.setObjectName(u"lbl_Vision_Val")
        self.lbl_Vision_Val.setMinimumSize(QSize(0, 26))
        self.lbl_Vision_Val.setFrameShape(QFrame.Shape.StyledPanel)
        self.lbl_Vision_Val.setFrameShadow(QFrame.Shadow.Sunken)
        self.lbl_Vision_Val.setAlignment(Qt.AlignmentFlag.AlignVCenter)

        self.formLayout_Comp.setWidget(2, QFormLayout.ItemRole.FieldRole, self.lbl_Vision_Val)


        self.verticalLayout_7.addLayout(self.formLayout_Comp)

        self.btn_CheckCompute = QPushButton(self.grp_CompEnv)
        self.btn_CheckCompute.setObjectName(u"btn_CheckCompute")
        self.btn_CheckCompute.setMinimumSize(QSize(0, 40))

        self.verticalLayout_7.addWidget(self.btn_CheckCompute)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_7.addItem(self.verticalSpacer_2)


        self.horizontalLayout_Env.addWidget(self.grp_CompEnv)

        self.tabWidget_Main.addTab(self.tab_Env, "")
        self.tab_FirmwareBurn = QWidget()
        self.tab_FirmwareBurn.setObjectName(u"tab_FirmwareBurn")
        self.verticalLayout_9 = QVBoxLayout(self.tab_FirmwareBurn)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.grp_FirmwareOTA = QGroupBox(self.tab_FirmwareBurn)
        self.grp_FirmwareOTA.setObjectName(u"grp_FirmwareOTA")
        sizePolicy5 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy5.setHorizontalStretch(0)
        sizePolicy5.setVerticalStretch(20)
        sizePolicy5.setHeightForWidth(self.grp_FirmwareOTA.sizePolicy().hasHeightForWidth())
        self.grp_FirmwareOTA.setSizePolicy(sizePolicy5)
        self.verticalLayout_8 = QVBoxLayout(self.grp_FirmwareOTA)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.label_7 = QLabel(self.grp_FirmwareOTA)
        self.label_7.setObjectName(u"label_7")
        sizePolicy5.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy5)

        self.verticalLayout_8.addWidget(self.label_7)

        self.btn_EnterUploadMode = QPushButton(self.grp_FirmwareOTA)
        self.btn_EnterUploadMode.setObjectName(u"btn_EnterUploadMode")
        sizePolicy.setHeightForWidth(self.btn_EnterUploadMode.sizePolicy().hasHeightForWidth())
        self.btn_EnterUploadMode.setSizePolicy(sizePolicy)
        self.btn_EnterUploadMode.setMinimumSize(QSize(0, 32))

        self.verticalLayout_8.addWidget(self.btn_EnterUploadMode)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.lineEdit_HexFilePath = QLineEdit(self.grp_FirmwareOTA)
        self.lineEdit_HexFilePath.setObjectName(u"lineEdit_HexFilePath")
        self.lineEdit_HexFilePath.setMinimumSize(QSize(0, 32))
        self.lineEdit_HexFilePath.setReadOnly(True)
        self.lineEdit_HexFilePath.setCursorMoveStyle(Qt.CursorMoveStyle.LogicalMoveStyle)

        self.horizontalLayout_7.addWidget(self.lineEdit_HexFilePath)

        self.btn_SelectHexFile = QPushButton(self.grp_FirmwareOTA)
        self.btn_SelectHexFile.setObjectName(u"btn_SelectHexFile")
        self.btn_SelectHexFile.setMinimumSize(QSize(0, 32))

        self.horizontalLayout_7.addWidget(self.btn_SelectHexFile)


        self.verticalLayout_8.addLayout(self.horizontalLayout_7)

        self.btn_ConfirmUpload = QPushButton(self.grp_FirmwareOTA)
        self.btn_ConfirmUpload.setObjectName(u"btn_ConfirmUpload")
        self.btn_ConfirmUpload.setMinimumSize(QSize(0, 32))

        self.verticalLayout_8.addWidget(self.btn_ConfirmUpload)

        self.progressBar_FirmwareUpload = QProgressBar(self.grp_FirmwareOTA)
        self.progressBar_FirmwareUpload.setObjectName(u"progressBar_FirmwareUpload")
        self.progressBar_FirmwareUpload.setMinimumSize(QSize(0, 32))
        self.progressBar_FirmwareUpload.setMinimum(0)
        self.progressBar_FirmwareUpload.setMaximum(100)
        self.progressBar_FirmwareUpload.setValue(0)

        self.verticalLayout_8.addWidget(self.progressBar_FirmwareUpload)


        self.verticalLayout_9.addWidget(self.grp_FirmwareOTA)

        self.groupBox_2 = QGroupBox(self.tab_FirmwareBurn)
        self.groupBox_2.setObjectName(u"groupBox_2")
        sizePolicy6 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy6.setHorizontalStretch(0)
        sizePolicy6.setVerticalStretch(200)
        sizePolicy6.setHeightForWidth(self.groupBox_2.sizePolicy().hasHeightForWidth())
        self.groupBox_2.setSizePolicy(sizePolicy6)
        self.formLayout_3 = QFormLayout(self.groupBox_2)
        self.formLayout_3.setObjectName(u"formLayout_3")
        self.label_8 = QLabel(self.groupBox_2)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setMinimumSize(QSize(0, 20))

        self.formLayout_3.setWidget(0, QFormLayout.ItemRole.LabelRole, self.label_8)

        self.label_9 = QLabel(self.groupBox_2)
        self.label_9.setObjectName(u"label_9")
        self.label_9.setMinimumSize(QSize(0, 20))

        self.formLayout_3.setWidget(1, QFormLayout.ItemRole.LabelRole, self.label_9)

        self.label_10 = QLabel(self.groupBox_2)
        self.label_10.setObjectName(u"label_10")
        self.label_10.setMinimumSize(QSize(0, 20))

        self.formLayout_3.setWidget(2, QFormLayout.ItemRole.LabelRole, self.label_10)


        self.verticalLayout_9.addWidget(self.groupBox_2)

        self.verticalSpacer_4 = QSpacerItem(370, 126, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_9.addItem(self.verticalSpacer_4)

        self.tabWidget_Main.addTab(self.tab_FirmwareBurn, "")

        self.verticalLayout_Main.addWidget(self.tabWidget_Main)

        self.grp_Log = QGroupBox(self.centralwidget)
        self.grp_Log.setObjectName(u"grp_Log")
        sizePolicy1.setHeightForWidth(self.grp_Log.sizePolicy().hasHeightForWidth())
        self.grp_Log.setSizePolicy(sizePolicy1)
        self.grp_Log.setMinimumSize(QSize(0, 160))
        self.grp_Log.setMaximumSize(QSize(16777215, 180))
        self.horizontalLayout_Log = QHBoxLayout(self.grp_Log)
        self.horizontalLayout_Log.setObjectName(u"horizontalLayout_Log")
        self.horizontalLayout_Log.setContentsMargins(4, 4, 4, 4)
        self.txt_Log = QPlainTextEdit(self.grp_Log)
        self.txt_Log.setObjectName(u"txt_Log")
        self.txt_Log.setReadOnly(True)

        self.horizontalLayout_Log.addWidget(self.txt_Log)

        self.verticalLayout_LogCtrl = QVBoxLayout()
        self.verticalLayout_LogCtrl.setObjectName(u"verticalLayout_LogCtrl")
        self.chk_AutoScroll = QCheckBox(self.grp_Log)
        self.chk_AutoScroll.setObjectName(u"chk_AutoScroll")
        self.chk_AutoScroll.setChecked(True)

        self.verticalLayout_LogCtrl.addWidget(self.chk_AutoScroll)

        self.btn_ClearLog = QPushButton(self.grp_Log)
        self.btn_ClearLog.setObjectName(u"btn_ClearLog")

        self.verticalLayout_LogCtrl.addWidget(self.btn_ClearLog)

        self.btn_SaveLogFile = QPushButton(self.grp_Log)
        self.btn_SaveLogFile.setObjectName(u"btn_SaveLogFile")

        self.verticalLayout_LogCtrl.addWidget(self.btn_SaveLogFile)

        self.verticalSpacer_Log = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_LogCtrl.addItem(self.verticalSpacer_Log)


        self.horizontalLayout_Log.addLayout(self.verticalLayout_LogCtrl)


        self.verticalLayout_Main.addWidget(self.grp_Log)


        self.horizontalLayout_Main.addLayout(self.verticalLayout_Main)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 1117, 33))
        self.menu_File = QMenu(self.menubar)
        self.menu_File.setObjectName(u"menu_File")
        self.menu_Tools = QMenu(self.menubar)
        self.menu_Tools.setObjectName(u"menu_Tools")
        self.menu_Help = QMenu(self.menubar)
        self.menu_Help.setObjectName(u"menu_Help")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.menubar.addAction(self.menu_File.menuAction())
        self.menubar.addAction(self.menu_Tools.menuAction())
        self.menubar.addAction(self.menu_Help.menuAction())
        self.menu_File.addAction(self.action_Load)
        self.menu_File.addAction(self.action_Save)
        self.menu_File.addSeparator()
        self.menu_File.addAction(self.action_Exit)
        self.menu_Tools.addAction(self.action)
        self.menu_Help.addAction(self.action_Protocol)
        self.menu_Help.addAction(self.action_About)

        self.retranslateUi(MainWindow)

        self.tabWidget_Main.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"\u4eff\u751f\u673a\u5668\u9526\u9ca4\u63a7\u5236\u53f0 V2.5", None))
        self.action_Load.setText(QCoreApplication.translate("MainWindow", u"\u8f7d\u5165\u914d\u7f6e(&L)...", None))
        self.action_Save.setText(QCoreApplication.translate("MainWindow", u"\u4fdd\u5b58\u914d\u7f6e(&S)...", None))
        self.action_Exit.setText(QCoreApplication.translate("MainWindow", u"\u9000\u51fa(&X)", None))
        self.action_OpenSerial.setText(QCoreApplication.translate("MainWindow", u"\u8fde\u63a5\u4e32\u53e3", None))
        self.action_SearchDev.setText(QCoreApplication.translate("MainWindow", u"\u641c\u7d22\u8bbe\u5907", None))
        self.action_Protocol.setText(QCoreApplication.translate("MainWindow", u"\u901a\u4fe1\u534f\u8bae\u8bf4\u660e(&P)", None))
        self.action_About.setText(QCoreApplication.translate("MainWindow", u"\u5173\u4e8e(&A)", None))
        self.action.setText(QCoreApplication.translate("MainWindow", u"\u4e32\u53e3\u8c03\u8bd5\u52a9\u624b", None))
        self.grp_Serial.setTitle(QCoreApplication.translate("MainWindow", u"\u8fde\u63a5\u63a7\u5236\u5668", None))
        self.lbl_Port.setText(QCoreApplication.translate("MainWindow", u"\u672c\u5730\u7aef\u53e3\u53f7\uff1a", None))
        self.combo_Port.setItemText(0, QCoreApplication.translate("MainWindow", u"\u70b9\u51fb\u5237\u65b0", None))

        self.lbl_CtrlVer.setText(QCoreApplication.translate("MainWindow", u"\u63a7\u5236\u5668\u7248\u672c\uff1a", None))
        self.comboBox.setItemText(0, QCoreApplication.translate("MainWindow", u"V2", None))
        self.comboBox.setItemText(1, QCoreApplication.translate("MainWindow", u"V1", None))

        self.lbl_CtrlLink.setText(QCoreApplication.translate("MainWindow", u"\u8fde\u63a5\u72b6\u6001\uff1a", None))
        self.lbl_CtrlLinkState.setText(QCoreApplication.translate("MainWindow", u"\u672a\u8fde\u63a5", None))
        self.btn_OpenSerial.setText(QCoreApplication.translate("MainWindow", u"\u542f\u52a8\u4e32\u53e3(&W)", None))
        self.grp_Controller.setTitle(QCoreApplication.translate("MainWindow", u"\u63a7\u5236\u5668\u53c2\u6570", None))
        self.lbl_CtrlID.setText(QCoreApplication.translate("MainWindow", u"\u63a7\u5236\u5668ID(01-FF)\uff1a", None))
#if QT_CONFIG(tooltip)
        self.lbl_CtrlID_Val.setToolTip(QCoreApplication.translate("MainWindow", u"\u683c\u5f0f\uff1a0x + 4\u4f4dHEX\uff08\u5982 0x00AF\uff09", None))
#endif // QT_CONFIG(tooltip)
        self.lbl_CtrlID_Val.setText(QCoreApplication.translate("MainWindow", u"0x01", None))
        self.lbl_CtrlCh.setText(QCoreApplication.translate("MainWindow", u"\u4fe1\u9053\u53f7(0-83)\uff1a", None))
        self.btn_SetCtrlID.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6eID", None))
        self.btn_SetCtrlCh.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u4fe1\u9053", None))
        self.btn_InitCtrl.setText(QCoreApplication.translate("MainWindow", u"\u521d\u59cb\u5316\u63a7\u5236\u5668", None))
        self.btn_QueryCtrl.setText(QCoreApplication.translate("MainWindow", u"\u67e5\u8be2\u53c2\u6570", None))
        self.groupBox_SecondPWD.setTitle(QCoreApplication.translate("MainWindow", u"\u63a7\u5236\u53e3\u4ee4(\u7528\u4e8e\u914d\u5bf9\u540e\u8ba4\u8bc1)", None))
        self.lineEdit_SecondPWD.setText(QCoreApplication.translate("MainWindow", u"AABB", None))
        self.pushButton_SecondPWD_random.setText(QCoreApplication.translate("MainWindow", u"\u968f\u673a\u53e3\u4ee4", None))
        self.pushButton_SecondPWD_set.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u53e3\u4ee4", None))
        self.grp_Devices.setTitle(QCoreApplication.translate("MainWindow", u"\u8bbe\u5907\u5217\u8868", None))
        self.btn_Search.setText(QCoreApplication.translate("MainWindow", u"\u641c\u7d22\u8bbe\u5907(&R)", None))
        self.btn_AddDevManual.setText(QCoreApplication.translate("MainWindow", u"\u624b\u52a8\u6dfb\u52a0(&M)", None))
        ___qtablewidgetitem = self.table_Devices.horizontalHeaderItem(0)
        ___qtablewidgetitem.setText(QCoreApplication.translate("MainWindow", u"FishID", None));
        ___qtablewidgetitem1 = self.table_Devices.horizontalHeaderItem(1)
        ___qtablewidgetitem1.setText(QCoreApplication.translate("MainWindow", u"\u914d\u5bf9\u72b6\u6001", None));
        self.btn_SelectAll.setText(QCoreApplication.translate("MainWindow", u"\u5168\u9009", None))
        self.btn_SelectNone.setText(QCoreApplication.translate("MainWindow", u"\u5168\u4e0d\u9009", None))
        self.btn_SelectNone_2.setText(QCoreApplication.translate("MainWindow", u"\u914d\u5bf9", None))
        self.btn_SelectNone_3.setText(QCoreApplication.translate("MainWindow", u"\u53d6\u6d88\u914d\u5bf9", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"\u63d0\u793a\uff1a\u914d\u5bf9\u540e\u65e0\u6cd5\u518d\u6b21\u88ab\u914d\u5bf9", None))
        self.grp_Monitor.setTitle(QCoreApplication.translate("MainWindow", u"\u7cfb\u7edf\u76d1\u89c6", None))
        self.btn_GlobalStop_2.setText(QCoreApplication.translate("MainWindow", u"\u6025\u505c", None))
        self.btn_GlobalStop.setText(QCoreApplication.translate("MainWindow", u"\u5168\u90e8\u6025\u505c(&S)", None))
        ___qtablewidgetitem2 = self.table_SysMonitor.horizontalHeaderItem(0)
        ___qtablewidgetitem2.setText(QCoreApplication.translate("MainWindow", u"\u9009\u62e9", None));
        ___qtablewidgetitem3 = self.table_SysMonitor.horizontalHeaderItem(1)
        ___qtablewidgetitem3.setText(QCoreApplication.translate("MainWindow", u"FishID", None));
        ___qtablewidgetitem4 = self.table_SysMonitor.horizontalHeaderItem(2)
        ___qtablewidgetitem4.setText(QCoreApplication.translate("MainWindow", u"\u7535\u538b / \u529f\u7387", None));
        ___qtablewidgetitem5 = self.table_SysMonitor.horizontalHeaderItem(3)
        ___qtablewidgetitem5.setText(QCoreApplication.translate("MainWindow", u"\u7535\u91cf / \u65f6\u95f4", None));
        ___qtablewidgetitem6 = self.table_SysMonitor.horizontalHeaderItem(4)
        ___qtablewidgetitem6.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u884c\u72b6\u6001", None));
        self.lbl_MonitorHint.setText(QCoreApplication.translate("MainWindow", u"\u63d0\u793a\uff1a\u70b9\u51fb\u9009\u4e2d\u540e\u624d\u53ef\u53d1\u9001\u6307\u4ee4\uff0c\u652f\u6301\u591a\u9009", None))
        self.grp_Gear_Ctrl.setTitle(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6/\u8f6c\u5411\u63a7\u5236", None))
        self.lbl_SpeedDial.setText(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\u6863\u4f4d\uff080=\u505c\u6b62\uff0c15=\u6700\u9ad8\uff09", None))
        self.lbl_SpeedMin.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62 0", None))
        self.lbl_SpeedMid.setText(QCoreApplication.translate("MainWindow", u"\u5de1\u822a 8", None))
        self.lbl_SpeedMax.setText(QCoreApplication.translate("MainWindow", u"\u6700\u9ad8 15", None))
        self.spin_GearSpeed.setSuffix(QCoreApplication.translate("MainWindow", u" \u6863", None))
        self.lbl_TurnDial.setText(QCoreApplication.translate("MainWindow", u"\u8f6c\u5411\u6863\u4f4d\uff081=\u6700\u5de6\uff0c8=\u76f4\u884c\uff0c15=\u6700\u53f3\uff09", None))
        self.lbl_TurnLeft.setText(QCoreApplication.translate("MainWindow", u"1 \u5de6\u8f6c", None))
        self.lbl_TurnMid.setText(QCoreApplication.translate("MainWindow", u"8 \u76f4\u884c", None))
        self.lbl_TurnRight.setText(QCoreApplication.translate("MainWindow", u"15 \u53f3\u8f6c", None))
        self.spin_GearTurn.setSuffix(QCoreApplication.translate("MainWindow", u" \u6863", None))
        self.chk_Sync_Gear.setText(QCoreApplication.translate("MainWindow", u"\u5b9e\u65f6\u540c\u6b65\u6307\u4ee4", None))
        self.spin_Sync_Gear_Ms.setSuffix(QCoreApplication.translate("MainWindow", u" ms", None))
        self.btn_SendGear.setText(QCoreApplication.translate("MainWindow", u"\u53d1\u9001\u6307\u4ee4(&G)", None))
        self.btn_SetBootFromGear.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u4e3a\u4e0a\u7535\u8fd0\u52a8\u53c2\u6570", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_SimpleMotion), QCoreApplication.translate("MainWindow", u"\u7b80\u6613\u8fd0\u52a8\u63a7\u5236", None))
        self.grp_Servo_Pos.setTitle(QCoreApplication.translate("MainWindow", u"\u4f4d\u7f6e\u63a7\u5236", None))
        self.radio_S1_Only.setText(QCoreApplication.translate("MainWindow", u"\u4ec5\u63a7 S1", None))
        self.radio_S2_Only.setText(QCoreApplication.translate("MainWindow", u"\u4ec5\u63a7 S2", None))
        self.radio_Dual_Sync.setText(QCoreApplication.translate("MainWindow", u"\u53cc\u673a\u540c\u6b65", None))
        self.lbl_S1.setText(QCoreApplication.translate("MainWindow", u"S1", None))
        self.lbl_S2.setText(QCoreApplication.translate("MainWindow", u"S2", None))
        self.chk_Sync_Servo.setText(QCoreApplication.translate("MainWindow", u"\u5b9e\u65f6\u540c\u6b65\u6307\u4ee4", None))
        self.spin_Sync_Servo_Ms.setSuffix(QCoreApplication.translate("MainWindow", u" ms", None))
        self.btn_SendServo.setText(QCoreApplication.translate("MainWindow", u"\u53d1\u9001\u4f4d\u7f6e(&V)", None))
        self.btn_ServoQuickReset.setText(QCoreApplication.translate("MainWindow", u"\u4e00\u952e\u590d\u4f4d", None))
        self.grp_Servo_Power.setTitle(QCoreApplication.translate("MainWindow", u"\u7535\u6e90\u8f93\u51fa", None))
        self.chk_S1_Pwr.setText(QCoreApplication.translate("MainWindow", u"S1 \u4f9b\u7535", None))
        self.chk_S2_Pwr.setText(QCoreApplication.translate("MainWindow", u"S2 \u4f9b\u7535", None))
        self.btn_SetServoPower.setText(QCoreApplication.translate("MainWindow", u"\u5e94\u7528", None))
        self.grp_Servo_Status.setTitle(QCoreApplication.translate("MainWindow", u"\u8235\u673a\u8fd0\u884c\u76d1\u89c6", None))
        self.btn_QueryServoAll.setText(QCoreApplication.translate("MainWindow", u"\u67e5\u8be2\u5168\u90e8\u72b6\u6001", None))
        self.btn_ResetFault_ServoStat.setText(QCoreApplication.translate("MainWindow", u"\u590d\u4f4d\u635f\u574f\u8235\u673a", None))
        self.lbl_S1_FaultFlag.setText(QCoreApplication.translate("MainWindow", u"S1\u635f\u574f\u6807\u5fd7\u4f4d\uff1a", None))
        self.lbl_S1_FaultFlag_Val.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u884c\u6b63\u5e38", None))
        self.lbl_S2_FaultFlag.setText(QCoreApplication.translate("MainWindow", u"S2\u635f\u574f\u6807\u5fd7\u4f4d\uff1a", None))
        self.lbl_S2_FaultFlag_Val.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u884c\u6b63\u5e38", None))
        self.lbl_S1_Curr.setText(QCoreApplication.translate("MainWindow", u"S1\u8235\u673a\u8f93\u51fa\u7535\u6d41\uff1a", None))
        self.bar_S1_Curr.setFormat(QCoreApplication.translate("MainWindow", u"%v mA", None))
        self.lbl_S2_Curr.setText(QCoreApplication.translate("MainWindow", u"S2\u8235\u673a\u8f93\u51fa\u7535\u6d41\uff1a", None))
        self.bar_S2_Curr.setFormat(QCoreApplication.translate("MainWindow", u"%v mA", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_Servo), QCoreApplication.translate("MainWindow", u"\u8235\u673a\u7efc\u5408\u7ba1\u7406", None))
        self.grp_CPG_Param.setTitle(QCoreApplication.translate("MainWindow", u"CPG \u53c2\u6570\u914d\u7f6e", None))
        self.lbl_Amp.setText(QCoreApplication.translate("MainWindow", u"\u6446\u5e45\uff1a", None))
        self.spin_Amp.setSuffix(QCoreApplication.translate("MainWindow", u" \u00b0", None))
        self.lbl_Freq.setText(QCoreApplication.translate("MainWindow", u"\u9891\u7387\uff1a", None))
        self.spin_Freq.setSuffix(QCoreApplication.translate("MainWindow", u" Hz", None))
        self.lbl_Bias.setText(QCoreApplication.translate("MainWindow", u"\u504f\u7f6e\uff1a", None))
        self.spin_Bias.setSuffix(QCoreApplication.translate("MainWindow", u" \u00b0", None))
        self.grp_CPG_Inter.setTitle(QCoreApplication.translate("MainWindow", u"\u95f4\u6b47\u53c2\u6570", None))
        self.lbl_N.setText(QCoreApplication.translate("MainWindow", u"\u6446\u52a8 N \u6b21\uff1a", None))
        self.lbl_Coast.setText(QCoreApplication.translate("MainWindow", u"\u6ed1\u884c\u65f6\u95f4(s)\uff1a", None))
        self.chk_Sync_CPG.setText(QCoreApplication.translate("MainWindow", u"\u5b9e\u65f6\u540c\u6b65\u6307\u4ee4", None))
        self.spin_Sync_CPG_Ms.setSuffix(QCoreApplication.translate("MainWindow", u" ms", None))
        self.btn_SendCPG.setText(QCoreApplication.translate("MainWindow", u"\u53d1\u9001\u6307\u4ee4(&C)", None))
        self.btn_SetBootFromCPG.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u4e3a\u4e0a\u7535\u8fd0\u52a8\u53c2\u6570", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_AdvMotion), QCoreApplication.translate("MainWindow", u"\u9ad8\u9636\u8fd0\u52a8\u63a7\u5236", None))
        self.lbl_PlayId.setText(QCoreApplication.translate("MainWindow", u"\u5e8f\u53f7 (1-255)\uff1a", None))
        self.btn_PlayStart.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u59cb\u8868\u6f14(&P)", None))
        self.btn_PlayStop.setText(QCoreApplication.translate("MainWindow", u"\u505c\u6b62\u8868\u6f14(&T)", None))
        self.btn_SetBootFromPlay.setText(QCoreApplication.translate("MainWindow", u"\u8bbe\u7f6e\u4e3a\u4e0a\u7535\u8fd0\u52a8\u53c2\u6570", None))
        ___qtablewidgetitem7 = self.table_PlayDesc.horizontalHeaderItem(0)
        ___qtablewidgetitem7.setText(QCoreApplication.translate("MainWindow", u"\u5e8f\u53f7", None));
        ___qtablewidgetitem8 = self.table_PlayDesc.horizontalHeaderItem(1)
        ___qtablewidgetitem8.setText(QCoreApplication.translate("MainWindow", u"\u52a8\u4f5c\u63cf\u8ff0", None));

        __sortingEnabled = self.table_PlayDesc.isSortingEnabled()
        self.table_PlayDesc.setSortingEnabled(False)
        ___qtablewidgetitem9 = self.table_PlayDesc.item(0, 0)
        ___qtablewidgetitem9.setText(QCoreApplication.translate("MainWindow", u"1", None));
        ___qtablewidgetitem10 = self.table_PlayDesc.item(0, 1)
        ___qtablewidgetitem10.setText(QCoreApplication.translate("MainWindow", u"\u5feb\u901f\u51b2\u523a\u52a8\u4f5c", None));
        ___qtablewidgetitem11 = self.table_PlayDesc.item(1, 0)
        ___qtablewidgetitem11.setText(QCoreApplication.translate("MainWindow", u"2", None));
        ___qtablewidgetitem12 = self.table_PlayDesc.item(1, 1)
        ___qtablewidgetitem12.setText(QCoreApplication.translate("MainWindow", u"8\u5b57\u56de\u65cb\u6e38\u52a8", None));
        ___qtablewidgetitem13 = self.table_PlayDesc.item(2, 0)
        ___qtablewidgetitem13.setText(QCoreApplication.translate("MainWindow", u"3", None));
        ___qtablewidgetitem14 = self.table_PlayDesc.item(2, 1)
        ___qtablewidgetitem14.setText(QCoreApplication.translate("MainWindow", u"\u6389\u5934", None));
        self.table_PlayDesc.setSortingEnabled(__sortingEnabled)

        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_Play), QCoreApplication.translate("MainWindow", u"\u8868\u6f14\u6a21\u5f0f", None))
        self.grp_EditFishID.setTitle(QCoreApplication.translate("MainWindow", u"\u4fee\u6539\u8bbe\u5907\u4fe1\u606f", None))
        self.label_EditFishID.setText(QCoreApplication.translate("MainWindow", u"FishID\uff1a", None))
#if QT_CONFIG(tooltip)
        self.txt_F_ID.setToolTip(QCoreApplication.translate("MainWindow", u"\u26a0 \u614e\u91cd\u4fee\u6539\uff0c\u4fee\u6539\u540e\u9700\u91cd\u65b0\u8fde\u63a5\u3002\u683c\u5f0f\uff1a0x + 4\u4f4dHEX\uff08\u5982 0x00AF\uff09", None))
#endif // QT_CONFIG(tooltip)
        self.txt_F_ID.setText(QCoreApplication.translate("MainWindow", u"0x0001", None))
        self.btn_EditFishID.setText(QCoreApplication.translate("MainWindow", u"\u4fee\u6539FishID", None))
        self.label_EditCh.setText(QCoreApplication.translate("MainWindow", u"\u4fe1\u9053\u53f7\uff1a", None))
        self.btn_EditFishCh.setText(QCoreApplication.translate("MainWindow", u"\u4fee\u6539\u4fe1\u9053\u53f7", None))
        self.label_EditPwd.setText(QCoreApplication.translate("MainWindow", u"\u5bc6\u3000\u7801\uff1a", None))
#if QT_CONFIG(tooltip)
        self.txt_F_Pwd.setToolTip(QCoreApplication.translate("MainWindow", u"\u683c\u5f0f\uff1a0x + 4\u4f4dHEX\uff08\u5982 0x0000\uff09", None))
#endif // QT_CONFIG(tooltip)
        self.txt_F_Pwd.setText(QCoreApplication.translate("MainWindow", u"0x0000", None))
        self.btn_EditFishPwd.setText(QCoreApplication.translate("MainWindow", u"\u4fee\u6539\u5bc6\u7801", None))
        self.grp_Startup_Basic.setTitle(QCoreApplication.translate("MainWindow", u"\u5f00\u673a\u53c2\u6570", None))
        self.label_BFirmwareVer.setText(QCoreApplication.translate("MainWindow", u"\u56fa\u4ef6\u8f6f\u4ef6\u7248\u672c\uff1a", None))
        self.spin_F_ServoBiasS1.setSuffix(QCoreApplication.translate("MainWindow", u" \u00b0", None))
        self.lbl_BProtocolVer_Val.setText(QCoreApplication.translate("MainWindow", u"V2.1", None))
        self.label_BProtocolVer.setText(QCoreApplication.translate("MainWindow", u"\u901a\u4fe1\u534f\u8bae\u7248\u672c\uff1a", None))
        self.label_BServoBiasS1.setText(QCoreApplication.translate("MainWindow", u"S1\u8235\u673a\u5b89\u88c5\u504f\u5dee\uff1a", None))
        self.label_BOverCurrentTimeout.setText(QCoreApplication.translate("MainWindow", u"\u8fc7\u6d41\u7ef4\u6301\u8d85\u65f6\u65f6\u95f4\uff1a", None))
        self.lbl_BFirmwareVer_Val.setText(QCoreApplication.translate("MainWindow", u"V2.15", None))
        self.spin_F_OverCurrentTimeout.setSuffix(QCoreApplication.translate("MainWindow", u" s", None))
        self.combo_F_Mode.setItemText(0, QCoreApplication.translate("MainWindow", u"0: \u4fdd\u6301\u590d\u4f4d (\u9759\u6b62)", None))
        self.combo_F_Mode.setItemText(1, QCoreApplication.translate("MainWindow", u"1: CPG \u6e38\u52a8", None))
        self.combo_F_Mode.setItemText(2, QCoreApplication.translate("MainWindow", u"2: \u95f4\u6b47\u6e38\u52a8", None))
        self.combo_F_Mode.setItemText(3, QCoreApplication.translate("MainWindow", u"4: \u8868\u6f14\u6a21\u5f0f", None))
        self.combo_F_Mode.setItemText(4, QCoreApplication.translate("MainWindow", u"5: \u6863\u4f4d\u6a21\u5f0f", None))
        self.combo_F_Mode.setItemText(5, QCoreApplication.translate("MainWindow", u"7: \u4fdd\u62a4\u6a21\u5f0f (\u65e0\u529b\u77e9)", None))

        self.label_Warn_2.setText(QCoreApplication.translate("MainWindow", u"\u26a0 \u8c28\u614e\u4fee\u6539", None))
        self.label_BMode.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u673a\u9ed8\u8ba4\u6a21\u5f0f\uff1a", None))
        self.label_BReply.setText(QCoreApplication.translate("MainWindow", u"\u6d88\u606f\u56de\u590d\uff1a", None))
#if QT_CONFIG(tooltip)
        self.chk_F_Reply.setToolTip(QCoreApplication.translate("MainWindow", u"\u9ed8\u8ba4\u5f00\u542f\u56de\u590d\u65b9\u4fbf\u8c03\u8bd5\uff0c\u5173\u95ed\u540e\u9700\u53d1\u7279\u5b9a\u6307\u4ee4\u624d\u53ef\u5524\u9192", None))
#endif // QT_CONFIG(tooltip)
        self.chk_F_Reply.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u673a\u81ea\u52a8\u5f00\u542f\u6d88\u606f\u56de\u590d", None))
        self.label_BServoBiasS2.setText(QCoreApplication.translate("MainWindow", u"S2\u8235\u673a\u5b89\u88c5\u504f\u5dee\uff1a", None))
        self.spin_F_ServoBiasS2.setSuffix(QCoreApplication.translate("MainWindow", u" \u00b0", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"\u5de6- \u53f3+", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"\u5de6- \u53f3+", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"\u8d85\u65f6\u540e\u5173\u95ed\u8f93\u51fa", None))
        self.btn_FlashRead.setText(QCoreApplication.translate("MainWindow", u"\u8bfb\u53d6\u8bbe\u5907\u914d\u7f6e(&D)", None))
        self.btn_FlashSave.setText(QCoreApplication.translate("MainWindow", u"\u4fdd\u5b58\u914d\u7f6e\u5230\u8bbe\u5907(&K)", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_BootConfig), QCoreApplication.translate("MainWindow", u"\u5f00\u673a\u53c2\u6570\u8bbe\u7f6e", None))
        self.grp_Diag.setTitle(QCoreApplication.translate("MainWindow", u"\u7ef4\u62a4\u547d\u4ee4", None))
        self.btn_ResetFault_6.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u542f / \u5173\u95ed\u6d88\u606f\u56de\u590d", None))
        self.btn_ResetFault.setText(QCoreApplication.translate("MainWindow", u"\u67e5\u8be2\u8fd0\u884c\u72b6\u6001 / \u7535\u538b / \u529f\u7387 / \u7535\u91cf / \u65f6\u95f4", None))
        self.btn_ResetFault_5.setText(QCoreApplication.translate("MainWindow", u"\u67e5\u8be2 Flash \u5185\u6570\u636e", None))
        self.btn_FactoryReset.setText(QCoreApplication.translate("MainWindow", u"\u6062\u590d\u51fa\u5382\u8bbe\u7f6e", None))
        self.grp_AutoReport.setTitle(QCoreApplication.translate("MainWindow", u"\u72b6\u6001\u81ea\u52a8\u56de\u4f20\u914d\u7f6e", None))
        self.lbl_ReportItems.setText(QCoreApplication.translate("MainWindow", u"\u56de\u4f20\u72b6\u6001\u9009\u9879\uff1a", None))
        self.chk_Rpt_Servo.setText(QCoreApplication.translate("MainWindow", u"\u8235\u673a\u8f93\u51fa\u7535\u6d41", None))
        self.chk_Rpt_Motion.setText(QCoreApplication.translate("MainWindow", u"\u8fd0\u884c\u72b6\u6001 / \u7535\u538b / \u529f\u7387 / \u7535\u91cf / \u65f6\u95f4", None))
        self.lbl_Ms.setText(QCoreApplication.translate("MainWindow", u"\u56de\u62a5\u95f4\u9694(ms)\uff1a", None))
        self.radio_Rpt_Off.setText(QCoreApplication.translate("MainWindow", u"\u5173\u95ed\u81ea\u52a8\u56de\u62a5", None))
        self.radio_Rpt_On.setText(QCoreApplication.translate("MainWindow", u"\u5f00\u542f\u81ea\u52a8\u56de\u62a5", None))
        self.btn_SetAutoReport.setText(QCoreApplication.translate("MainWindow", u"\u5e94\u7528\u8bbe\u7f6e", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_Config), QCoreApplication.translate("MainWindow", u"\u9ad8\u7ea7\u8bbe\u7f6e", None))
        self.grp_IMU6Axis.setTitle(QCoreApplication.translate("MainWindow", u"\u516d\u8f74 IMU \u6570\u636e", None))
        self.lbl_Gyro_Status_Val.setText(QCoreApplication.translate("MainWindow", u"\u6b63\u5e38", None))
        self.lbl_IMU_GZ_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 \u00b0/s", None))
        self.lbl_IMU_GX_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 \u00b0/s", None))
        self.lbl_IMU_AX_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 g", None))
        self.lbl_IMU_AZ_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 g", None))
        self.lbl_IMU_AX.setText(QCoreApplication.translate("MainWindow", u"\u52a0\u901f\u5ea6 X\uff1a", None))
        self.lbl_IMU_AY.setText(QCoreApplication.translate("MainWindow", u"\u52a0\u901f\u5ea6 Y\uff1a", None))
        self.lbl_IMU_GY_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 \u00b0/s", None))
        self.lbl_IMU_GZ.setText(QCoreApplication.translate("MainWindow", u"\u89d2\u901f\u5ea6 Z\uff1a", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"\u67e5\u8be2\u6570\u636e", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"\u81ea\u52a8\u56de\u4f20", None))
        self.lbl_Gyro_Status.setText(QCoreApplication.translate("MainWindow", u"\u9640\u87ba\u4eea\u72b6\u6001\uff1a", None))
        self.lbl_IMU_GY.setText(QCoreApplication.translate("MainWindow", u"\u89d2\u901f\u5ea6 Y\uff1a", None))
        self.lbl_IMU_AZ.setText(QCoreApplication.translate("MainWindow", u"\u52a0\u901f\u5ea6 Z\uff1a", None))
        self.lbl_IMU_GX.setText(QCoreApplication.translate("MainWindow", u"\u89d2\u901f\u5ea6 X\uff1a", None))
        self.lbl_IMU_AY_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 g", None))
        self.grp_YawFiltered.setTitle(QCoreApplication.translate("MainWindow", u"\u5904\u7406\u540e\u6570\u636e", None))
        self.lbl_Attitude_Roll.setText(QCoreApplication.translate("MainWindow", u"\u59ff\u6001\u89d2 Roll\uff1a", None))
        self.lbl_Attitude_Pitch_Val.setText(QCoreApplication.translate("MainWindow", u"0.0 \u00b0", None))
        self.lbl_YawFiltered_Hint.setText(QCoreApplication.translate("MainWindow", u"\u9c7c\u5934\u671d\u5411\u89d2\uff1a", None))
        self.lbl_LinearAccel.setText(QCoreApplication.translate("MainWindow", u"\u7ebf\u6027\u52a0\u901f\u5ea6\uff1a", None))
        self.lbl_DepthInfo_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 m", None))
        self.lbl_Attitude_Pitch.setText(QCoreApplication.translate("MainWindow", u"\u59ff\u6001\u89d2 Pitch\uff1a", None))
        self.lbl_DepthInfo.setText(QCoreApplication.translate("MainWindow", u"\u6df1\u5ea6\u4fe1\u606f\uff1a", None))
        self.lbl_RealtimeFreq.setText(QCoreApplication.translate("MainWindow", u"\u5b9e\u65f6\u6446\u52a8\u9891\u7387\uff1a", None))
        self.lbl_LinearAccel_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 m/s\u00b2", None))
        self.lbl_YawFiltered_Val.setText(QCoreApplication.translate("MainWindow", u"0.0\u00b0", None))
        self.lbl_DepthSensor_Status.setText(QCoreApplication.translate("MainWindow", u"\u6df1\u5ea6\u4f20\u611f\u5668\uff1a", None))
        self.lbl_RealtimeFreq_Val.setText(QCoreApplication.translate("MainWindow", u"0.00 Hz", None))
        self.lbl_Attitude_Yaw.setText(QCoreApplication.translate("MainWindow", u"\u59ff\u6001\u89d2 Yaw\uff1a", None))
        self.lbl_DepthSensor_Status_Val.setText(QCoreApplication.translate("MainWindow", u"\u672a\u8fde\u63a5", None))
        self.lbl_Attitude_Roll_Val.setText(QCoreApplication.translate("MainWindow", u"0.0 \u00b0", None))
        self.lbl_Attitude_Yaw_Val.setText(QCoreApplication.translate("MainWindow", u"0.0 \u00b0", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"\u81ea\u52a8\u56de\u4f20", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"\u67e5\u8be2\u6570\u636e", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"\u9c7c\u5934\u4fef\u4ef0\u89d2\uff1a", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"0 \u00b0", None))
        self.grp_BLDC_Control.setTitle(QCoreApplication.translate("MainWindow", u"\u65e0\u5237\u7535\u673a\u63a7\u5236", None))
        self.radio_BLDC_Forward.setText(QCoreApplication.translate("MainWindow", u"\u6b63\u8f6c", None))
        self.radio_BLDC_Reverse.setText(QCoreApplication.translate("MainWindow", u"\u53cd\u8f6c", None))
        self.lbl_BLDC_Speed.setText(QCoreApplication.translate("MainWindow", u"\u901f\u5ea6\uff1a", None))
        self.spin_BLDC_Speed.setSuffix(QCoreApplication.translate("MainWindow", u" %", None))
        self.btn_BLDC_Apply.setText(QCoreApplication.translate("MainWindow", u"\u5e94\u7528\u7535\u673a\u63a7\u5236", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_3DMotion), QCoreApplication.translate("MainWindow", u"\u4e09\u7ef4\u8fd0\u52a8", None))
        self.grp_SysEnv.setTitle(QCoreApplication.translate("MainWindow", u"\u7cfb\u7edf\u73af\u5883", None))
        self.lbl_OS.setText(QCoreApplication.translate("MainWindow", u"\u64cd\u4f5c\u7cfb\u7edf\uff1a", None))
        self.lbl_OS_Val.setText(QCoreApplication.translate("MainWindow", u"\u68c0\u6d4b\u4e2d...", None))
        self.lbl_CPU.setText(QCoreApplication.translate("MainWindow", u"CPU\u578b\u53f7\uff1a", None))
        self.lbl_CPU_Val.setText(QCoreApplication.translate("MainWindow", u"\u68c0\u6d4b\u4e2d...", None))
        self.lbl_GPU_Basic.setText(QCoreApplication.translate("MainWindow", u"\u663e\u5361\u578b\u53f7\uff1a", None))
        self.lbl_GPU_Basic_Val.setText(QCoreApplication.translate("MainWindow", u"\u68c0\u6d4b\u4e2d...", None))
        self.lbl_RAM.setText(QCoreApplication.translate("MainWindow", u"\u5185\u5b58\u5bb9\u91cf\uff1a", None))
        self.lbl_RAM_Val.setText(QCoreApplication.translate("MainWindow", u"\u68c0\u6d4b\u4e2d...", None))
        self.grp_CompEnv.setTitle(QCoreApplication.translate("MainWindow", u"\u7b97\u529b\u73af\u5883", None))
        self.lbl_GPU_Count.setText(QCoreApplication.translate("MainWindow", u"GPU\u6570\u91cf\uff1a", None))
        self.lbl_GPU_Count_Val.setText(QCoreApplication.translate("MainWindow", u"--", None))
        self.lbl_CUDA.setText(QCoreApplication.translate("MainWindow", u"CUDA\u7248\u672c\uff1a", None))
        self.lbl_CUDA_Val.setText(QCoreApplication.translate("MainWindow", u"--", None))
        self.lbl_Vision.setText(QCoreApplication.translate("MainWindow", u"\u5168\u5c40\u89c6\u89c9\u652f\u6301\uff1a", None))
        self.lbl_Vision_Val.setText(QCoreApplication.translate("MainWindow", u"--", None))
        self.btn_CheckCompute.setText(QCoreApplication.translate("MainWindow", u"\u68c0\u67e5\u7b97\u529b\u73af\u5883", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_Env), QCoreApplication.translate("MainWindow", u"\u7cfb\u7edf\u73af\u5883\u68c0\u67e5", None))
        self.grp_FirmwareOTA.setTitle(QCoreApplication.translate("MainWindow", u"\u70e7\u5f55\u987a\u5e8f", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"\u8bf7\u6309\u987a\u5e8f\u64cd\u4f5c\uff1a1) \u70b9\u51fb\u201c\u8fdb\u5165\u4e0a\u4f20\u6a21\u5f0f\u201d \u2192 2) \u9009\u62e9 HEX \u6587\u4ef6 \u2192 3) \u70b9\u51fb\u201c\u786e\u8ba4\u4e0a\u4f20\u201d\u3002", None))
        self.btn_EnterUploadMode.setText(QCoreApplication.translate("MainWindow", u"\u8fdb\u5165\u4e0a\u4f20\u6a21\u5f0f", None))
        self.lineEdit_HexFilePath.setPlaceholderText(QCoreApplication.translate("MainWindow", u"\u8bf7\u9009\u62e9\u8981\u4e0a\u4f20\u7684 .hex \u6587\u4ef6", None))
        self.btn_SelectHexFile.setText(QCoreApplication.translate("MainWindow", u"\u9009\u62e9HEX\u6587\u4ef6", None))
        self.btn_ConfirmUpload.setText(QCoreApplication.translate("MainWindow", u"\u786e\u8ba4\u4e0a\u4f20", None))
        self.progressBar_FirmwareUpload.setFormat(QCoreApplication.translate("MainWindow", u"\u4e0a\u4f20\u8fdb\u5ea6\uff1a%p%", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainWindow", u"\u91cd\u8981\u63d0\u793a", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"\u2022 \u4e0a\u4f20\u8fc7\u7a0b\u4e2d\u8bf7\u52ff\u65ad\u7535\u3001\u52ff\u62d4\u6389\u63a7\u5236\u5668\u6216\u4e32\u53e3\u8bbe\u5907\u3002", None))
        self.label_9.setText(QCoreApplication.translate("MainWindow", u"\u2022 \u8bf7\u786e\u4fdd\u5df2\u9009\u62e9\u6b63\u786e\u7684 HEX \u6587\u4ef6\u4e0e\u76ee\u6807\u8bbe\u5907\uff0c\u4e0a\u4f20\u4e2d\u8bf7\u52ff\u5207\u6362\u7aef\u53e3\u3002", None))
        self.label_10.setText(QCoreApplication.translate("MainWindow", u"\u2022 \u4e0a\u4f20\u5b8c\u6210\u524d\u4e0d\u8981\u5173\u95ed\u8f6f\u4ef6\uff1b\u82e5\u5931\u8d25\uff0c\u8bf7\u91cd\u65b0\u8fdb\u5165\u4e0a\u4f20\u6a21\u5f0f\u540e\u518d\u5c1d\u8bd5\u3002", None))
        self.tabWidget_Main.setTabText(self.tabWidget_Main.indexOf(self.tab_FirmwareBurn), QCoreApplication.translate("MainWindow", u"\u70e7\u5f55\u7a0b\u5e8f", None))
        self.grp_Log.setTitle(QCoreApplication.translate("MainWindow", u"\u901a\u8baf\u65e5\u5fd7", None))
        self.txt_Log.setPlainText("")
        self.chk_AutoScroll.setText(QCoreApplication.translate("MainWindow", u"\u81ea\u52a8\u6eda\u52a8\u5230\u5e95\u90e8", None))
        self.btn_ClearLog.setText(QCoreApplication.translate("MainWindow", u"\u6e05\u7a7a\u65e5\u5fd7(&L)", None))
        self.btn_SaveLogFile.setText(QCoreApplication.translate("MainWindow", u"\u4fdd\u5b58\u81f3\u6587\u4ef6(&F)", None))
        self.menu_File.setTitle(QCoreApplication.translate("MainWindow", u"\u6587\u4ef6(F)", None))
        self.menu_Tools.setTitle(QCoreApplication.translate("MainWindow", u"\u5de5\u5177(T)", None))
        self.menu_Help.setTitle(QCoreApplication.translate("MainWindow", u"\u5e2e\u52a9(H)", None))
    # retranslateUi

