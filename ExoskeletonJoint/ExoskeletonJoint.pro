QT       += core gui
QT       += serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    motorcontrol.cpp \
    robotcontrol.cpp \
    serialportemg.cpp \
    serialportencoder.cpp \
    serialportmotor.cpp \
    widget.cpp

HEADERS += \
    include/IOPort/IOPort.h \
    include/crc/crc_ccitt.h \
    include/serialPort/SerialPort.h \
    include/serialPort/include/errorClass.h \
    include/unitreeMotor/include/motor_msg.h \
    include/unitreeMotor/unitreeMotor.h \
    motorcontrol.h \
    robotcontrol.h \
    serialportemg.h \
    serialportencoder.h \
    serialportmotor.h \
    widget.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/lib/release/ -lUnitreeMotorSDK_M80106_Linux64
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/lib/debug/ -lUnitreeMotorSDK_M80106_Linux64
else:unix: LIBS += -L$$PWD/lib/ -lUnitreeMotorSDK_M80106_Linux64

INCLUDEPATH += $$PWD/''
DEPENDPATH += $$PWD/''
